/* touchscreen/ektf2k_kthread_mtk.c - ELAN EKTF2K touchscreen driver
 * for MTK65xx serial platform.
 *
 * Copyright (C) 2012 Elan Microelectronics Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * 2011/12/06: The first release, version 0x0001
 * 2012/2/15:  The second release, version 0x0002 for new bootcode
 * 2012/5/8:   Release version 0x0003 for china market
 *             Integrated 2 and 5 fingers driver code together and
 *             auto-mapping resolution.
 * 2012/8/24:  MTK version
 * 2013/2/1:   Release for MTK6589/6577/6575/6573/6513 Platform
 *             For MTK6575/6573/6513, please disable both of ELAN_MTK6577 and MTK6589DMA.
 *                          It will use 8+8+2 received packet protocol
 *             For MTK6577, please enable ELAN_MTK6577 and disable MTK6589DMA.
 *                          It will use Elan standard protocol (18bytes of protocol).
 *             For MTK6589, please enable both of ELAN_MTK6577 and MTK6589DMA.
 * 2013/5/15   Fixed MTK6589_DMA issue.
 */
//#define SOFTKEY_AXIS_VER
//#define ELAN_TEN_FINGERS
//#define   MTK6589_DMA
//#define   ELAN_MTK6577
#define   MTK6752_DMA
#define   ELAN_MTK6752
//#define ELAN_BUTTON
//#define TPD_HAVE_BUTTON
//#define ELAN_TEN_FINGERS	// Thunder Open flag for Ten fingers flag 20141229 
#define DEVICE_NAME "elan_ktf2k"

#if defined( MTK6589_DMA ) || defined( MTK6752_DMA )
#define   ELAN_DMA_MODE
#endif

#ifdef ELAN_TEN_FINGERS
#define   PACKET_SIZE       44    /* support 10 fingers packet */
#else
//<<Mel - 4/10, Modify firmware support 2 fingers.
//#define   PACKET_SIZE       8     /* support 2 fingers packet  */
#define PACKET_SIZE       18    /* support 5 fingers packet  */
//>>Mel - 4/10, Modify firmware support 2 fingers.
#endif

//colby add start
#define USE_ANDROID_M
//colby add end

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
//#include <linux/earlysuspend.h> //colby mask 20151001
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>

//colby add start
#include <linux/regulator/consumer.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif
//colby add end

#include <linux/dma-mapping.h>

//colby add start
#ifndef USE_ANDROID_M
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#endif
//colby add end

//colby add start
#ifndef USE_ANDROID_M
#if !defined( TPD_NO_GPIO )
#include "cust_gpio_usage.h"
#endif
#endif
//colby add end

/* For linux 2.6.36.3 */
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>

/* DMA */
#include <linux/dma-mapping.h>

#include "tpd.h"
//#include "mach/eint.h"

#include "ektf2k_mtk.h"

//colby add start
#ifndef USE_ANDROID_M
#include <cust_eint.h>
#endif
//colby add end

//if TP FW is updating, dont sleep in suspend - start
#include <linux/wakelock.h>
//if TP FW is updating, dont sleep in suspend - end

//colby add start
#ifdef USE_ANDROID_M
#define PINCTRL_STATE_ACTIVE    "pmx_ts_active"
#define PINCTRL_STATE_SUSPEND   "pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE   "pmx_ts_release"
#endif
//colby add end

#define   ELAN_DEBUG 1

#define   I2C_ELAN_DEV_ADDR     (0x10) //(0x10)  //(0x15)  //(0x5D)
#define   I2C_ELAN_SLAVE_ADDR   (I2C_ELAN_DEV_ADDR<<1)  /* 0x2A(0x15<<1), 0x40(0x20<<1), 0xBA(0x5D<<1) */
#define   I2C_ELAN_BUS          0 //I2C0

#define   PWR_STATE_DEEP_SLEEP  0
#define   PWR_STATE_NORMAL      1
#define   PWR_STATE_MASK        BIT(3)

#define   CMD_S_PKT             (0x52)
#define   CMD_R_PKT             (0x53)
#define   CMD_W_PKT             (0x54)

#define   HELLO_PKT             (0x55)
#define   FIVE_FINGERS_PKT      (0x5D)

#define   TWO_FINGERS_PKT       (0x5A)
#define   TEN_FINGERS_PKT       (0x62)
#define   MTK_FINGERS_PKT       (0x6D)  /** 2 Fingers: 5A 5 Fingers 5D, 10 Fingers: 62 **/
#define   GESTURE_PKT       (0x88)

#define   RESET_PKT             (0x77)
#define   CALIB_PKT             (0xA8)


#define   TPD_OK                0
//#define HAVE_TOUCH_KEY

//#define   LCT_VIRTUAL_KEY

#if defined( ELAN_DMA_MODE ) //?
static uint8_t  * gpDMABuf_va = NULL;
//static uint32_t   gpDMABuf_pa = NULL;
static dma_addr_t   gpDMABuf_pa = 0;
#endif

/** 0604 add -start **/
//#define ESD_CHECK
#if defined( ESD_CHECK )
  static int    have_interrupts = 0;
  static struct workqueue_struct *esd_wq = NULL;
  static struct delayed_work      esd_work;
  static unsigned long  delay = 2*HZ;

//declare function
  static void elan_touch_esd_func(struct work_struct *work);
#endif
/** 0604 add -end **/

#if defined( TPD_HAVE_BUTTON )
#define   TPD_KEY_COUNT         3
#define   TPD_KEYS              { KEY_MENU, KEY_HOMEPAGE, KEY_BACK }
#if defined( LCT_VIRTUAL_KEY )
#define   TPD_KEYS_DIM          {{ 107, 1370, 109, TPD_BUTTON_HEIGH }, \
                                 { 365, 1370, 109, TPD_BUTTON_HEIGH }, \
                                 { 617, 1370, 102, TPD_BUTTON_HEIGH }}
#endif

//static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS; //colby mask 20151001
#if defined( LCT_VIRTUAL_KEY )
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#endif /* End.. (TPD_HAVE_BUTTON) */

//<<Mel - 4/10, Add pin define.
#define   CUST_EINT_POLARITY_LOW            0
#define   CUST_EINT_TOUCH_PANEL_SENSITIVE   1
//Mel - 4/10, Add pin define>>.

// modify
#define   SYSTEM_RESET_PIN_SR   135

//Add these Define

//<2014/13/44-Yuting Shih. Masked for compile error if non-firmware.
#define   IAP_PORTION           1   //1	//If as ture, Open Auto update when Handset power on 20141229
#define   FW_UPDATE_PROBE_CEI           1  //whether do FW update when driver probe
#define	CHECK_HW_ID				1 // check HW ID
#define	CHECK_CTP_ID				1 // check CTP ID
//>2014/13/44-Yuting Shih.
#define   PAGERETRY             30
#define   IAPRESTART            5
#define   CMD_54001234          0 
//1014 start
#define NO_DEBUG       0
#define DEBUG_MSG     0
static int debug_flag = DEBUG_MSG;
#define touch_debug(level, ...) \
        do { \
             if (debug_flag > (level)) \
                 printk("[elan_debug]:" __VA_ARGS__); \
        } while (0)
//1014 end

#define ELAN_PATH "/data/rawdata.sh"
/*
static uint8_t file_fw_data_old[] = {
#include "eKTHF2152_VL60_test.i"  
};

static uint8_t file_fw_data_new[] = {
#include "E2-3_FW55A1_Test.i"  
};

static uint8_t *file_fw_data = file_fw_data_old;
*/


// For Firmware Update
#define   ELAN_IOCTLID              0xD0
#define   IOCTL_I2C_SLAVE           _IOW(ELAN_IOCTLID,  1, int)
#define   IOCTL_MAJOR_FW_VER        _IOR(ELAN_IOCTLID,  2, int)
#define   IOCTL_MINOR_FW_VER        _IOR(ELAN_IOCTLID,  3, int)
#define   IOCTL_RESET               _IOR(ELAN_IOCTLID,  4, int)
#define   IOCTL_IAP_MODE_LOCK       _IOR(ELAN_IOCTLID,  5, int)
#define   IOCTL_CHECK_RECOVERY_MODE _IOR(ELAN_IOCTLID,  6, int)
#define   IOCTL_FW_VER              _IOR(ELAN_IOCTLID,  7, int)
#define   IOCTL_X_RESOLUTION        _IOR(ELAN_IOCTLID,  8, int)
#define   IOCTL_Y_RESOLUTION        _IOR(ELAN_IOCTLID,  9, int)
#define   IOCTL_FW_ID               _IOR(ELAN_IOCTLID, 10, int)
#define   IOCTL_ROUGH_CALIBRATE     _IOR(ELAN_IOCTLID, 11, int)
#define   IOCTL_IAP_MODE_UNLOCK     _IOR(ELAN_IOCTLID, 12, int)
#define   IOCTL_I2C_INT             _IOR(ELAN_IOCTLID, 13, int)
#define   IOCTL_RESUME              _IOR(ELAN_IOCTLID, 14, int)
#define   IOCTL_POWER_LOCK          _IOR(ELAN_IOCTLID, 15, int)
#define   IOCTL_POWER_UNLOCK        _IOR(ELAN_IOCTLID, 16, int)
#define   IOCTL_FW_UPDATE           _IOR(ELAN_IOCTLID, 17, int)
#define   IOCTL_BC_VER              _IOR(ELAN_IOCTLID, 18, int)
#define   IOCTL_2WIREICE            _IOR(ELAN_IOCTLID, 19, int)


#define   CUSTOMER_IOCTLID          0xA0
#define   IOCTL_CIRCUIT_CHECK       _IOR(CUSTOMER_IOCTLID,  1, int)
#define   IOCTL_GET_UPDATE_PROGREE  _IOR(CUSTOMER_IOCTLID,  2, int)

//
#define I2C_RETRIES                     5
#define ELAN_FW_VERSION_DP1       0x5506
#define ELAN_FW_VER_CPT_PMMA       0x5545
#define ELAN_FW_VER_CPT_GLASS       0x5590
#define ELAN_FW_VER_INX_NONAF_KD       0x55F4
#define ELAN_FW_VER_INX_GLASS       0x55C8
#define ELAN_FW_VER_INX_AF_KD       0x55D1

#define ELAN_SENSOR_OPTION_DP1       0x9999
#define ELAN_SENSOR_OPTION_CPT_PMMA       0x00BA
#define ELAN_SENSOR_OPTION_CPT_GLASS       0x00BE
#define ELAN_SENSOR_OPTION_INX_NONAF_KD       0x00BF	//Innolux + NON AF + KD
#define ELAN_SENSOR_OPTION_INX_GLASS       0x00D0	//Innolux
#define ELAN_SENSOR_OPTION_INX_AF_KD       0x00D2	//Innolux AF + KD

uint8_t HELLO_PACKET_BOOT[8] = { 0 };
uint8_t Cali_PACKET_BOOT[4] = { 0 };
char result_lines_probe[2048];
int result_lines_index = 0;
uint16_t last_X, last_Y;
static int gesture_en = 0;
static int fw_status = 0; //update fw_status 20151020
//
//check CTP ID start
#if CHECK_CTP_ID
static int CTP_ID_g = -1;
//static int TP_ID_2_g;
#endif
//check CTP ID end

//if TP FW is updating, dont sleep in suspend - start
enum FW_LOCK {
	fw_lock_disable,
	fw_lock_suspend,
	fw_lock_resume,
};
static enum FW_LOCK fw_lock_status = fw_lock_disable;
//if TP FW is updating, dont sleep in suspend - end

extern struct tpd_device *tpd;

uint8_t RECOVERY    = 0x00;
int   FW_VERSION    = 0x00;
int   X_RESOLUTION  = 0x00;  /* Please fill the right resolution if resolution mapping error. */
int   Y_RESOLUTION  = 0x00;  /* Please fill the right resolution if resolution mapping error. */
int   FW_ID         = 0x00;
int   BC_VERSION    = 0x00;
int   SENSOR_OPTION = 0x00;
int   work_lock     = 0x00;
int   power_lock    = 0x00;
int   circuit_ver   = 0x01;
int   button_state  = 0;

/*++++i2c transfer start+++++++*/
//I2C device address to 0x15(7-bit address).
//int   file_fops_addr=0x10;
//int   file_fops_addr = 0x15;
int   file_fops_addr = I2C_ELAN_DEV_ADDR;
//Modify I2C slave address to 0x15
/*++++i2c transfer end+++++++*/
int   tpd_down_flag = 0;


//if TP FW is updating, dont sleep in suspend - start
struct wake_lock tp_lock;
//if TP FW is updating, dont sleep in suspend - end

struct i2c_client   * i2c_client = NULL;
struct task_struct  * thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static inline int elan_ktf2k_ts_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y);
#if 0
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
kal_bool auto_umask);
#endif

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
//static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info); //colby mask 20151001
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);


static int tpd_flag = 0;

#if 0
static int key_pressed = -1;

struct osd_offset{
  int left_x;
  int right_x;
  unsigned int key_event;
};

static struct osd_offset OSD_mapping[] = { /* Range need define by Case!  */
  {  35, 290, KEY_MENU },   /* menu_left_x, menu_right_x, KEY_MENU */
  { 303, 467, KEY_HOME },   /* home_left_x, home_right_x, KEY_HOME */
  { 473, 637, KEY_BACK },   /* back_left_x, back_right_x, KEY_BACK */
  { 641, 905, KEY_SEARCH }, /* search_left_x, search_right_x, KEY_SEARCH */
};
#endif

#if( IAP_PORTION )
uint8_t ic_status   = 0x00; /* 0:OK 1:master fail 2:slave fail  */
//<<Mel - 4/10, Modify I2C slave address to 0x15.
//uint8_t I2C_DATA[3] = {/*0x10*/0x15, 0x20, 0x21 };  /*I2C devices address*/
uint8_t I2C_DATA[3] = { I2C_ELAN_DEV_ADDR, 0x20, 0x21 };  /*I2C devices address*/
//Mel - 4/10, Modify I2C slave address to 0x15>>.
int   update_progree  = 0;
int   is_OldBootCode  = 0;  /* 0:new, 1:old */



/*The newest firmware, if update must be changed here */
// Thunder modify for update touch Firmware 20141229
static uint8_t file_fw_data[] = {
  #include "VY36_DP1_5506_9999.i" /* modify */
};

//add DP2 FW, start
//PMMA
static uint8_t file_fw_data_CPT_PMMA[] = {
  #include "VY36_PMMA_FW5545_ID00BA_20151002.i" /* modify */
};

//GLASS
static uint8_t file_fw_data_CPT_GLASS[] = {
  #include "VY36_Glass_FW5590_ID00BE_20160518.i" /* modify */
};
//add DP2 FW, end

//add 2nd source TP FW, start
//GLASS
static uint8_t file_fw_data_INX_GLASS[] = {
  #include "VY36_INX_3260_55C8_00D0_20160329.i" /* modify */
};
//add 2nd source TP FW, end

//add 2nd source TP FW, start
//GLASS + AF + KD
static uint8_t file_fw_data_INX_AF_KD[] = {
  #include "VY36_INX_3260_55D1_00D2_20160203.i" /* modify */
};
//add 2nd source TP FW, end

//add 2nd source TP FW, start
//GLASS + nonAF + KD
static uint8_t file_fw_data_INX_nonAF_KD[] = {
  #include "VY36_INX_3260_55F4_00BF_20160518.i" /* modify */
};
//add 2nd source TP FW, end

int PageNum     = 0;
enum
{
  PageSize    = 132,
  ACK_Fail    = 0x00,
  ACK_OK      = 0xAA,
  ACK_REWRITE = 0x55,
};

enum
{
  E_FD        = -1,
};
#endif

// Add 0821 start
static const struct i2c_device_id tpd_id[] =
{
  { "ektf2k_mtk", 0 },
  { }
};

#if defined( ELAN_MTK6577 ) || defined( ELAN_MTK6752 )
  static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO("ektf2k_mtk", ( I2C_ELAN_SLAVE_ADDR >> 1))};
#else
  unsigned short force[] = { I2C_ELAN_BUS, I2C_ELAN_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END };
  static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
#endif

//colby add start
#ifdef USE_ANDROID_M
#ifdef CONFIG_OF
static struct of_device_id elan_match_table[]={
    {.compatible = "mediatek,cap_touch"},
    {},
};
#endif
#endif
//colby add end

static struct i2c_driver tpd_i2c_driver =
{
    .driver = {
      .name   = "ektf2k_mtk",
      .owner  = THIS_MODULE,
//colby add start
#ifdef USE_ANDROID_M
#ifdef CONFIG_OF
			.of_match_table = elan_match_table,
#endif
#endif
//colby add end
    },
    .probe    = tpd_probe,
    .remove   = tpd_remove,
    .id_table = tpd_id,
    .detect   = tpd_detect,
  //.address_data = &addr_data,
};
//Add 0821 end



struct elan_ktf2k_ts_data {
  struct i2c_client       * client;
  struct input_dev        * input_dev;
  struct workqueue_struct * elan_wq;
  struct work_struct      work;
  //struct early_suspend    early_suspend; //colby mask 20151001
//colby add start
#ifdef USE_ANDROID_M
  struct elan_ktf2k_i2c_platform_data *pdata;
  //struct regulator *vdd;
  //struct regulator *vcc_i2c;
  struct pinctrl *ts_pinctrl;
  struct pinctrl_state *pinctrl_state_active;
  struct pinctrl_state *pinctrl_state_suspend;
  struct pinctrl_state *pinctrl_state_release;
  int		rst_gpio;
#endif
//colby add end
  int   intr_gpio;
  int   analog_data[6]; //elan add for IC frequency check 20151002
/* Firmware Information */
  int   fw_ver;
  int   fw_id;
  int   bc_ver;
  int   x_resolution;
  int   y_resolution;
  int   sensor_option;
/* For Firmare Update */
  struct miscdevice   firmware;
  struct hrtimer      timer;
  //0123
  struct attribute_group attrs;
  //0123
};

static struct elan_ktf2k_ts_data  * private_ts;
static int __hello_packet_handler(struct i2c_client *client);
static int  __fw_packet_handler(struct i2c_client *client);
static int  elan_ktf2k_ts_rough_calibrate(struct i2c_client *client);
//static void  tpd_resume(struct i2c_client *client); //colby mask 20151001
static void tpd_resume(struct device *h);
static int analog_info(struct i2c_client *client); //elan add for IC frequency check 20151002


// check HW ID start
#if CHECK_HW_ID
extern int board_type_with_hw_id(void);
extern int get_ftm_pin(void);
int g_HW_ID = -1;
int g_CURRENT_MODE = -1;
#endif
// check HW ID end

#if( IAP_PORTION )
//int   Update_FW_One(/*struct file *filp,*/ struct i2c_client *client, int recovery);
int   Update_FW_One(void *unused);	// Thunder modify for update touch Firmware 20141229
int   IAPReset(void);
#endif

#define   ELAN_ENABLE         1
#define   ELAN_DISABLE        0

static int elan_power_config(struct elan_ktf2k_ts_data *ts);
static int elan_power_on(struct elan_ktf2k_ts_data *ts, int on);

//colby add start
#ifndef USE_ANDROID_M
static void elan_power_enable(int enable)
{
    if( ELAN_ENABLE == enable )
    {
    #if defined( BUILD_LK )
        mt6325_upmu_set_rg_vgp1_en( 1 );
    #else
      #if 1
        hwPowerOn( MT6351_POWER_LDO_VLDO28, VOL_2800, "TP" );
        printk("[elan] MT6351_POWER_LDO_VLDO28: VOL_2800\n");
      #else
        hwPowerOn( MT6325_POWER_LDO_VGP1, VOL_3000, "TP" );
        printk("[elan] MT6325_POWER_LDO_VGP1: VOL_3000\n");
      #endif
    #endif
    }
    else
    {
    #if defined( BUILD_LK )
        mt6325_upmu_set_rg_vgp1_en( 0 );
    #else
        hwPowerDown( MT6325_POWER_LDO_VGP1, "TP" );
    #endif
    }
}
#endif
//colby 

#if defined( ELAN_DMA_MODE )

static int elan_i2c_dma_recv_data(struct i2c_client *client, uint8_t *buf,uint8_t len)
{
int       rc;
uint8_t * pReadData  = 0;
unsigned  short addr = 0;

  pReadData = gpDMABuf_va;
  addr = client->addr ;
  client->addr |= I2C_DMA_FLAG;
  if( !pReadData )
  {
    printk("[elan] dma_alloc_coherent failed!\n");
    return -1;
  }
  rc = i2c_master_recv( client, (u8 *)(uintptr_t)gpDMABuf_pa, len );
  printk("[elan] elan_i2c_dma_recv_data rc=%d!\n", rc);
//copy_to_user( buf, pReadData, len );
  client->addr = addr;

  return rc;
}


static int elan_i2c_dma_send_data(struct i2c_client *client, uint8_t *buf,uint8_t len)
{
int rc;
unsigned short addr = 0;
uint8_t *pWriteData = gpDMABuf_va;
  addr = client->addr ;
  client->addr |= I2C_DMA_FLAG;
  
if( !pWriteData )
  {
    printk("[elan] dma_alloc_coherent failed!\n");
    return -1;
  }
//copy_from_user(pWriteData, ((void*)buf), len);

  rc = i2c_master_send( client, (u8 *)(uintptr_t)gpDMABuf_pa, len);
  printk("[elan] elan_i2c_dma_send_data rc=%d!\n", rc);
  client->addr = addr;

  return rc;
}
#endif

/* For Firmware Update */
int elan_iap_open(struct inode *inode, struct file *filp)
{
  touch_debug(1, "[ELAN]into elan_iap_open\n");
  if( private_ts == NULL ) printk("private_ts is NULL~~~");

  return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp)
{
  return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
int   ret;
char *tmp;

  printk("[ELAN]into elan_iap_write\n");

#if defined( ESD_CHECK )  //0604
  have_interrupts = 1;
#endif
  if( count > 8192  )
    count = 8192;

  tmp = kmalloc( count, GFP_KERNEL );
  if( tmp == NULL )
    return -ENOMEM;

#if defined( ELAN_DMA_MODE )
  if( copy_from_user( gpDMABuf_va, buff, count ))
  {
    kfree( tmp );
    return -EFAULT;
  }
  ret = elan_i2c_dma_send_data( private_ts->client, tmp, count );
#else
  if( copy_from_user( tmp, buff, count ))
  {
    kfree( tmp );
    return -EFAULT;
  }
  ret = i2c_master_send( private_ts->client, tmp, count );
#endif /* End.. (ELAN_DMA_MODE) */

  kfree( tmp );
  return (( ret == 1 ) ? count : ret );
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
long  rc;
int   ret;
char *tmp;

  printk("[ELAN]into elan_iap_read\n");
#if defined( ESD_CHECK ) //0604
  have_interrupts = 1;
#endif
  if( count > 8192 )
    count = 8192;

  tmp = kmalloc( count, GFP_KERNEL );
  if( tmp == NULL)
    return -ENOMEM;

#if defined( ELAN_DMA_MODE )
  ret = elan_i2c_dma_recv_data( private_ts->client, tmp, count );
  if( ret >= 0 )
    rc = copy_to_user( buff, gpDMABuf_va, count );
#else
  ret = i2c_master_recv( private_ts->client, tmp, count );
  if( ret >= 0 )
    rc = copy_to_user( buff, tmp, count );
#endif /* End.. (ELAN_DMA_MODE) */

  kfree( tmp );
//return ret;
  return (( ret == 1 ) ? count : ret );
}




//colby add start
#if 1
static long elan_iap_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
//static long elan_iap_ioctl(/*struct inode *inode,*/ struct file *filp, unsigned int cmd, unsigned long arg)
//static long elan_iap_ioctl(/*struct inode *inode,*/ struct file *filp, unsigned int iFd,  unsigned int cmd, unsigned long arg)
{
//int __user *ip = (int __user *)arg; //colby mask 20151001

  printk("[ELAN]into elan_iap_ioctl\n");
  printk("cmd value %x\n",cmd);
	touch_debug(1, "ELAN]into elan_iap_ioctl\n");
	touch_debug(1, "cmd value %x\n",cmd);
	
//colby add start
#if 1
  switch ( cmd )
  {
    case IOCTL_I2C_SLAVE:
    {
      private_ts->client->addr = (int __user)arg;
      private_ts->client->addr &= I2C_MASK_FLAG;
      private_ts->client->addr |= I2C_ENEXT_FLAG;
    //file_fops_addr = 0x15;
    } break;
    case IOCTL_MAJOR_FW_VER:
      break;
    case IOCTL_MINOR_FW_VER:
      break;
    case IOCTL_RESET:
    {
//colby add start
#ifndef USE_ANDROID_M
      mt_set_gpio_mode( GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO );
      mt_set_gpio_dir( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );
      mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
      mdelay( 10 );
    //#if !defined(EVB)
      mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ZERO );
    //#endif
      mdelay(10);
      mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
#endif
//colby add end
    } break;
    case IOCTL_IAP_MODE_LOCK:
    {
      //if( 0 == work_lock )	// Thunder remove for factory AATS issue on Android L 20141229
      //{
        //printk("[elan]%s %X = IOCTL_IAP_MODE_LOCK\n", __func__, IOCTL_IAP_MODE_LOCK );
        //printk("[elan]%X = IOCTL_IAP_MODE_LOCK\n", IOCTL_IAP_MODE_LOCK );
        work_lock = 1;
        //disable_irq(CUST_EINT_TOUCH_PANEL_NUM);

//colby add start
#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
        //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );  //disable_irq( CUST_EINT_TOUCH_PANEL_NUM );
      //cancel_work_sync( &private_ts->work );
      #if defined( ESD_CHECK )  //0604
        cancel_delayed_work_sync( &esd_work );
      #endif
      //}
    } break;
    case IOCTL_IAP_MODE_UNLOCK:
    {
      //if( 1 == work_lock )	// Thunder remove for factory AATS issue on Android L 20141229
      //{
        work_lock = 0;
        //enable_irq(CUST_EINT_TOUCH_PANEL_NUM);

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
        //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM ); //enable_irq(CUST_EINT_TOUCH_PANEL_NUM);
      #if defined( ESD_CHECK )  //0604
        queue_delayed_work( esd_wq, &esd_work, delay );
      #endif
      //}
    } break;
    case IOCTL_CHECK_RECOVERY_MODE:
    {
      return RECOVERY;
    } //break;
    case IOCTL_FW_VER:
    {
      __fw_packet_handler( private_ts->client );
      return FW_VERSION;
    } //break;
    case IOCTL_X_RESOLUTION:
    {
      __fw_packet_handler( private_ts->client );
      return X_RESOLUTION;
    } //break;
    case IOCTL_Y_RESOLUTION:
    {
      __fw_packet_handler( private_ts->client );
      return Y_RESOLUTION;
    } //break;
    case IOCTL_FW_ID:
    {
      __fw_packet_handler( private_ts->client );
      return FW_ID;
    } //break;
    case IOCTL_ROUGH_CALIBRATE:
    {
      return elan_ktf2k_ts_rough_calibrate( private_ts->client );
    } //break;
    case IOCTL_I2C_INT:
    {
//colby add start
#ifndef USE_ANDROID_M
      put_user( mt_get_gpio_in( GPIO_CTP_EINT_PIN ), ip );
      printk("[elan]GPIO_CTP_EINT_PIN = %d\n", mt_get_gpio_in( GPIO_CTP_EINT_PIN ));
#else
      //put_user(  __gpio_get_value(private_ts->intr_gpio));
      printk("[elan]GPIO_CTP_EINT_PIN = %d\n", __gpio_get_value(private_ts->intr_gpio));
#endif
//colby add end
    } break;
    case IOCTL_RESUME:
    {
      //tpd_resume( private_ts->client );
    } break;
    case IOCTL_CIRCUIT_CHECK:
    {
      return circuit_ver;
    } //break;
    case IOCTL_POWER_LOCK:
    {
      power_lock = 1;
    } break;
    case IOCTL_POWER_UNLOCK:
    {
      power_lock = 0;
    } break;
  #if( IAP_PORTION )
    case IOCTL_GET_UPDATE_PROGREE:
    {
      update_progree = (int __user)arg;
    } break;
    case IOCTL_FW_UPDATE:
    {
      RECOVERY = 0; //= IAPReset( private_ts->client );
      Update_FW_One(NULL);
    } break;
  #endif
    case IOCTL_BC_VER:
    {
      __fw_packet_handler( private_ts->client );
      return BC_VERSION;
    } //break;
    default:
    {
    } break;
  }
#endif
//colby add end

  return 0;
}
#endif
//colby add end

struct file_operations elan_touch_fops =
{
    .open     = elan_iap_open,
    .write    = elan_iap_write,
    .read     = elan_iap_read,
    .release  = elan_iap_release,
    .unlocked_ioctl = elan_iap_ioctl,
};

#if( IAP_PORTION )
int EnterISPMode(struct i2c_client *client, uint8_t  *isp_cmd)
{
int   len = 0;
//char  buff[4] = { 0x00 }; //colby mask 20151001

  //len = i2c_master_send( private_ts->client, isp_cmd, sizeof( isp_cmd ));	// Thunder modify for update touch Firmware 20141229
  len = i2c_master_send( private_ts->client, isp_cmd, 4);	// send 4 byte only
  //if( len != sizeof( isp_cmd ))
  if( len != 4)
  {
    printk("[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
    return -1;
  }
  else
  {
    printk("[ELAN] IAPMode write data successfully! cmd = [%02X, %02X, %02X, %02X]\n", isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
    printk("[ELAN] len=%d, size of isp_cmd = %ld\n", len, sizeof(isp_cmd));
  }
  return 0;
}

int ExtractPage(struct file *filp, uint8_t * szPage, int byte)
{
int len = 0;

  len = filp->f_op->read( filp, szPage,byte, &filp->f_pos);
  if( len != byte )
  {
    printk("[ELAN] ExtractPage ERROR: read page error, read error. len=%d\r\n", len);
    return -1;
  }

  return 0;
}

int WritePage(uint8_t * szPage, int byte)
{
int len = 0;

  len = i2c_master_send( private_ts->client, szPage, byte );
  if( len != byte )
  {
    printk("[ELAN] ERROR: write page error, write error. len=%d\r\n", len );
    return -1;
  }

  return 0;
}

int GetAckData(struct i2c_client *client)
{
int   len = 0;
char  buff[2] = { 0x00 };

  len = i2c_master_recv( private_ts->client, buff, sizeof( buff ));
  if( len != sizeof( buff ))
  {
    printk("[ELAN] ERROR: read data error, write 50 times error. len=%d\r\n", len );
    return -1;
  }

  printk("[ELAN] GetAckData:%02X,%02X\n", buff[0], buff[1] );
  if( buff[0] == 0xAA /* && buff[1] == 0xAA */)
    return ACK_OK;
  else if(( buff[0] == 0x55 ) && ( buff[1] == 0x55 ))
    return ACK_REWRITE;
  else
    return ACK_Fail;

  return 0;
}

void print_progress(int page, int ic_num, int j)
{
int   i, percent, page_tatol, percent_tatol;
char  str[256] = { '\0' };

  str[0] = '\0';
  for( i = 0; i < (( page ) / 10 ); i++ )
  {
    str[i]    = '#';
    str[i+1]  = '\0';
  }

  page_tatol    = page + 377 * ( ic_num - j );
  percent       = (( 100 * page ) / ( 377 ));
  percent_tatol = (( 100 * page_tatol ) / ( 377 * ic_num ));

  if( 377 ==  page )
    percent = 100;

  if(( 377 * ic_num ) == page_tatol )
    percent_tatol = 100;

  printk("[ELAN]progress %s| %d%%\n", str, percent );
  //printk("\rprogress %s| %d%%", str, percent );

  if( 377 == page )
    printk("\n");
}

/*
* Restet and (Send normal_command ?)
* Get Hello Packet
*/
int IAPReset(void)
{
int   res = 1;
printk("[elan] IAPReset enter\n");

//colby add start
#ifndef USE_ANDROID_M
  mt_set_gpio_mode( GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO );
  mt_set_gpio_dir( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );
  mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
  mdelay( 10 );
//#if !defined( EVB )
  mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ZERO );
  mdelay( 10 );
//#endif
  mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
#else
//
	printk("[elan] IAPReset start, GTP_RST_PORT = %d\n", GTP_RST_PORT);
	tpd_gpio_output(GTP_RST_PORT, 1);
	mdelay( 10 );
	tpd_gpio_output(GTP_RST_PORT, 0);
	mdelay( 10 );
	tpd_gpio_output(GTP_RST_PORT, 1);
	mdelay(50);
	printk("[elan] IAPReset end\n");
#endif
//colby add end

#if 0
  printk("[ELAN] read Hello packet data!\n");
  res = __hello_packet_handler( client );
#endif
  return  res;
}

/* Check Master & Slave is "55 aa 33 cc" */
int CheckIapMode(void)
{
char buff[4] = { 0 }, len = 0;

//WaitIAPVerify(1000000);
//len = read( fd, buff, sizeof( buff ));
  len = i2c_master_recv( private_ts->client, buff, sizeof( buff ));
   
  if( sizeof( buff ) != len )
  {
    printk("[ELAN] CheckIapMode ERROR: read data error,len = %d\r\n", len );
    return -1;
  }
  else
  {
    if(( buff[0] == 0x55 ) && ( buff[1] == 0xAA ) && ( buff[2] == 0x33 ) && ( buff[3] == 0xCC ))
    {
      printk("[ELAN] CheckIapMode is 55 AA 33 CC\n");
      return 0;
    }
    else// if( j == 9 )
    {
      printk("[ELAN] Mode = 0x%02X 0x%02X 0x%02X 0x%02X\r\n", buff[0], buff[1], buff[2], buff[3] );
      printk("[ELAN] ERROR: CheckIapMode error\n");
      return -1;
    }
  }
  printk("\n");
}

//int Update_FW_One(struct i2c_client *client, int recovery)
//static int Update_FW_One()
int Update_FW_One(void *unused)	// Thunder modify for update touch Firmware 20141229
{
int res = 0, ic_num = 1;
int iPage = 0, rewriteCnt = 0; /* rewriteCnt for PAGE_REWRITE */
int i = 0;
int restartCnt = 0, checkCnt = 0; /* For IAP_RESTART */
//uint8_t recovery_buffer[4] = {0}; //colby mask 20151001
int curIndex = 0;
int byte_count = 0;
uint8_t *szBuff = NULL;
//if TP FW is updating, dont sleep in suspend - start
uint8_t sleep_cmd[] = { CMD_W_PKT, 0x50, 0x00, 0x01 };
//if TP FW is updating, dont sleep in suspend - end
//printk("[ELAN] Update_FW_One enter\n"); //colby mask 20151001
#if( CMD_54001234 )
uint8_t isp_cmd[] = { 0x54, 0x00, 0x12, 0x34 };  //54 00 12 34
#else
uint8_t isp_cmd[] = { 0x45, 0x49, 0x41, 0x50 };  //45 49 41 50
#endif
//uint8_t recovery_buffer[4] = { 0 };
uint8_t data;
//if TP FW is updating, dont sleep in suspend - start
	printk("[ELAN][elan]do wake lock before TP FW update");
	wake_lock(&tp_lock);
//if TP FW is updating, dont sleep in suspend - end
printk("[ELAN] Update_FW_One enter\n");
fw_status = 2; //update fw_status 20151020
//colby add start
#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	#ifdef ESD_CHECK	//0822
	cancel_delayed_work_sync(&esd_work);
    #endif


IAP_RESTART:

  data = I2C_DATA[0];  /* Master */
  //dev_dbg( &client->dev, "[ELAN] %s: address data=0x%x \r\n", __func__, data );
  curIndex = 0;
//if( recovery != 0x80 )
//{
    printk("[ELAN] Firmware upgrade normal mode !\n");

    IAPReset();
    mdelay( 50 );

    res = EnterISPMode( private_ts->client, isp_cmd );  /* enter ISP mode */
    ///* 1111 /* modify */

    mdelay( 10 );
#if 1  //1111
  /* Check IC's status is IAP mode(55 AA 33 CC) or not */
    res = CheckIapMode();  /* Step 1 enter ISP mode */
    if( -1 == res ) /* CheckIapMode fail */
    {
      checkCnt ++;
      if (checkCnt >= 5)
      {
        printk("[ELAN] ERROR: CheckIapMode %d times fails!\n", IAPRESTART);
        goto Upd_Fail;	// Thunder modify for update touch Firmware 20141229
      }
      else
      {
        printk("[ELAN] CheckIapMode retry %dth times! And restart IAP~~~\n\n", checkCnt);
        goto IAP_RESTART;
      }
    }
    else
      printk("[ELAN]  CheckIapMode ok!\n");
#endif


/* Send Dummy Byte */
  printk("[ELAN] send one byte data: %X, %X", private_ts->client->addr, data );
  res = i2c_master_send( private_ts->client, &data,  sizeof( data ));
  if( sizeof( data ) != res )
  {
    printk("[ELAN] dummy error code = %d\n", res );
  }
  mdelay( 50 );

/* Start IAP */
// check sensor option/TP ID for choosing FW start
if(SENSOR_OPTION == 0x9999){//HS + TR
	  PageNum = (sizeof(file_fw_data)/sizeof(uint8_t)/PageSize);
	  printk("[ELAN] Update_FW_One() USE (HS - TR) FW\n");
}
else if(SENSOR_OPTION == 0x00BA){//CPT/PMMA + TR
//file_fw_data_CPT_PMMA
	  PageNum = (sizeof(file_fw_data_CPT_PMMA)/sizeof(uint8_t)/PageSize);
	  printk("[ELAN] Update_FW_One() USE (CPT/PMMA - TR) FW\n");
	  //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00BE){//CPT/Glass + TR
//file_fw_data_CPT_GLASS
	  PageNum = (sizeof(file_fw_data_CPT_GLASS)/sizeof(uint8_t)/PageSize);
	  printk("[ELAN] Update_FW_One() USE (CPT/Glass - TR) FW\n");
	  //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00BF){//Innolux + NON AF + KD
//file_fw_data_INX_nonAF_KD
	  PageNum = (sizeof(file_fw_data_INX_nonAF_KD)/sizeof(uint8_t)/PageSize);
	  printk("[ELAN] Update_FW_One() USE (INX/GLASS_nonAF) FW\n");
	  //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00D2){//Innolux AF + KD
//file_fw_data_INX_AF_KD
	  PageNum = (sizeof(file_fw_data_INX_AF_KD)/sizeof(uint8_t)/PageSize);
	  printk("[ELAN] Update_FW_One() USE (INX/GLASS_AF_KD) FW\n");
	  //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00D0){//INX/Glass
//file_fw_data_INX_GLASS
	  PageNum = (sizeof(file_fw_data_INX_GLASS)/sizeof(uint8_t)/PageSize);
	  printk("[ELAN] Update_FW_One() USE (INX/Glass) FW\n");
	  //goto Upd_Fail;
}
else{
	  PageNum = (sizeof(file_fw_data)/sizeof(uint8_t)/PageSize);
	  printk("[ELAN] Update_FW_One() UNKNOWN Sensor Option, use (HS - TR) FW\n");// because DP1 boot code version is wrong, so download DP1 FW if sensor otion is UNKNOWN
}
  //PageNum = (sizeof(file_fw_data)/sizeof(uint8_t)/PageSize);	// Thunder modify for update touch Firmware 20141229
// check sensor option/TP ID for choosing FW end

  printk("[ELAN] Update FW PageNum =%d\n",PageNum );
  for( iPage = 1; iPage <= PageNum; iPage++ )
  {
PAGE_REWRITE:

#if 1
  /* 8byte mode */
  //szBuff = fw_data + (( iPage - 1) * PageSize );
    for( byte_count = 1; byte_count <= 17; byte_count++ )
    {
    //printk("[ELAN] byte %d, curIndex = %d\n", byte_count, curIndex );
      //szBuff    = file_fw_data + curIndex;
// check sensor option/TP ID for choosing FW start
if(SENSOR_OPTION == 0x9999){//HS + TR
    szBuff    = file_fw_data + curIndex;
    //printk("[ELAN] Update_FW_One() USE (HS - TR) FW\n");

}
else if(SENSOR_OPTION == 0x00BA){//CPT/PMMA + TR
//file_fw_data_CPT_PMMA
    szBuff    = file_fw_data_CPT_PMMA + curIndex;
    //printk("[ELAN] Update_FW_One() USE (CPT/PMMA - TR) FW\n");
    //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00BE){//CPT/Glass + TR
//file_fw_data_CPT_GLASS
    szBuff    = file_fw_data_CPT_GLASS + curIndex;
    //printk("[ELAN] Update_FW_One() USE (CPT/Glass - TR) FW\n");
    //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00BF){//Innolux + NON AF + KD
//file_fw_data_INX_nonAF_KD
    szBuff    = file_fw_data_INX_nonAF_KD + curIndex;
    //printk("[ELAN] Update_FW_One() USE (INX/GLASS_nonAF) FW, skip\n");
    //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00D2){//Innolux AF + KD
//file_fw_data_INX_AF_KD
    szBuff    = file_fw_data_INX_AF_KD + curIndex;
    //printk("[ELAN] Update_FW_One() USE (INX/GLASS_AF_KD) FW\n");
    //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00D0){//INX/Glass
//file_fw_data_INX_GLASS
    szBuff    = file_fw_data_INX_GLASS + curIndex;
    //printk("[ELAN] Update_FW_One() USE (INX/Glass) FW, skip\n");
    //goto Upd_Fail;
}
else{
    szBuff    = file_fw_data + curIndex;
    //printk("[ELAN] Update_FW_One() UNKNOWN Sensor Option, use (HS - TR) FW\n");// because DP1 boot code version is wrong, so download DP1 FW if sensor otion is UNKNOWN
}
// check sensor option/TP ID for choosing FW end

      if( 17 != byte_count )
      {
        curIndex  = curIndex + 8;
        //printk("[ELAN] curIndex =%d\n", curIndex);
      //ioctl( fd, IOCTL_IAP_MODE_LOCK, data );
        res = WritePage( szBuff, 8 );
      }
      else
      {
        curIndex  =  curIndex + 4;
        //printk("[ELAN] Last Index in this page, curIndex =%d\n", curIndex);
      //ioctl( fd, IOCTL_IAP_MODE_LOCK, data );
        res = WritePage( szBuff, 4 );
      }
    } /* End.. for(byte_count=1;...) */
#endif
#if 0 /* 132byte mode */
// check sensor option/TP ID for choosing FW start
if(SENSOR_OPTION == 0x9999){//HS + TR
    szBuff    = file_fw_data + curIndex;
    printk("[ELAN] Update_FW_One() USE (HS - TR) FW\n");

}
else if(SENSOR_OPTION == 0x00BA){//CPT/PMMA + TR
//file_fw_data_CPT_PMMA
    szBuff    = file_fw_data_CPT_PMMA + curIndex;
    printk("[ELAN] Update_FW_One() USE (CPT/PMMA - TR) FW\n");
    //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00BE){//CPT/Glass + TR
//file_fw_data_CPT_GLASS
    szBuff    = file_fw_data_CPT_GLASS + curIndex;
    printk("[ELAN] Update_FW_One() USE (CPT/Glass - TR) FW\n");
    //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00BF){//Innolux + NON AF + KD
//file_fw_data_INX_nonAF_KD
    szBuff    = file_fw_data_INX_nonAF_KD + curIndex;
    printk("[ELAN] Update_FW_One() USE (INX/GLASS_nonAF) FW\n");
    //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00D2){//Innolux AF + KD
//file_fw_data_INX_AF_KD
    szBuff    = file_fw_data_INX_AF_KD + curIndex;
    printk("[ELAN] Update_FW_One() USE (INX/GLASS_AF_KD) FW\n");
    //goto Upd_Fail;
}
else if(SENSOR_OPTION == 0x00D0){//INX/Glass
//file_fw_data_INX_GLASS
    szBuff    = file_fw_data_INX_GLASS + curIndex;
    printk("[ELAN] Update_FW_One() USE (INX/Glass) FW, skip\n");
    //goto Upd_Fail;
}
else{
    szBuff    = file_fw_data + curIndex;
    printk("[ELAN] Update_FW_One() UNKNOWN Sensor Option, use (HS - TR) FW\n");// because DP1 boot code version is wrong, so download DP1 FW if sensor otion is UNKNOWN
}
    //szBuff    = file_fw_data + curIndex;
// check sensor option/TP ID for choosing FW end

    curIndex  = curIndex + PageSize;
    printk("[ELAN] curIndex =%d\n", curIndex);
    memcpy( gpDMABuf_va, szBuff, PageSize );
    //res = WritePage( szBuff, PageSize );
    res = elan_i2c_dma_send_data( private_ts->client, szBuff, PageSize );
#endif
#if 1
    if(( PageNum == iPage )) //last page need to wait longer than other page
    {
      mdelay( 450 );
    }
    else
    {
      mdelay( 50);
    }
#endif
    res = GetAckData( private_ts->client );
    if( ACK_OK != res )
    {
      mdelay( 50 );
      printk("[ELAN] ERROR: GetAckData fail! res = %d\r\n", res );
      if( ACK_REWRITE == res )
      {
        rewriteCnt = rewriteCnt + 1;
        if( PAGERETRY == rewriteCnt )
        {
          printk("[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY );
          //return Upd_Fail; //1111
          goto Upd_Fail;	// Thunder modify for update touch Firmware 20141229
        }
        else
        {
          printk("[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt );
          curIndex = curIndex - PageSize;
          goto PAGE_REWRITE;
        }
      }
      else
      {
        restartCnt = restartCnt + 1;
        if( restartCnt >= 5 )
        {
          printk("[ELAN] ID 0x%02x ReStart %d times fails!\n", data, IAPRESTART );
          //return Upd_Fail; 
          goto Upd_Fail; // Thunder modify for update touch Firmware 20141229
        }
        else
        {
          printk("[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt );
          goto IAP_RESTART;
        }
      }
    }
    else
    {
      //printk("  data : 0x%02X ", data );
      rewriteCnt = 0;
      print_progress( iPage,ic_num, i );
    }

    //mdelay( 10 );
  } /* End.. for(iPage=1;...) */

   //1111 - start
  if (IAPReset() > 0)
     printk("[ELAN] IAPRESET Success!\n"); 
  mdelay(150);

  res= __hello_packet_handler(private_ts->client);
  if (res > 0) 
      printk("[ELAN] Update ALL Firmware successfully!\n"); 
  __fw_packet_handler(private_ts->client);

  printk("[ELAN] shows mt_eint_unmask\n");
  
#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	
  #ifdef ESD_CHECK
	   queue_delayed_work(esd_wq, &esd_work, delay);
  #endif
  printk("[ELAN] Update_FW_One OK\n");
//if TP FW is updating, dont sleep in suspend - start
  if(fw_lock_status == fw_lock_suspend){
	printk("[ELAN] in suspend mode, let TP into sleep mode. fw_lock_status = %d\n", fw_lock_status);
	fw_lock_status = fw_lock_disable;
	//into sleep mode
	if(( i2c_master_send(private_ts->client, sleep_cmd, sizeof( sleep_cmd ))) != sizeof( sleep_cmd )){
	printk("[ELAN] i2c_master_send failed\n");
	}
	}
  else if(fw_lock_status == fw_lock_resume){
	printk("[ELAN] in resume mode, no need do anything. fw_lock_status = %d\n", fw_lock_status);
	fw_lock_status = fw_lock_disable;
	}
  else{
	fw_lock_status = fw_lock_disable;
	printk("[ELAN] fw_lock_disable, no need do anything.\n");
	}
//if TP FW is updating, dont sleep in suspend - end
  fw_status = 3; //update fw_status 20151020
//if TP FW is updating, dont sleep in suspend - start
	wake_unlock(&tp_lock);
	printk("[ELAN]do wake unlock after TP FW update OK");
//if TP FW is updating, dont sleep in suspend - end
  //1111 - end
  return 0;
  
  //1111 - start  
Upd_Fail:

  printk("FAIL 1111 show_fw_update mt_eint_unmask\n");

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);


#ifdef ESD_CHECK  //1111
	queue_delayed_work(esd_wq, &esd_work, delay);
#endif

	fw_status = 4; //update fw_status 20151020
//if TP FW is updating, dont sleep in suspend - start
	wake_unlock(&tp_lock);
	printk("[ELAN] do wake unlock after TP FW update fail");
//if TP FW is updating, dont sleep in suspend - end
	return -1;
 //1111 - end   
  
}

#endif
/* End Firmware Update */


#if 0
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h);
static void elan_ktf2k_ts_late_resume(struct early_suspend *h);
#endif

//colby add start
#if 0
static ssize_t elan_ktf2k_gpio_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
struct elan_ktf2k_ts_data *ts = private_ts;
int   ret = 0;

  ret = mt_get_gpio_in( GPIO_CTP_EINT_PIN );  // = gpio_get_value(ts->intr_gpio);
  printk( KERN_DEBUG "GPIO_TP_INT_N = %d\n", ts->intr_gpio );
  sprintf( buf, "GPIO_TP_INT_N=%d\n", ret );
  ret = strlen(buf) + 1;
  return ret;
}

static DEVICE_ATTR( gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL );

static ssize_t elan_ktf2k_vendor_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
struct elan_ktf2k_ts_data *ts = private_ts;
ssize_t   ret = 0;

  sprintf( buf, "%s_x%4.4x\n", "ELAN_KTF2K", ts->fw_ver );
  ret = strlen( buf) + 1;
  return ret;
}
#endif
//colby add end


static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata( client ); //colby mask 20151001
int   status = 0, retry = 10;

  do
  {
//colby add start
#ifndef USE_ANDROID_M
    status = mt_get_gpio_in( GPIO_CTP_EINT_PIN ); // = gpio_get_value(ts->intr_gpio);
    //printk("[elan]: %s: status = %d\n", __func__, status );
#else
//
    status = __gpio_get_value(private_ts->intr_gpio);
#endif
//colby add end
    printk("[elan]:: status = %d\n", status );
    retry--;
  //mdelay( 200 ); //0403 modify
    mdelay( 10 );
  } while( status == 1 && retry > 0 );

//  printk("[elan]%s: poll interrupt status %s\n",
//      __func__, ( status == 1 ? "high" : "low" ));

  printk("[elan]: poll interrupt status %s\n",
      ( status == 1 ? "high" : "low" ));

  return ( status == 0 ? 0 : -ETIMEDOUT );
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
  return __elan_ktf2k_ts_poll( client );
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
      uint8_t *buf, size_t size)
{
int rc;

  //dev_dbg( &client->dev, "[elan]%s: enter\n", __func__ );
	if( buf == NULL ) {
		printk( "%s: buf = NULL.\n", __func__);
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index,
			"%s: buf = NULL.\n", __func__);
		return -EINVAL;
	}

	if (( i2c_master_send( client, cmd, 4 )) != 4 ) {
		printk( "%s: i2c_master_send failed.\n", __func__);
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index,
			"%s: i2c_master_send failed.\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf2k_ts_poll( client );
	if( rc < 0 ) {
		printk( "%s: Int poll failed!\n", __func__);
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index,
			"%s: Int poll failed!\n", __func__);
		return -EINVAL;
	}
	else {
		if(( i2c_master_recv( client, buf, size ) != size ) || ( buf[0] != CMD_S_PKT )) {
			printk( "%s: buf[0] = 0x%X, buf[1] = 0x%X, buf[2] = 0x%X, buf[3] = 0x%X\n", __func__, buf[0], buf[1], buf[2], buf[3] );
			result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index,
				"%s: buf[0] = 0x%X, buf[1] = 0x%X, buf[2] = 0x%X, buf[3] = 0x%X\n", __func__, buf[0], buf[1], buf[2], buf[3] );
			return -EINVAL;
		}
	}

  return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
int rc;
uint8_t buf_recv[8] = { 0 };
//uint8_t buf_recv1[4] = { 0 };

//mdelay(1500);
  result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "__hello_packet_handler: do polling int\n");
  rc = elan_ktf2k_ts_poll( client );
  if( rc < 0 )
  {
    //printk( "[elan] %s: Int poll failed!\n", __func__ );
    printk( "[elan] Int poll failed!\n");
  result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "__hello_packet_handler: Int poll failed!\n");
    RECOVERY = 0x80;
    //return RECOVERY; //dont return RECOVERY, just set RECOVERY = 0x80, 20150818
  }

  rc = i2c_master_recv( client, buf_recv, 8 );
  if( rc < 0 ){
	printk( "[elan] %s: read hello packet fail! rc = %d\n", __func__, rc);
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: read hello packet fail! rc = %d\n", __func__, rc);
	return rc;
  }
  memcpy(HELLO_PACKET_BOOT, buf_recv, sizeof(HELLO_PACKET_BOOT)); //record hello packet at boot
  printk("[elan] %s: Hello packet %2x:%2X:%2x:%2x:%2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);

  if(( buf_recv[0] == 0x55 ) && ( buf_recv[1] == 0x55 ) && ( buf_recv[2] == 0x80)  && ( buf_recv[3] == 0x80 ))
  {
	SENSOR_OPTION = (buf_recv[4]) | (buf_recv[5] << 8);
	printk("[elan] into RECOVERY MODE!!! SENSOR_OPTION = 0x%04X\n", SENSOR_OPTION);
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: into RECOVERY MODE!!! SENSOR_OPTION = 0x%04X\n", __func__, SENSOR_OPTION);
	RECOVERY = 0x80;
	return RECOVERY;
  }
  else if(( buf_recv[0] == 0x55 ) && ( buf_recv[1] == 0x55 ) && ( buf_recv[2] == 0x55)  && ( buf_recv[3] == 0x55 )){
	printk("[elan] get Hello Packet OK.\n");
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: get Hello Packet OK.\n", __func__);
  }
  else{
	printk("[elan] get UNKNOWN Packet, return fail\n");
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: get UNKNOWN Packet, return fail\n", __func__);
	return -1;
  }

  mdelay( 300 );
  
  rc = i2c_master_recv( client, buf_recv, 4 );
  memcpy(Cali_PACKET_BOOT, buf_recv, sizeof(Cali_PACKET_BOOT)); //record hello packet at boot
  printk("[elan] Calibration Packet %02x:%02X:%02x:%02x\n", buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] );
  result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "__hello_packet_handler: return rc = %d\n", rc);
  return rc;
}


static int __fw_packet_handler(struct i2c_client *client)
{
int rc;
int major, minor;
uint8_t cmd[]     = { CMD_R_PKT, 0x00, 0x00, 0x01 };/* Get Firmware Version*/
uint8_t cmd_x[]   = { 0x53, 0x60, 0x00, 0x00 };     /*Get x resolution*/
uint8_t cmd_y[]   = { 0x53, 0x63, 0x00, 0x00 };     /*Get y resolution*/
uint8_t cmd_id[]  = { 0x53, 0xF0, 0x00, 0x01 };     /*Get firmware ID*/
uint8_t cmd_bc[]  = { CMD_R_PKT, 0x10, 0x00, 0x01 };/* Get BootCode Version*/ //0403 modify
uint8_t cmd_sensor_option[]  = { CMD_R_PKT, 0xD3, 0x00, 0x01 };/* Get Sensor Option*/ //20150729 modify
uint8_t buf_recv[4] = { 0x00 };

  printk( "[elan] %s: \n", __func__);
  
/* Firmware version */
  rc = elan_ktf2k_ts_get_data( client, cmd, buf_recv, 4 );
  if( rc < 0 ){
  	printk( "[elan] %s: Get Firmware Version fail\n", __func__);
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: Get Firmware Version fail\n", __func__);
	return rc;
  	}
  major = (( buf_recv[1] & 0x0F) << 4 ) | (( buf_recv[2] & 0xF0) >> 4 );
  minor = (( buf_recv[2] & 0x0F) << 4 ) | (( buf_recv[3] & 0xF0) >> 4 );
  private_ts->fw_ver = major << 8 | minor;
  FW_VERSION = ( major << 8 ) | minor;

/* Firmware ID */
  rc = elan_ktf2k_ts_get_data( client, cmd_id, buf_recv, 4 );
  if( rc < 0 ){
  	printk( "[elan] %s: Get firmware ID fail\n", __func__);
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: Get firmware ID fail\n", __func__);
	return rc;
  	}
  major = (( buf_recv[1] & 0x0F) << 4 ) | (( buf_recv[2] & 0xF0 ) >> 4 );
  minor = (( buf_recv[2] & 0x0F) << 4 ) | (( buf_recv[3] & 0xF0 ) >> 4 );
  private_ts->fw_id = major << 8 | minor;
  FW_ID = ( major << 8 ) | minor;

/* X Resolution */
  rc = elan_ktf2k_ts_get_data( client, cmd_x, buf_recv, 4 );
  if( rc < 0 ){
	printk( "[elan] %s: Get x resolution fail\n", __func__);
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: Get x resolution fail\n", __func__);
	return rc;
  	}
  minor = (( buf_recv[2] )) | (( buf_recv[3] & 0xF0 ) << 4 );
  private_ts->x_resolution =minor;
  X_RESOLUTION = minor;

/* Y Resolution */
  rc = elan_ktf2k_ts_get_data( client, cmd_y, buf_recv, 4 );
  if( rc < 0 ){
  	printk( "[elan] %s: Get y resolution fail\n", __func__);
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: Get y resolution fail\n", __func__);
	return rc;
  	}
  minor = (( buf_recv[2] )) | (( buf_recv[3] & 0xF0 ) << 4 );
  private_ts->y_resolution =minor;
  Y_RESOLUTION = minor;

/* Bootcode version */
  rc = elan_ktf2k_ts_get_data( client, cmd_bc, buf_recv, 4 );
  if( rc < 0 ){
  	printk( "[elan] %s: Get BootCode Version fail\n", __func__);
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: Get BootCode Version fail\n", __func__);
	return rc;
  	}
  major = (( buf_recv[1] & 0x0F) << 4 ) | (( buf_recv[2] & 0xF0 ) >> 4 );
  minor = (( buf_recv[2] & 0x0F) << 4 ) | (( buf_recv[3] & 0xF0 ) >> 4 );
  private_ts->bc_ver = major << 8 | minor;
  BC_VERSION = major << 8 | minor;

  /* Sensor Option */
  rc = elan_ktf2k_ts_get_data( client, cmd_sensor_option, buf_recv, 4 );
  if( rc < 0 ){
	printk( "[elan] %s: Get Sensor Oprion fail\n", __func__);
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: Get Sensor Oprion fail\n", __func__);
	return rc;
  	}
  minor = (( buf_recv[3] )) | (buf_recv[2] << 8 );
  private_ts->sensor_option =minor;
  SENSOR_OPTION = minor;

  printk( "[elan] : __fw_packet_handler OK!!\n");
  printk( "[elan] : =========================================================\n");
  printk( "[elan] : firmware version: 0x%4.4X\n",
       FW_VERSION);
  printk( "[elan] : firmware ID: 0x%4.4X\n",
       FW_ID);
  printk( "[elan] : x resolution: %d, y resolution: %d, LCM x resolution: %d, LCM y resolution: %d\n",
       X_RESOLUTION, Y_RESOLUTION, LCM_X_MAX, LCM_Y_MAX );
  printk( "[elan] : bootcode version: 0x%4.4X\n",
       BC_VERSION );
  printk( "[elan] : sensor option: 0x%4.4X\n",
       SENSOR_OPTION );
  printk( "[elan] : =========================================================\n");

  result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: __fw_packet_handler OK\n", __func__);
  return 0;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
      uint16_t *x, uint16_t *y)
{
  *x = *y = 0;

  *x = ( data[0] & 0xF0 );
  *x <<= 4;
  *x |= data[1];

  *y = ( data[0] & 0x0F );
  *y <<= 8;
  *y |= data[2];

  return 0;
}

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
int rc;

  rc = __hello_packet_handler( client );
  printk("[elan] hellopacket's rc = %d\n", rc );
  mdelay( 10 );

  if( 0x80 != rc && rc >= 0)
  {
    rc = __fw_packet_handler( client );
    if( rc < 0 )
      printk("[elan] %s, fw_packet_handler fail, rc = %d\n", __func__, rc );
    else
      printk("[elan] firmware checking done.\n" );
    /* Check for FW_VERSION, if 0x0000 means FW update fail! */
    if( 0x00 == FW_VERSION )
    {
      //rc = 0x80;
      RECOVERY = 0x80;  //dont return rc = 0x80, just set RECOVERY = 0x80, 20150818
      printk("[elan] FW_VERSION = %d, last FW update fail\n", FW_VERSION );
    }
  }
  return rc; /* Firmware need to be update if rc equal to 0x80(Recovery mode)   */
}

static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client)
{
uint8_t cmd[] = { CMD_W_PKT, 0x29, 0x00, 0x01 };

//dev_info(&client->dev, "[elan] %s: enter\n", __func__);
  printk("[elan]  elan_ktf2k_ts_rough_calibrate enter\n" );
  dev_info( &client->dev, "[elan] dump cmd: %02X, %02X, %02X, %02X\n",
      cmd[0], cmd[1], cmd[2], cmd[3]);

  if(( i2c_master_send(client, cmd, sizeof( cmd ))) != sizeof( cmd ))
  {
    //dev_err( &client->dev, "[elan] %s: i2c_master_send failed\n", __func__ );
    printk("[elan] elan_ktf2k_ts_rough_calibrate fail.");
    return -EINVAL;
  }
  return 0;
}

//colby add start
#if 0
static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
uint8_t cmd[] = { CMD_W_PKT, 0x50, 0x00, 0x01 };

  //dev_dbg( &client->dev, "[elan] %s: enter\n", __func__ );

  cmd[1] |= ( state << 3 );

  dev_dbg(&client->dev, "[elan] dump cmd: %02X, %02X, %02X, %02X\n",
      cmd[0], cmd[1], cmd[2], cmd[3]);

  if(( i2c_master_send( client, cmd, sizeof( cmd ))) != sizeof( cmd ))
  {
    printk("[elan] elan_ktf2k_ts_set_power_state i2c_master_send failed\n" );
    return -EINVAL;
  }

  return 0;
}

static int elan_ktf2k_ts_get_power_state(struct i2c_client *client)
{
int rc = 0;
uint8_t   cmd[] = { CMD_R_PKT, 0x50, 0x00, 0x01 };
uint8_t   buf[4], power_state;

  rc = elan_ktf2k_ts_get_data( client, cmd, buf, 4 );
  if( rc )
    return rc;

  power_state = buf[1];
  dev_dbg( &client->dev, "[elan] dump repsponse: %0x\n", power_state );
  power_state = ( power_state & PWR_STATE_MASK ) >> 3;
  dev_dbg( &client->dev, "[elan] power state = %s\n",
      ( power_state == PWR_STATE_DEEP_SLEEP ? "Deep Sleep" : "Normal/Idle" ));

  return power_state;
}

static int elan_ktf2k_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
int err;
u8 beg = addr;
struct i2c_msg msgs[2] = {
  {
    .addr   = client->addr,
    .flags  = 0,
    .len    = 1,
    .buf    = &beg,
  },
  {
    .addr   = client->addr,
    .flags  = I2C_M_RD,
    .len    = len,
    .buf    = data,
    .ext_flag = I2C_DMA_FLAG,
  }
};

  if( !client )
    return -EINVAL;

  err = i2c_transfer( client->adapter, msgs, sizeof (msgs ) / sizeof( msgs[0] ));
  if( err != len )
  {
    printk("[elan] elan_ktf2k_read_block err = %d\n", err );
    err = -EIO;
  }
  else
  {
    printk("[elan] elan_ktf2k_read_block ok\n");
    err = 0;  /* no error */
  }

  return err;
}
#endif
//colby add end


static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf)
{
int   rc, bytes_to_recv = PACKET_SIZE;
uint8_t *pReadData = 0;
unsigned short addr = 0;

  if( buf == NULL )
    return -EINVAL;

  memset( buf, 0, bytes_to_recv );

//#if defined( ELAN_MTK6577 ) || defined( ELAN_MTK6752 )
#if defined( ELAN_DMA_MODE )
  addr  = client->addr ;
  client->addr |= I2C_DMA_FLAG;
  pReadData     = gpDMABuf_va;
  if( !pReadData )
  {
    printk("[elan] dma_alloc_coherent failed!\n");
  }
  rc = i2c_master_recv( client, (u8 *)(uintptr_t)gpDMABuf_pa, bytes_to_recv ); //colby add (u8 *)(uintptr_t) for gpDMABuf_pa
  rc = copy_to_user( buf, (u8 *)(uintptr_t)pReadData, bytes_to_recv ); //colby add rc for return of copy_to_user 20151001
  client->addr = addr;
  //#if defined( ELAN_DEBUG )
//  printk("[elan_dma] %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",
//      buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],buf[8], buf[9],
//      buf[10], buf[11], buf[12], buf[13], buf[14], buf[15],buf[16], buf[17] );
    touch_debug(0, "[elan_dma] %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",
      buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],buf[8], buf[9],
      buf[10], buf[11], buf[12], buf[13], buf[14], buf[15],buf[16], buf[17] );
  //#endif
#else
  rc = i2c_master_recv( client, buf, 8 );
  if( rc != 8 )
    printk("[elan_debug] The first package error.\n");
  printk("[elan_recv] %x %x %x %x %x %x %x %x\n",
      buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
  mdelay( 1 );

  if( buf[0] == FIVE_FINGERS_PKT ) /* For five finger */
  {
    rc = i2c_master_recv( client, buf + 8, 8 );
    if( rc != 8 )
      printk("[elan_debug] The second package error.\n");
    printk("[elan_recv] %x %x %x %x %x %x %x %x\n",
        buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
    rc = i2c_master_recv( client, buf + 16, 2 );
    if( rc != 2 )
      printk("[elan_debug] The third package error.\n");
    mdelay( 1 );
    printk("[elan_recv] %x %x \n", buf[16], buf[17]);
  }
#endif

  return rc;
}

#if defined( SOFTKEY_AXIS_VER ) /* SOFTKEY is reported via AXI */
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
struct input_dev *idev = tpd->dev;
uint16_t  x, y;
uint16_t  fbits=0;
int   finger_num = 0; //colby add 20151001
int   limitY = ELAN_Y_MAX - 100; /* limitY need define by Case! */
uint8_t   i, num, reported = 0;
uint8_t   idx, btn_idx;

/* for 10 fingers */
  if( buf[0] == TEN_FINGERS_PKT )
  {
    finger_num = 10;
    num   = buf[2] & 0x0F;
    fbits = buf[2] & 0x30;
    fbits = ( fbits << 4 ) | buf[1];
    idx   = 3;
    btn_idx = 33;
  }
/* for 5 fingers */
  else if(( buf[0] == MTK_FINGERS_PKT ) || ( buf[0] == FIVE_FINGERS_PKT ))
  {
    finger_num = 5;
    num   = buf[1] & 0x07;
    fbits = buf[1] >> 3;
    idx   = 2;
    btn_idx = 17;
  }
/* For 2 fingers */
  else
  {
    finger_num = 2;
    num   = buf[7] & 0x03;
    fbits = buf[7] & 0x03;
    idx   = 1;
    btn_idx = 7;
  }

  switch( buf[0] )
  {
//<<Mel - 4/10, Add 0x78 packet.
    case 0x78 :  /* chip may reset due to watch dog */
    {
      //printk(KERN_EMERG "!!!!!!!tp chip check event\n");
    } break;
//Mel - 4/10, Add 0x78 packet>>.

    case MTK_FINGERS_PKT:
    case TWO_FINGERS_PKT:
    case FIVE_FINGERS_PKT:
    case TEN_FINGERS_PKT:
    {
    //input_report_key(idev, BTN_TOUCH, 1);
      if( num == 0 )
      {
      //dev_dbg(&client->dev, "no press\n");
        if( key_pressed < 0 )
        {
          input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 0 );
          input_report_abs( idev, ABS_MT_WIDTH_MAJOR, 0 );
          input_mt_sync( idev );

//colby add start      
#if 0
          if(( FACTORY_BOOT == get_boot_mode() ) || ( RECOVERY_BOOT == get_boot_mode()))
          {
            tpd_button( x, y, 0 );
          }
            TPD_EM_PRINT( x, y, x, y, 0, 0 );
#endif
//colby add end
        }
        else
        {
        //dev_err( &client->dev, "[elan] KEY_RELEASE: key_code:%d\n",OSD_mapping[key_pressed].key_event);
          input_report_key( idev, OSD_mapping[key_pressed].key_event, 0 );
          key_pressed = -1;
        }
      }
      else
      {
      //dev_dbg( &client->dev, "[elan] %d fingers\n", num );
      //input_report_key( idev, BTN_TOUCH, 1 );
        for( i = 0; i < finger_num; i++ )
        {
          if(( fbits & 0x01 ))
          {
          #if 1
            elan_ktf2k_ts_parse_xy( &buf[idx], &x, &y );
          #else
            elan_ktf2k_ts_parse_xy( &buf[idx], &y, &x );
            x = X_RESOLUTION - x;
            y = Y_RESOLUTION - y;
          #endif

          #if 1
            if(( X_RESOLUTION > 0 ) && ( Y_RESOLUTION > 0 ))
            {
              //x = ( x * LCM_X_MAX ) / X_RESOLUTION;
              //y = ( y * LCM_Y_MAX ) / Y_RESOLUTION;
              x = X_RESOLUTION;
              y = Y_RESOLUTION;
            }
            else
            {
              x = ( x * LCM_X_MAX ) / ELAN_X_MAX;
              y = ( y * LCM_Y_MAX ) / ELAN_Y_MAX;
            }
          #endif
            //intk("[elan_debug SOFTKEY_AXIS_VER] %s, x=%d, y=%d\n",__func__, x , y );
            printk("[elan_debug SOFTKEY_AXIS_VER]  x=%d, y=%d\n" , x , y );

            if( !(( x <= 0 ) || ( y <= 0 ) || ( x >= X_RESOLUTION ) || ( y >= Y_RESOLUTION )))
            {
              if( y < limitY )
              {
                input_report_abs( idev, ABS_MT_TRACKING_ID, i );
                input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 8 );
                input_report_abs( idev, ABS_MT_POSITION_X, x );
                input_report_abs( idev, ABS_MT_POSITION_Y, y );
                input_mt_sync( idev );

//colby add start
#if 0
                if(( FACTORY_BOOT == get_boot_mode()) || ( RECOVERY_BOOT == get_boot_mode()))
                {
                  tpd_button( x, y, 1 );
                }
                TPD_EM_PRINT( x, y, x, y, i - 1, 1 );
#endif
//colby add end
              }
              else
              {
              int j = 0;

                for( j = 0; j < 4; j++)
                {
                  if(( x > OSD_mapping[j].left_x ) && ( x < OSD_mapping[j].right_x ))
                  {
                  //dev_err(&client->dev, "[elan] KEY_PRESS: key_code:%d\n",OSD_mapping[j].key_event );
                  //printk("[elan] %d KEY_PRESS: key_code:%d\n", j, OSD_mapping[j].key_event );
                    input_report_key( idev, OSD_mapping[j].key_event, 1 );
                    key_pressed = j;
                  }
                }
              }
              reported++;
            } /* End if(!((x<=0)||..) border */
          } /* End.. if finger status */
            fbits = fbits >> 1;
            idx += 3;
        } /* End.. for(i=0;..) */
      } /* End.. if(num).. else */

      if( reported )
        input_sync( idev );
      else
      {
        input_mt_sync( idev );
        input_sync( idev );
      }

    } break;

    default:
    {
      //dev_err( &client->dev, "[elan] %s: unknown packet type: %02X\n", __func__, buf[0] );
    } break;
  } /* End.. switch */

  return;
}

#else /* SOFTKEY is reported via BTN bit */
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
struct input_dev *idev = tpd->dev;
uint16_t  x =0, y = 0;
uint16_t  fbits=0;
int       finger_num = 0;
uint8_t   i, num = 0, reported = 0; //colby add 20151001
uint8_t   idx = 0, btn_idx = 0;

/* For 10 fingers */
  if( buf[0] == TEN_FINGERS_PKT )
  {
    finger_num = 10;
    num   = buf[2] & 0x0F;
    fbits = buf[2] & 0x30;
    fbits = ( fbits << 4 ) | buf[1];
    idx   = 3;
    btn_idx = 33;
  }
/* For 5 fingers */
  else if(( buf[0] == MTK_FINGERS_PKT ) || ( buf[0] == FIVE_FINGERS_PKT ))
  {
    finger_num = 5;
    num   = buf[1] & 0x07;
    fbits = buf[1] >>3;
    idx   = 2;
    btn_idx = 17;
  }
/* For 2 fingers */
  else if(buf[0] ==GESTURE_PKT)
  {
    printk("[elan]gesture package, buf[0] = 0x%x\n", buf[0]);  
  }
  else
  {
    finger_num = 2;
    num   = buf[7] & 0x03;
    fbits = buf[7] & 0x03;
    idx   = 1;
    btn_idx = 7;
  }

  switch( buf[0] )
  {
  //<<Mel - 4/10, Add 0x78 packet.
    case 0x78 :  /* chip may reset due to watch dog */
    {
		printk("[elan] TPIC alive 0x%02X\n", buf[0]);
    //printk(KERN_EMERG "!!!!!!!tp chip check event\n");
    } break;
  //Mel - 4/10, Add 0x78 packet>>.
    case GESTURE_PKT:
    {
#if 0
	//gesture package
	printk("[elan]tp gesture package, buf[1] = 0x%X\n", buf[1]);
	if(buf[1] == 0x06){
		printk("[elan]Gesture = M, should not report.\n");
		}
	else if(buf[1] == 0x07){
		printk("[elan]Gesture = C, should not report.\n");
		}
	else if(buf[1] == 0x0F){
		printk("[elan]Gesture = Double Click, should report.\n");
		//test key event
		input_report_key( idev, KEY_GESTURE, 1 );
		input_sync( idev );
		input_report_key( idev, KEY_GESTURE, 0 );
		input_sync(idev);
		//
		}
	else{
		printk("[elan]Gesture = UNKNOWN\n");
		}
#endif
    }break;
    case MTK_FINGERS_PKT:
    case TWO_FINGERS_PKT:
    case FIVE_FINGERS_PKT:
    case TEN_FINGERS_PKT:
    {
    //input_report_key( idev, BTN_TOUCH, 1 );
      if( num == 0 )
      {
        dev_dbg(&client->dev, "no press\n");
      #if defined( ELAN_DEBUG )
        printk("[elan]tp button_state0 = %X\n", button_state );
        printk("[elan]tp buf[btn_idx] = %X, KEY_MENU = %X, KEY_HOME = %X, KEY_BACK = %X, KEY_SEARCH = %X\n",
            buf[btn_idx], KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEARCH );
      #endif
      #if defined( ELAN_BUTTON )
        switch( buf[btn_idx] )
        {
          case ELAN_KEY_BACK:
          {
            printk("[elan]KEY back 1\n");
          #if !defined( LCT_VIRTUAL_KEY )
            input_report_key( idev, KEY_BACK, 1 );
          #else
            input_report_key( idev, BTN_TOUCH, 1 );
            input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 8 );
            input_report_abs( idev, ABS_MT_POSITION_X, 617 );
            input_report_abs( idev, ABS_MT_POSITION_Y, 1360 );
          #endif
            button_state = KEY_BACK;
          } break;

          case ELAN_KEY_HOME:
          {
            printk("[elan]KEY home 1\n");
          #if !defined( LCT_VIRTUAL_KEY )
            input_report_key( idev, KEY_HOMEPAGE, 1 );
          #else
            input_report_key( idev, BTN_TOUCH, 1 );
            input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 8 );
            input_report_abs( idev, ABS_MT_POSITION_X, 365 );
            input_report_abs( idev, ABS_MT_POSITION_Y, 1360 );
          #endif
            button_state = KEY_HOMEPAGE;
          } break;

          case ELAN_KEY_MENU:
          {
            printk("[elan]KEY menu 1\n");
          #ifndef LCT_VIRTUAL_KEY
            input_report_key( idev, KEY_MENU, 1 );
          #else
            input_report_key( idev, BTN_TOUCH, 1 );
            input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 8 );
            input_report_abs( idev, ABS_MT_POSITION_X, 107 );
            input_report_abs( idev, ABS_MT_POSITION_Y, 1360 );
          #endif
            button_state = KEY_MENU;
          } break;

        /* TOUCH release*/
          default:
          {
            printk("[elan] test tpd up\n");
            input_report_key( idev, BTN_TOUCH, 0 );
            input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 0 );
            input_report_abs( idev, ABS_MT_WIDTH_MAJOR, 0 );
            input_mt_sync( idev );
            tpd_down_flag = 0;
          } break;
        } /* End.. switch(buf[btn_idx]) */
      //input_sync(idev);
      #endif /* End.. (ELAN_BUTTON) */

//colby add start      
#if 0
        if(( FACTORY_BOOT == get_boot_mode()) || ( RECOVERY_BOOT == get_boot_mode()))
        {
          tpd_button( x, y, 0);
        }
#endif
//colby add end

        input_report_key( idev, BTN_TOUCH, 0 );
//add different TP source info when TOUCH up, start
	if(SENSOR_OPTION == 0x9999){//HS + TR
		printk("[elan] (HS - TR) Touch up: X = %d, Y = %d\n", last_X, last_Y);
	}
	else if(SENSOR_OPTION == 0x00BA){//CPT/PMMA + TR
		printk("[elan] (CPT/PMMA - TR) Touch up: X = %d, Y = %d\n", last_X, last_Y);
	}
	else if(SENSOR_OPTION == 0x00BE){//CPT/Glass + TR
		printk("[elan] (CPT/Glass - TR) Touch up: X = %d, Y = %d\n", last_X, last_Y);
	}
	else if(SENSOR_OPTION == 0x00BF){//Innolux + NON AF + KD
		printk("[elan] (INX/GLASS_nonAF) Touch up: X = %d, Y = %d\n", last_X, last_Y);
	}
	else if(SENSOR_OPTION == 0x00D2){//Innolux AF + KD
		printk("[elan] (INX/GLASS_AF_KD) Touch up: X = %d, Y = %d\n", last_X, last_Y);
	}
	else if(SENSOR_OPTION == 0x00D0){//INX/Glass
		printk("[elan] (INX/Glass) Touch up: X = %d, Y = %d\n", last_X, last_Y);
	}
	else{
		printk("[elan] (UNKNOWN) Touch up: X = %d, Y = %d\n", last_X, last_Y);
		}
//add different TP source info when TOUCH up, end
        //printk("[elan] Touch up: X = %d, Y = %d\n", last_X, last_Y);
        //TPD_EM_PRINT( x, y, x, y, 0, 0 ); //colby mask 20151001
      }
      else
      {
		//dev_dbg(&client->dev, "[elan] %d fingers\n", num);
        //printk( "[elan] %d fingers\n", num);
        //input_report_key( idev, BTN_TOUCH, 1 );
        //printk("[elan] input_report_key, After BTN_TOUCH\n");		
        for( i = 0; i < finger_num; i++ )
        {
          if(( fbits & 0x01 ))
          {
            elan_ktf2k_ts_parse_xy( &buf[idx], &x, &y );
          //elan_ktf2k_ts_parse_xy(&buf[idx], &y, &x );
          #if 1
            if(( X_RESOLUTION > 0 )&& ( Y_RESOLUTION > 0 ))
            {
              x = ( x * LCM_X_MAX ) / X_RESOLUTION;
              y = ( y * LCM_Y_MAX ) / Y_RESOLUTION;
            }
            else
            {
              x = ( x * LCM_X_MAX ) / ELAN_X_MAX;
              y = ( y * LCM_Y_MAX )  /ELAN_Y_MAX;
            }
          #endif
          //x = ( x * LCM_X_MAX ) / ELAN_X_MAX;
          //y = ( y * LCM_Y_MAX ) / ELAN_Y_MAX;
//record last x,y start
          last_X = x;
          last_Y = y;
//record last x,y end
          #if defined( ELAN_DEBUG )
            //printk("[elan_debug  BTN bit] %s, x=%d, y=%d\n",__func__, x , y );
            //printk("[elan_debug  BTN bit] , x=%d, y=%d\n", x , y );
            touch_debug(0, "[elan_debug  BTN bit] , x=%d, y=%d\n", x , y );
          #endif
            //x = LCM_X_MAX - x;
            //y = Y_RESOLUTION - y;
            if( !(( x <= 0 ) || ( y <= 0) || ( x >= LCM_X_MAX ) || ( y >= LCM_Y_MAX )))
            {
              input_report_key( idev, BTN_TOUCH, 1 );
              input_report_abs( idev, ABS_MT_TRACKING_ID, i );
              input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 8 );
              input_report_abs( idev, ABS_MT_POSITION_X, x );
              input_report_abs( idev, ABS_MT_POSITION_Y, y );
              input_mt_sync( idev );
              reported++;
              tpd_down_flag = 1;
//colby add start
#if 0
              if(( FACTORY_BOOT == get_boot_mode()) || ( RECOVERY_BOOT == get_boot_mode()))
              {
                tpd_button( x, y, 1 );
              }
              TPD_EM_PRINT(x, y, x, y, i - 1, 1 );
#endif
//colby add end
            } /* End.. if border */
          } /* end.. if finger status */
          fbits = fbits >> 1;
          idx += 3;
        } /* End.. for */
      } /* End.. if(num==0).. else */

      if( reported )
        input_sync( idev );
      else
      {
        input_mt_sync( idev );
        input_sync( idev );
      }

    } break;

    default:
    {
      //printk("[elan] %s: unknown packet type: %0X\n", __func__, buf[0]);
      printk("[elan] : unknown packet type: %0X\n", buf[0]);
    } break;
  } /* End.. switch(buf[0]) */

  return;
}
#endif /* End.. (SOFTKEY_AXIS_VER) */

//colby add start
#if 0
static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
struct elan_ktf2k_ts_data *ts = container_of( work, struct elan_ktf2k_ts_data, work );
int rc;
uint8_t   buf[PACKET_SIZE] = { 0 };

  if( mt_get_gpio_in( GPIO_CTP_EINT_PIN ))  //if( gpio_get_value(ts->intr_gpio ))
  {
    printk("[elan]: Detected Jitter at INT pin.\n");

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
    //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );  //enable_irq( ts->client->irq );
    return;
  }

  rc = elan_ktf2k_ts_recv_data( ts->client, buf );
  if( rc < 0 )
  {
    printk("[elan] elan_ktf2k_ts_recv_data Error, Error code %d \n", rc );

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
    //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );  //enable_irq(ts->client->irq);
    return;
  }

//printk("[elan] %2x,%2x,%2x,%2x,%2x,%2x\n",buf[0],buf[1],buf[2],buf[3],buf[5],buf[6]);
  elan_ktf2k_ts_report_data( ts->client, buf );

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
  //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );  //enable_irq(ts->client->irq);

  return;
}
#endif
//colby add end

//colby add start
#if 0
static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
//struct elan_ktf2k_ts_data *ts = dev_id;
//struct i2c_client   *client = ts->client;

  //dev_dbg( &client->dev, "[elan] %s\n", __func__ );

//colby add start
#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
  //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
  tpd_flag = 1;
  wake_up_interruptible( &waiter );

  return IRQ_HANDLED;
}
#endif
//colby add end

//colby add start
#if 0
static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
int   err = 0;

  err = request_irq( client->irq, elan_ktf2k_ts_irq_handler,
                        IRQF_TRIGGER_LOW, client->name, ts );
  if( err )
    printk( "[elan] elan_ktf2k_ts_register_interrupt\n");
    //dev_err( &client->dev, "[elan] %s: request_irq %d failed\n",
    //    __func__, client->irq );

  return err;
}
#endif
//colby add end

static int touch_event_handler(void *unused)
{
struct sched_param  param = { .sched_priority = RTPM_PRIO_TPD };
//unsigned long time_eclapse; //colby mask 20151001
int   rc;
//int   touch_state = 3; //colby mask 20151001
//int button_state = 0;
//int   last_key = 0; //colby mask 20151001
//int   key; //colby mask 20151001
//int   index = 0; //colby mask 20151001
//int   i =0; //colby mask 20151001
uint8_t buf[PACKET_SIZE] = { 0 };

  sched_setscheduler( current, SCHED_RR, &param );

tpd_flag = 0; //set tpd_flag as 0, should ignore interrupt before touch_event_handler init.

  do
  {
#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
    //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );  //enable_irq(CUST_EINT_TOUCH_PANEL_NUM);
    set_current_state( TASK_INTERRUPTIBLE );
    wait_event_interruptible( waiter, tpd_flag != 0 );
    tpd_flag = 0;
    set_current_state( TASK_RUNNING );
    //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );  //disable_irq(CUST_EINT_TOUCH_PANEL_NUM);

    rc = elan_ktf2k_ts_recv_data( private_ts->client, buf );
    if( rc < 0 )
    {
      printk("[elan] rc<0\n");
      continue;
    }

    elan_ktf2k_ts_report_data(/*ts*/private_ts->client, buf );

  } while( !kthread_should_stop());

  return 0;
}

//colby add start
#if 0
static int exec_elan_user_process() //0829
{
	int ret=0;
	//char *argv[] = {ELAN_PATH,"&", NULL };	//normal mode
	char *argv[] = {ELAN_PATH, NULL };	//normal mode

	char *envp[] = {"HOME=/", "PATH=/sbin:/system/bin", NULL };
	do
	{
		ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC );
		if (ret != 0)
		printk("error in call to usermodehelper: %i\n", ret);
		else
		printk("everything all right\n");
	}while(ret!=0);

	return ret;
	
}
#endif
//colby add end

//static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) //colby mask 20151001
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info)
{
  //strcpy( info->type, TPD_DEVICE ); //colby mask 20151001
  strcpy( info->type, "mtk-tpd" );
  return 0;
}

//colby add start
#if 1
//static void tpd_eint_interrupt_handler(void *unused)//colby mask 20151001
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
//printk("[elan]TPD int\n");
#if defined( ESD_CHECK )  //0604
  have_interrupts = 1;
#endif
  tpd_flag = 1;

//colby add start
#ifdef USE_ANDROID_M
  disable_irq_nosync(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
  //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
  wake_up_interruptible( &waiter );
  return IRQ_HANDLED; //colby add 20151001
}
#endif
//colby add end

//0814 add start
static ssize_t show_reset(struct device *dev,
struct device_attribute *attr, char *buf)
{
#if( IAP_PORTION )
	//struct i2c_client *client = to_i2c_client(dev); //colby mask 20151001
	//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);  //colby mask 20151001

	IAPReset();

	return sprintf(buf, "Reset Touch Screen \n");
#endif
	return sprintf(buf, "Cannot Reset Touch Screen, no define IAP_PORTION \n");
}

static ssize_t store_reset(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	int reset_value;
	sscanf(buf,"%d",&reset_value);
	
//colby add start
#ifndef USE_ANDROID_M
	if (reset_value){
		printk("[elan]reset_value = %d\n", reset_value); // 1
		mt_set_gpio_mode( GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO );
		mt_set_gpio_dir( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );
		mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
		}
	else{
		printk("[elan]reset_value = %d\n", reset_value); // 0
		mt_set_gpio_mode( GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO );
		mt_set_gpio_dir( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );
		mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ZERO );
		}
#else
	if (reset_value){
		printk("[elan] store_reset start, GTP_RST_PORT = %d, reset_value = %d\n", GTP_RST_PORT, reset_value);
		tpd_gpio_output(GTP_RST_PORT, 1);
		mdelay(50);
		printk("[elan] store_reset end\n");
		}
	else{
		printk("[elan] store_reset start, GTP_RST_PORT = %d, reset_value = %d\n", GTP_RST_PORT, reset_value);
		tpd_gpio_output(GTP_RST_PORT, 0);
		mdelay(50);
		printk("[elan] store_reset end\n");
		}
	printk("[elan] store_reset : read reset pin = %d\n", __gpio_get_value(private_ts->rst_gpio));
#endif
//colby add end

	return count;
}

static ssize_t show_enable_irq(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	work_lock=0;
	//enable_irq(private_ts->client->irq);
	printk("[elan]mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM).....");

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
  #ifdef ESD_CHECK
	   queue_delayed_work(esd_wq, &esd_work, delay);
  #endif
	//wake_unlock(&private_ts->wakelock);

	return sprintf(buf, "Enable IRQ \n");
}

static ssize_t show_disable_irq(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev); //colby mask 20151001
	//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client); //colby mask 20151001

	work_lock=1;
	printk("[elan]mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM).....");
	
//colby add start
#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	#ifdef ESD_CHECK
	   cancel_delayed_work_sync(&esd_work);
  #endif
	//disable_irq(private_ts->client->irq);
	//wake_lock(&private_ts->wakelock);
	
	return sprintf(buf, "Disable IRQ \n");
}

static ssize_t show_calibrate(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	ret = elan_ktf2k_ts_rough_calibrate(client);
	return sprintf(buf, "%s\n",
	(ret == 0) ? " Testing the node of calibrate finish" : "Testing the node of calibrate fail");
}

static ssize_t show_fw_update(struct device *dev,
struct device_attribute *attr, char *buf)
{
#if( IAP_PORTION )
	int ret;
	//struct i2c_client *client = to_i2c_client(dev); //colby mask 20151001

	//printk("0822 show_fw_update mt_eint_mask\n");
	/*
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	#ifdef ESD_CHECK	//0822
	cancel_delayed_work_sync(&esd_work);
#endif
	power_lock = 1;
	wake_lock(&private_ts->wakelock);
	work_lock=1;  //0822     
	*/

	ret = Update_FW_One(NULL);

	/*
	work_lock=0;
	printk("0822 show_fw_update mt_eint_unmask\n");
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	wake_unlock(&private_ts->wakelock);
	#ifdef ESD_CHECK  //0822
	queue_delayed_work(esd_wq, &esd_work, delay);	
#endif
	*/
	return sprintf(buf, "Update Firmware\n");
#endif
	return sprintf(buf, "Cannot Update Firmware, no define IAP_PORTION\n");
}
static ssize_t show_fw_version_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
//
#ifdef ESD_CHECK
  cancel_delayed_work_sync(&esd_work);
#endif

#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#endif

  __fw_packet_handler(private_ts->client);

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#endif

#ifdef ESD_CHECK
  queue_delayed_work(esd_wq, &esd_work, delay);
#endif
//
	return sprintf(buf, "0x%04x\n", private_ts->fw_ver);
	
}

static ssize_t show_fw_id_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "0x%04x\n", private_ts->fw_id);
}

static ssize_t show_bc_version_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "0x%04x\n", private_ts->bc_ver);
}

static ssize_t show_drv_version_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "driver version 5.0");
}

static ssize_t show_iap_mode(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%s\n", 
	(private_ts->fw_ver == 0) ? "Recovery" : "Normal" );
}

//colby add start
#if 0
//0829
static ssize_t get_rawdata(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//get openshort_result, openshort_result
	mm_segment_t oldfs;
	char openshort[100]; 
	char discharge[100]; 
	struct file *fp; 
	int ret; 
	
	
	//fp=openFile("/data/openshort_result",O_RDONLY,0); 
	fp=filp_open("/data/openshort_result",O_RDONLY,0); 
	if (fp!=NULL) 
	{ 
		oldfs = get_fs(); 
		set_fs(get_ds());
		
		memset(buf,0,1024); 
		if ((ret=fp->f_op->read(fp,openshort,2,&fp->f_pos))>0) 
		printk("buf:%s\n",openshort); 
		else 
		printk("read openshort_result file error %d\n",ret); 
		
		set_fs(oldfs);
	} 
	filp_close(fp,NULL);
	//closeFile(fp);
	
	//fp=openFile("/data/discharge_result",O_RDONLY,0); 
	fp=filp_open("/data/discharge_result",O_RDONLY,0); 
	if (fp!=NULL) 
	{ 
		oldfs = get_fs(); 
		set_fs(get_ds());
		if ((ret=fp->f_op->read(fp,discharge,2,&fp->f_pos))>0) 
		printk("buf:%s\n",discharge); 
		else 
		printk("read discharge_result file error %d\n",ret); 
		
		set_fs(oldfs);
	} 
	filp_close(fp,NULL);
	

	
	if(ret)
	return sprintf(buf, "%s\n%s\n",openshort, discharge);
	else
	return sprintf(buf, "read file error\n");
}

static ssize_t exec_rawdata(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret;
	//exec rawdata ap
	ret = exec_elan_user_process();
	
	if(ret==0)
	return sprintf(buf, "Rawdata Execute Success\n");
	else
	return sprintf(buf, "Rawdata Execute Failed\n");

}
//0829
#endif
//colby add end

//1014 - debug
static ssize_t get_debug(struct device *dev,
struct device_attribute *attr, char *buf)
{
	printk("[elan]debug_flag=%d\n",debug_flag);
	if (debug_flag)
	return sprintf(buf, "Enable Debug \n");
	else
	return sprintf(buf, "Disable Debug \n");
}

static ssize_t set_debug(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	
	sscanf(buf,"%d",&debug_flag);
	if (debug_flag)
	printk("[elan]debug_flag Set Enable\n");
	else
	printk("[elan]debug_flag Set Disable\n");
	return count;
}
//1014

static ssize_t get_x_resolution(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", X_RESOLUTION);
}

static ssize_t get_y_resolution(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", Y_RESOLUTION);
}

static ssize_t get_packet_info(struct device *dev,
struct device_attribute *attr, char *buf)
{
//HELLO_PACKET_BOOT
//Cali_PACKET_BOOT
	printk("[elan] hello packet %2x:%2X:%2x:%2x:%2x:%2X:%2x:%2x\n", 		
		HELLO_PACKET_BOOT[0], HELLO_PACKET_BOOT[1], HELLO_PACKET_BOOT[2], HELLO_PACKET_BOOT[3] , HELLO_PACKET_BOOT[4], HELLO_PACKET_BOOT[5], HELLO_PACKET_BOOT[6], HELLO_PACKET_BOOT[7]);
	printk("[elan] calibration packet %2x:%2X:%2x:%2x\n", 		
		Cali_PACKET_BOOT[0], Cali_PACKET_BOOT[1], Cali_PACKET_BOOT[2], Cali_PACKET_BOOT[3]);
	return sprintf(buf, "HELLO_PACKET_BOOT = %2x:%2X:%2x:%2x:%2x:%2X:%2x:%2x, Cali_PACKET_BOOT = %2x:%2X:%2x:%2x, I2C Addr = %x\n", 
		HELLO_PACKET_BOOT[0], HELLO_PACKET_BOOT[1], HELLO_PACKET_BOOT[2], HELLO_PACKET_BOOT[3], HELLO_PACKET_BOOT[4], HELLO_PACKET_BOOT[5], HELLO_PACKET_BOOT[6], HELLO_PACKET_BOOT[7],
		Cali_PACKET_BOOT[0], Cali_PACKET_BOOT[1], Cali_PACKET_BOOT[2], Cali_PACKET_BOOT[3], I2C_ELAN_DEV_ADDR);
}

static ssize_t get_probe_log(struct device *dev,
struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "result_lines_probe = \n%s, size of result_lines_probe = %ld, result_lines_index = %d\n", result_lines_probe, strlen(result_lines_probe), result_lines_index);
}

static ssize_t get_fw_info(struct device *dev,
struct device_attribute *attr, char *buf)
{
//colby add start
#ifdef ESD_CHECK
  cancel_delayed_work_sync(&esd_work);
#endif
#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
  //mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
  __fw_packet_handler(private_ts->client);

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
  //mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef ESD_CHECK
  queue_delayed_work(esd_wq, &esd_work, delay);
#endif
	return sprintf(buf, "FW ID:0x%04x, FW VER: 0x%04x, SENSOR OPTION: 0x%04x\n", private_ts->fw_id, private_ts->fw_ver, SENSOR_OPTION);
}

static ssize_t get_ftm_fw_check(struct device *dev,
struct device_attribute *attr, char *buf)
{
  //get FW info
// ELAN_FW_VERSION_DP1       0x5506
// ELAN_FW_VER_CPT_PMMA       0x5545
// ELAN_FW_VER_CPT_GLASS       0x5590
// ELAN_FW_VER_INX_NONAF_KD       0x55F4
// ELAN_FW_VER_INX_GLASS       0x55C8
// ELAN_FW_VER_INX_AF_KD       0x55D1

// ELAN_SENSOR_OPTION_DP1       0x9999
// ELAN_SENSOR_OPTION_CPT_PMMA       0x00BA
// ELAN_SENSOR_OPTION_CPT_GLASS       0x00BE
// ELAN_SENSOR_OPTION_INX_NONAF_KD       0x00BF	//Innolux + NON AF + KD
// ELAN_SENSOR_OPTION_INX_GLASS       0x00D0	//Innolux
// ELAN_SENSOR_OPTION_INX_AF_KD       0x00D2	//Innolux AF + KD

//colby add start
#ifdef ESD_CHECK
  cancel_delayed_work_sync(&esd_work);
#endif
#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
  //mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	__fw_packet_handler(private_ts->client);

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
  //mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef ESD_CHECK
  queue_delayed_work(esd_wq, &esd_work, delay);
#endif
	printk("[elan] get_ftm_fw_check() FW_VERSION = %04X, SENSOR_OPTION = %04X\n", FW_VERSION, SENSOR_OPTION);

	printk("[elan] ===========================FW INFORMATION===========================================\n");
	printk("[elan] ELAN_FW_VERSION_DP1 = %04X, ELAN_SENSOR_OPTION_DP1 = %04X\n", ELAN_FW_VERSION_DP1, ELAN_SENSOR_OPTION_DP1);
	printk("[elan] ELAN_FW_VER_CPT_PMMA = %04X, ELAN_SENSOR_OPTION_CPT_PMMA = %04X\n", ELAN_FW_VER_CPT_PMMA, ELAN_SENSOR_OPTION_CPT_PMMA);
	printk("[elan] ELAN_FW_VER_CPT_GLASS = %04X, ELAN_SENSOR_OPTION_CPT_GLASS = %04X\n", ELAN_FW_VER_CPT_GLASS, ELAN_SENSOR_OPTION_CPT_GLASS);
	printk("[elan] ELAN_FW_VER_INX_NONAF_KD = %04X, ELAN_SENSOR_OPTION_INX_NONAF_KD = %04X\n", ELAN_FW_VER_INX_NONAF_KD, ELAN_SENSOR_OPTION_INX_NONAF_KD);
	printk("[elan] ELAN_FW_VER_INX_GLASS = %04X, ELAN_SENSOR_OPTION_INX_GLASS = %04X\n", ELAN_FW_VER_INX_GLASS, ELAN_SENSOR_OPTION_INX_GLASS);
	printk("[elan] ELAN_FW_VER_INX_AF_KD = %04X, ELAN_SENSOR_OPTION_INX_AF_KD = %04X\n", ELAN_FW_VER_INX_AF_KD, ELAN_SENSOR_OPTION_INX_AF_KD);
	printk("[elan] ====================================================================================\n");

// TP FW check always PASS for DP2, start
//printk("[elan] get_ftm_fw_check() PASS anyway for DP2\n");
//return sprintf(buf, "Pass\n");
// TP FW check always PASS for DP2, end

 //check FW & Sensor Option
	switch(SENSOR_OPTION){
		case ELAN_SENSOR_OPTION_DP1: // HS + TR
		{
			printk("[elan] get_ftm_fw_check() HS + TR\n");
			if(FW_VERSION == ELAN_FW_VERSION_DP1){
				//PASS
				return sprintf(buf, "Pass\n");
			}
			else{
				//FAIL
				return sprintf(buf,
				"fw check version FAIL:\n ELAN_FW_VERSION_DP1 = 0x%04X, FW_VERSION = 0x%04X\n ELAN_SENSOR_OPTION_DP1 = 0x%04X, SENSOR_OPTION = 0x%04X\n",
					ELAN_FW_VERSION_DP1,
					FW_VERSION,
					ELAN_SENSOR_OPTION_DP1,
					SENSOR_OPTION);
			}
		} break;

		case ELAN_SENSOR_OPTION_CPT_PMMA: // CPT/PMMA + TR
		{
			printk("[elan] get_ftm_fw_check() CPT/PMMA + TR\n");
			if(FW_VERSION == ELAN_FW_VER_CPT_PMMA){
				//PASS
				return sprintf(buf, "Pass\n");
			}
			else{
				//FAIL
				return sprintf(buf,
				"fw check version FAIL:\n ELAN_FW_VER_CPT_PMMA = 0x%04X, FW_VERSION = 0x%04X\n ELAN_SENSOR_OPTION_CPT_PMMA = 0x%04X, SENSOR_OPTION = 0x%04X\n",
					ELAN_FW_VER_CPT_PMMA,
					FW_VERSION,
					ELAN_SENSOR_OPTION_CPT_PMMA,
					SENSOR_OPTION);
			}
		} break;

		case ELAN_SENSOR_OPTION_CPT_GLASS:// CPT/GLASS + TR
		{
			printk("[elan] get_ftm_fw_check() CPT/GLASS + TR\n");
			if(FW_VERSION == ELAN_FW_VER_CPT_GLASS){
				//PASS
				return sprintf(buf, "Pass\n");
			}
			else{
				//FAIL
				return sprintf(buf,
				"fw check version FAIL:\n ELAN_FW_VER_CPT_GLASS = 0x%04X, FW_VERSION = 0x%04X\n ELAN_SENSOR_OPTION_CPT_GLASS = 0x%04X, SENSOR_OPTION = 0x%04X\n",
					ELAN_FW_VER_CPT_GLASS,
					FW_VERSION,
					ELAN_SENSOR_OPTION_CPT_GLASS,
					SENSOR_OPTION);
			}
		} break;

		case ELAN_SENSOR_OPTION_INX_NONAF_KD: //Innolux + NON AF + KD
		{
			printk("[elan] get_ftm_fw_check() INX/GLASS_nonAF\n");
			if(FW_VERSION == ELAN_FW_VER_INX_NONAF_KD){
				//PASS
				return sprintf(buf, "Pass\n");
			}
			else{
				//FAIL
				return sprintf(buf,
				"fw check version FAIL:\n ELAN_FW_VER_INX_NONAF_KD = 0x%04X, FW_VERSION = 0x%04X\n ELAN_SENSOR_OPTION_INX_NONAF_KD = 0x%04X, SENSOR_OPTION = 0x%04X\n",
					ELAN_FW_VER_INX_NONAF_KD,
					FW_VERSION,
					ELAN_SENSOR_OPTION_INX_NONAF_KD,
					SENSOR_OPTION);
			}
		} break;

		case ELAN_SENSOR_OPTION_INX_AF_KD: //Innolux AF + KD
		{
			printk("[elan] get_ftm_fw_check() INX/GLASS_AF_KD\n");
			if(FW_VERSION == ELAN_FW_VER_INX_AF_KD){
				//PASS
				return sprintf(buf, "Pass\n");
			}
			else{
				//FAIL
				return sprintf(buf,
				"fw check version FAIL:\n ELAN_FW_VER_INX_AF_KD = 0x%04X, FW_VERSION = 0x%04X\n ELAN_SENSOR_OPTION_INX_AF_KD = 0x%04X, SENSOR_OPTION = 0x%04X\n",
					ELAN_FW_VER_INX_AF_KD,
					FW_VERSION,
					ELAN_SENSOR_OPTION_INX_AF_KD,
					SENSOR_OPTION);
			}
		} break;

		case ELAN_SENSOR_OPTION_INX_GLASS:// Innolux/GLASS
		{
			printk("[elan] get_ftm_fw_check() INX/GLASS\n");
			if(FW_VERSION == ELAN_FW_VER_INX_GLASS){
				//PASS
				return sprintf(buf, "Pass\n");
			}
			else{
				//FAIL
				return sprintf(buf,
				"fw check version FAIL:\n ELAN_FW_VER_INX_GLASS = 0x%04X, FW_VERSION = 0x%04X\n ELAN_SENSOR_OPTION_INX_GLASS = 0x%04X, SENSOR_OPTION = 0x%04X\n",
					ELAN_FW_VER_INX_GLASS,
					FW_VERSION,
					ELAN_SENSOR_OPTION_INX_GLASS,
					SENSOR_OPTION);
			}
		} break;

		default:
		{
			printk("[elan] get_ftm_fw_check() UNKNOWN SENSOR_OPTION = 0x%04X, FW_VERSION = 0x%04X\n", SENSOR_OPTION, FW_VERSION);
			//FAIL
			return sprintf(buf,"fw check sensor option FAIL\n UNKNOWN SENSOR_OPTION = 0x%04X, FW_VERSION = 0x%04X\n", SENSOR_OPTION, FW_VERSION);
		} break;
	}
}

//colby add start
#if 1
static ssize_t get_int(struct device *dev,
struct device_attribute *attr, char *buf)
{
  
  //put_user( mt_get_gpio_in( GPIO_CTP_EINT_PIN ), ip );
//colby add start
#ifndef USE_ANDROID_M
	return sprintf(buf, "%d", mt_get_gpio_in( GPIO_CTP_EINT_PIN ));
#else
	//
	int int_val;
	int_val = __gpio_get_value(private_ts->intr_gpio);
	//return sprintf(buf, "get_int TBD: int_val = %d\n", int_val);
	return sprintf(buf, "%d", int_val);
#endif
//colby add end
}
#endif
//colby add end

//colby add start
#if 0
static ssize_t set_i2c_addr(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{

	//sscanf(buf,"%hx",private_ts->client->addr);
	printk("[elan]I2C Slave set to 0x%x", private_ts->client->addr);
	return count;
}

static ssize_t get_gesture(struct device *dev,
struct device_attribute *attr, char *buf)
{
	printk("[elan] gesture_en = %d", gesture_en);  
	return sprintf(buf, "%d\n", gesture_en);
}

static ssize_t set_gesture(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{	

	sscanf(buf,"%d",&gesture_en);
	if (gesture_en)
	printk("[elan]gesture_en Set Enable\n");
	else
	printk("[elan]gesture_en Set Disable\n");
	return count;
}

static ssize_t get_gesture_enable(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//set gesture_en to 1
	gesture_en = 1;
	printk("[elan] ENABLE gesture_en = %d", gesture_en);  
	return sprintf(buf, "%d\n", gesture_en);
}

static ssize_t get_gesture_disable(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//set gesture_en to 0
	gesture_en = 0;
	printk("[elan] DISABLE gesture_en = %d", gesture_en);  
	return sprintf(buf, "%d\n", gesture_en);
}
#endif
//colby add end

static ssize_t get_tp_id(struct device *dev,
struct device_attribute *attr, char *buf)
{
#if (CHECK_CTP_ID)
	printk("[elan] get_tp_id() CTP_ID_g = %d\n", CTP_ID_g);

	if(SENSOR_OPTION == 0x9999 && CTP_ID_g == 0){//HS + TR
		printk("[elan] HS - TR\n");
		return sprintf(buf, "HS - TR\n");
	}
	else if(SENSOR_OPTION == 0x00BA && CTP_ID_g == 0){//CPT/PMMA + TR
		printk("[elan] CP/PMMA - TR\n");
		return sprintf(buf, "CP/PMMA - TR\n");
	}
	else if(SENSOR_OPTION == 0x00BE && CTP_ID_g == 0){//CPT/Glass + TR
		printk("[elan] CP/Glass - TR\n");
		return sprintf(buf, "CP/Glass - TR\n");
	}
	else if(SENSOR_OPTION == 0x00BF && CTP_ID_g == 1){//Innolux + NON AF + KD
		printk("[elan] IN/GLASS_nonAF\n");
		return sprintf(buf, "IN/GLASS_nonAF\n");
	}
	else if(SENSOR_OPTION == 0x00D2 && CTP_ID_g == 1){//Innolux AF + KD
		printk("[elan] IN/GLASS_AF_KD\n");
		return sprintf(buf, "IN/GLASS_AF_KD\n");
	}
	else if(SENSOR_OPTION == 0x00D0 && CTP_ID_g == 1){//INX/Glass
		printk("[elan] IN/Glass\n");
		return sprintf(buf, "IN/Glass\n");
	}
	else{
		printk("[elan] UNKNOWN, SENSOR_OPTION = 0x%04X, CTP_ID_g = %d\n", SENSOR_OPTION, CTP_ID_g);
		return sprintf(buf, "UNKNOWN, SENSOR_OPTION = 0x%04X, CTP_ID_g = %d\n", SENSOR_OPTION, CTP_ID_g);
	}
#endif
#if 0
	int TP_ID_1 = 2;
	int TP_ID_2 = 2;
	TP_ID_1 = mt_get_gpio_in( TP_ID1 );
	TP_ID_2 = mt_get_gpio_in( TP_ID2 );
	
	if(TP_ID_1 != TP_ID_1_g || TP_ID_2 !=TP_ID_2_g){
		printk("[elan] WARNING!!!!!TP ID is different.");  
		return sprintf(buf, "[elan] WARNING!!!!!TP ID is different. \nTP_ID_1 = %d, TP_ID_2 = %d, TP_ID_1_g = %d, TP_ID_2_g = %d\n", TP_ID_1, TP_ID_2, TP_ID_1_g, TP_ID_2_g);
	}
	
	printk("[elan] TP_ID_1 = %d, TP_ID_2 = %d, TP_ID_1_g = %d, TP_ID_2_g = %d\n", TP_ID_1, TP_ID_2, TP_ID_1_g, TP_ID_2_g);
	
	
	if(TP_ID_1 ==0 && TP_ID_2 ==0){
		printk("[elan] (%d,%d) - TR_GFF\n", TP_ID_1, TP_ID_2);
		return sprintf(buf, "(%d,%d) - TR_GFF\n", TP_ID_1, TP_ID_2);
	}
	else if(TP_ID_1 ==0 && TP_ID_2 ==1){
		printk("[elan] (%d,%d) - HH_OGS\n", TP_ID_1, TP_ID_2);
		return sprintf(buf, "(%d,%d) - HH_OGS\n", TP_ID_1, TP_ID_2);
	}
	else if(TP_ID_1 ==1 && TP_ID_2 ==0){
		printk("[elan] (%d,%d) - TR_OGS\n", TP_ID_1, TP_ID_2);
		return sprintf(buf, "(%d,%d) - TR_OGS\n", TP_ID_1, TP_ID_2);
	}
	else if(TP_ID_1 ==1 && TP_ID_2 ==1){
		printk("[elan] (%d,%d) - N/A\n", TP_ID_1, TP_ID_2);
		return sprintf(buf, "(%d,%d) - N/A\n", TP_ID_1, TP_ID_2);
	}
	else{
		printk("[elan] (%d,%d)\n", TP_ID_1, TP_ID_2);
		}
	//return sprintf(buf, "%d, %d\n", TP_ID_1, TP_ID_2);
#endif
return sprintf(buf, "get_tp_id() is unavailable\n");
}

//colby add start
#if 0
static ssize_t get_gesture_command(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int rc;
	int major, minor;
	int gesture_mode;
	int which_gesture_mode;
	uint8_t cmd[]     = { 0x53, 0x40, 0x00, 0x01 };		/* Get Gesture state*/
	uint8_t cmd_which_gesture[]   = { 0x53, 0x30, 0x00, 0x01 };	/*Get Which Gesture*/
	uint8_t buf_recv[4] = { 0x00 };

//colby add start
#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	printk( "[elan] %s: \n", __func__);

	/* Get Gesture state */
	rc = elan_ktf2k_ts_get_data( private_ts->client, cmd, buf_recv, 4 );
	if( rc < 0 ){
		printk( "[elan] %s: Get Get Gesture state fail\n", __func__);
		return rc;
	}
	printk( "[elan] Get Get Gesture state: buf_recv[0] = 0x%2.2X, buf_recv[1] = 0x%2.2X, buf_recv[2] = 0x%2.2X, buf_recv[3] = 0x%2.2X\n", buf_recv[0], buf_recv[1] , buf_recv[2] , buf_recv[3] );
	gesture_mode = buf_recv[2] &0x0F;


	/* Get Which Gesture */
	rc = elan_ktf2k_ts_get_data( private_ts->client, cmd_which_gesture, buf_recv, 4 );
	if( rc < 0 ){
		printk( "[elan] %s: Get Which Gesture fail\n", __func__);
		return rc;
	}
	printk( "[elan] Get Which Gesture: buf_recv[0] = 0x%2.2X, buf_recv[1] = 0x%2.2X, buf_recv[2] = 0x%2.2X, buf_recv[3] = 0x%2.2X\n", buf_recv[0], buf_recv[1] , buf_recv[2] , buf_recv[3] );
	major = (( buf_recv[1] & 0x0F) << 4 ) | (( buf_recv[2] & 0xF0 ) >> 4 );
	minor = (( buf_recv[2] & 0x0F) << 4 ) | (( buf_recv[3] & 0xF0 ) >> 4 );
	which_gesture_mode = major << 8 | minor;

	printk( "[elan] : get_gesture_command OK!!\n");
	printk( "[elan] : =========================================================\n");
	printk( "[elan] : Gesture State: 0x%X\n",
	gesture_mode);
	printk( "[elan] : Which Gesture: 0x%4.4X\n",
	which_gesture_mode);
	printk( "[elan] : =========================================================\n");

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	//return 0;
	return sprintf(buf, "Gesture State: 0x%X\nWhich Gesture: 0x%4.4X\n", gesture_mode, which_gesture_mode);
}

static ssize_t set_gesture_command(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	int retval = TPD_OK;
	int gesture_enable;
	uint8_t cmd_gesture_enable[]		= { 0x54, 0x40, 0x01, 0x01 };		/* Set Gesture mode enable*/
	uint8_t cmd_gesture_disable[]		= { 0x54, 0x40, 0x00, 0x01 };		/* Set Gesture mode disable*/

	sscanf(buf,"%d",&gesture_enable);
	printk( "[elan] %s: gesture_enable = %d\n", __func__, gesture_enable);

//colby add start
#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	if (gesture_enable){
		/* Set Gesture mode enable */
		printk("[elan] TP enter into gesture mode enable, retry = %d\n", private_ts ->client ->adapter->retries);
		if(( i2c_master_send(private_ts->client, cmd_gesture_enable, sizeof( cmd_gesture_enable ))) != sizeof( cmd_gesture_enable )){
			printk("[elan] : i2c_master_send failed\n");
			return -retval;
		}
		else
			printk("[elan] : i2c_master_send success\n");
	}
	else{
		/* Set Gesture mode disable */
		printk("[elan] TP enter into gesture mode disable, retry = %d\n", private_ts ->client ->adapter->retries);
		if(( i2c_master_send(private_ts->client, cmd_gesture_disable, sizeof( cmd_gesture_disable ))) != sizeof( cmd_gesture_disable )){
			printk("[elan] : i2c_master_send failed\n");
			return -retval;
		}
		else
			printk("[elan] : i2c_master_send success\n");
	}

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	return count;
}
#endif
//colby add end

//elan add for IC frequency check 20151002 start

/*************************************
This function is designed to check analog data is correct or not.
There are six analog registers (0x8002 to 0x8007).
The analog data was erased, if the content of analog data is zero.
 *************************************/
static int analog_info(struct i2c_client *client)
{
	 //struct elan_ktf2k_ts_data *ts = private_ts;
        int rc;
        int error_code = 0;
        uint8_t cmd8002[] = {0x53, 0x80, 0x02, 0x01};
        uint8_t cmd8003[] = {0x53, 0x80, 0x03, 0x01};
        uint8_t cmd8004[] = {0x53, 0x80, 0x04, 0x01};
        uint8_t cmd8005[] = {0x53, 0x80, 0x05, 0x01};
        uint8_t cmd8006[] = {0x53, 0x80, 0x06, 0x01};
        uint8_t cmd8007[] = {0x53, 0x80, 0x07, 0x01};

        uint8_t buf_recv[4] = { 0x00 };

#ifdef ESD_CHECK
  cancel_delayed_work_sync(&esd_work);
#endif

        disable_irq(private_ts->client->irq);

        rc = elan_ktf2k_ts_get_data(client, cmd8002, buf_recv, 4);
        if (rc < 0)
        {
                printk("[elan]Get analog infomation error(address 0x8002)\n");
                enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
		queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                return rc;
        }else {
                private_ts->analog_data[0] = (buf_recv[2] <<8) | buf_recv[3];
                printk("[elan]ts->analog_data[0] = 0x%x\n", private_ts->analog_data[0]);
                if (private_ts->analog_data[0] == 0) {
                        error_code = 8002;
                        enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
			queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                        return error_code;
                }
        }

        rc = elan_ktf2k_ts_get_data(client, cmd8003, buf_recv, 4);
        if (rc < 0)
        {
                printk("[elan]Get analog infomation error(address 0x8003)\n");
                enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
		queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                return rc;
        }else {
                private_ts->analog_data[1] = (buf_recv[2] <<8) | buf_recv[3];
                printk("[elan]ts->analog_data[1] = 0x%x\n", private_ts->analog_data[1]);
                if (private_ts->analog_data[1] == 0) {
                        error_code = 8003;
                        enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
			queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                        return error_code;
                }
        }
        rc = elan_ktf2k_ts_get_data(client, cmd8004, buf_recv, 4);
        if (rc < 0)
        {
                printk("[elan]Get analog infomation error(address 0x8004)\n");
                enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
		queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                return rc;
        }else {
                private_ts->analog_data[2] = (buf_recv[2] <<8) | buf_recv[3];
                printk("[elan]ts->analog_data[2] = 0x%x\n", private_ts->analog_data[2]);
                if (private_ts->analog_data[2] == 0) {
                        error_code = 8004;
                        enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
			queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                        return error_code;
                }
        }
        rc = elan_ktf2k_ts_get_data(client, cmd8005, buf_recv, 4);
        if (rc < 0)
        {
                printk("[elan]Get analog infomation error(address 0x8005)\n");
                enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
		queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                return rc;
        }else {
                private_ts->analog_data[3] = (buf_recv[2] <<8) | buf_recv[3];
                printk("[elan]ts->analog_data[3] = 0x%x\n", private_ts->analog_data[3]);
                if (private_ts->analog_data[3] == 0) {
                        error_code = 8005;
                        enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
			queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                        return error_code;
                }
        }
        rc = elan_ktf2k_ts_get_data(client, cmd8006, buf_recv, 4);
        if (rc < 0)
        {
                printk("[elan]Get analog infomation error(address 0x8006)\n");
                enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
		queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                return rc;
        }else {
                private_ts->analog_data[4] = (buf_recv[2] <<8) | buf_recv[3];
                printk("[elan]ts->analog_data[4] = 0x%x\n", private_ts->analog_data[4]);
                if (private_ts->analog_data[4] == 0) {
                        error_code = 8006;
                        enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
			queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                        return error_code;
                }
        }
        rc = elan_ktf2k_ts_get_data(client, cmd8007, buf_recv, 4);
        if (rc < 0)
        {
                printk("[elan]Get analog infomation error(address 0x8007)\n");
                enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
		queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                return rc;
        }else {
                private_ts->analog_data[5] = (buf_recv[2] <<8) | buf_recv[3];
                printk("[elan]ts->analog_data[5] = 0x%x\n", private_ts->analog_data[5]);
                if (private_ts->analog_data[5] == 0) {
                        error_code = 8007;
                        enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
			queue_delayed_work(esd_wq, &esd_work, delay);
#endif
                        return error_code;
                }
        }

	enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
  queue_delayed_work(esd_wq, &esd_work, delay);
#endif

        return 0;
}

static ssize_t show_analog(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = analog_info(private_ts->client);
	return sprintf(buf, "%x %x %x %x %x %x error_code = %d %s\n", private_ts->analog_data[0], private_ts->analog_data[1], private_ts->analog_data[2], private_ts->analog_data[3], private_ts->analog_data[4], private_ts->analog_data[5], ret , (ret == 0) ? " [Elan] Get Analog Data OK" : "[Elan] Get Analog Data Fail");
}
//elan add for IC frequency check 20151002 end

static ssize_t get_ping_TP(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int rc;
	int x, y;
	uint8_t cmd_x[]   = { 0x53, 0x60, 0x00, 0x00 };     /*Get x resolution*/
	uint8_t cmd_y[]   = { 0x53, 0x63, 0x00, 0x00 };     /*Get y resolution*/
	uint8_t buf_recv[4] = { 0x00 };

#ifdef ESD_CHECK
  cancel_delayed_work_sync(&esd_work);
#endif

       disable_irq(private_ts->client->irq);

	printk( "[elan] %s: enter\n", __func__);

	/* X Resolution */
	  rc = elan_ktf2k_ts_get_data( private_ts->client, cmd_x, buf_recv, 4 );
	  if( rc < 0 ){
		printk( "[elan] %s: Get x resolution fail\n", __func__);
		enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
		queue_delayed_work(esd_wq, &esd_work, delay);
#endif
		return sprintf(buf, "PING FAIL(read x)\n");
		}
	  else{
		x = (( buf_recv[2] )) | (( buf_recv[3] & 0xF0 ) << 4 );
		}

	/* Y Resolution */
	  rc = elan_ktf2k_ts_get_data( private_ts->client, cmd_y, buf_recv, 4 );
	  if( rc < 0 ){
		printk( "[elan] %s: Get y resolution fail\n", __func__);
		enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
		queue_delayed_work(esd_wq, &esd_work, delay);
#endif
		return sprintf(buf, "PING FAIL(read y)\n");
		}
	  else{
		y = (( buf_recv[2] )) | (( buf_recv[3] & 0xF0 ) << 4 );
		}

	printk( "[elan] : =========================================================\n");
	printk( "[elan] : x : %d, y : %d\n", x, y);
	printk( "[elan] : X_RESOLUTION : %d, Y_RESOLUTION : %d\n", X_RESOLUTION, Y_RESOLUTION);
	printk( "[elan] : =========================================================\n");

	if(x == X_RESOLUTION && y == Y_RESOLUTION){
		enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
		queue_delayed_work(esd_wq, &esd_work, delay);
#endif
		return sprintf(buf, "PING PASS\n");
		}
	else{
		enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
		queue_delayed_work(esd_wq, &esd_work, delay);
#endif
		return sprintf(buf, "PING FAIL(wrong x.y)\n");
		}
}

static ssize_t get_sen_opt(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int rc;
	uint8_t cmd_sensor_option[]  = { CMD_R_PKT, 0xD3, 0x00, 0x01 };/* Get Sensor Option*/ //20150729 modify
	int sen_opt = 0;
	uint8_t buf_recv[4] = { 0x00 };

#ifdef ESD_CHECK
  cancel_delayed_work_sync(&esd_work);
#endif

       disable_irq(private_ts->client->irq);

	printk( "[elan] %s: enter\n", __func__);

	 /* Sensor Option */
	  rc = elan_ktf2k_ts_get_data( private_ts->client, cmd_sensor_option, buf_recv, 4 );
	  if( rc < 0 ){
		printk( "[elan] %s: Get Senaoe Oprion fail\n", __func__);
		enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
		queue_delayed_work(esd_wq, &esd_work, delay);
#endif
		return rc;
		}
	  else{
		sen_opt = (( buf_recv[3] )) | (buf_recv[2] << 8 );
		private_ts->sensor_option =sen_opt;
		SENSOR_OPTION = sen_opt;
		}

	printk( "[elan] : =========================================================\n");
	printk( "[elan] : SENSOR_OPTION : 0x%X\n", SENSOR_OPTION);
	printk( "[elan] : =========================================================\n");

	enable_irq(private_ts->client->irq);
#ifdef ESD_CHECK
  queue_delayed_work(esd_wq, &esd_work, delay);
#endif

	return sprintf(buf, "%X\n", SENSOR_OPTION);
}

static ssize_t store_tp_power(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	int power_value;
	sscanf(buf,"%d",&power_value);

	elan_power_config(private_ts);

	if (power_value){
		printk("[elan] store_tp_power start, power_value = %d\n", power_value);
		elan_power_on(private_ts, 1);
		printk("[elan] store_tp_power end\n");
		}
	else{
		printk("[elan] store_tp_power start, power_value = %d\n", power_value);
		elan_power_on(private_ts, 0);
		printk("[elan] store_tp_power end\n");
		}

	return count;
}

static ssize_t get_fw_state(struct device *dev,
struct device_attribute *attr, char *buf)
{
//fw_status:
//	0: UNKNOWN
//	1: Do update
//	2: Updating
//	3: Done
//	4: Fail

	printk( "[elan] get_fw_state() fw_status : %d\n", fw_status);

	if(fw_status ==0)
		return sprintf(buf, "UNKNOWN\n");
	else if(fw_status ==1)
		return sprintf(buf, "DO UPDATE\n");
	else if(fw_status ==2)
		return sprintf(buf, "UPDATING\n");
	else if(fw_status ==3)
		return sprintf(buf, "DONE\n");
	else if(fw_status ==4)
		return sprintf(buf, "FAIL\n");
	else
		return sprintf(buf, "UNKNOWN\n");
}

static DEVICE_ATTR(reset, 0664, show_reset, store_reset);
static DEVICE_ATTR(enable_irq, 0664, show_enable_irq, NULL);
static DEVICE_ATTR(disable_irq, 0664, show_disable_irq, NULL);
static DEVICE_ATTR(calibrate, 0664, show_calibrate, NULL);
static DEVICE_ATTR(fw_version, 0664, show_fw_version_value, NULL);
static DEVICE_ATTR(fw_id, 0664, show_fw_id_value, NULL);
static DEVICE_ATTR(bc_version, 0664, show_bc_version_value, NULL);
static DEVICE_ATTR(drv_version, 0664, show_drv_version_value, NULL);
static DEVICE_ATTR(fw_update, 0664, show_fw_update, NULL);
static DEVICE_ATTR(iap_mode, 0664, show_iap_mode, NULL);
//static DEVICE_ATTR(rawdata_result, 0664, get_rawdata, NULL); //0829 //colby mask 20151001
//static DEVICE_ATTR(exec_rawdata, 0664, NULL, exec_rawdata); //0829  //colby mask 20151001
static DEVICE_ATTR(debug, 0664, get_debug, set_debug); //1014
static DEVICE_ATTR(x_resolution, 0664, get_x_resolution, NULL); //0126
static DEVICE_ATTR(y_resolution, 0664, get_y_resolution, NULL); //0126
static DEVICE_ATTR(packet_info, 0664, get_packet_info, NULL); //0126
static DEVICE_ATTR(probe_log, 0664, get_probe_log, NULL); //0126
static DEVICE_ATTR(fw_info, 0664, get_fw_info, NULL); //0130
static DEVICE_ATTR(ftm_fw_check, 0664, get_ftm_fw_check, NULL); //0130
static DEVICE_ATTR(gpio_int, 0664, get_int, NULL); //0130 //colby mask 20151001
//static DEVICE_ATTR(i2c_addr, 0664, NULL, set_i2c_addr); //0130 //colby mask 20151001
//static DEVICE_ATTR(gesture, 0664, get_gesture, set_gesture); //colby mask 20151001
//static DEVICE_ATTR(gesture_enable, 0664, get_gesture_enable, NULL); //colby mask 20151001
//static DEVICE_ATTR(gesture_disable, 0664, get_gesture_disable, NULL); //colby mask 20151001
static DEVICE_ATTR(tp_id, 0664, get_tp_id, NULL);
//static DEVICE_ATTR(gesture_command, 0664, get_gesture_command, set_gesture_command); //colby mask 20151001
static DEVICE_ATTR(analog, S_IRUGO, show_analog, NULL); //elan add for IC frequency check 20151002
static DEVICE_ATTR(ping_TP, 0664, get_ping_TP, NULL); //20150925
static DEVICE_ATTR(sen_opt, 0664, get_sen_opt, NULL); //20151013
static DEVICE_ATTR(tp_power, 0664, NULL, store_tp_power); //20151013
static DEVICE_ATTR(fw_state, 0664, get_fw_state, NULL); //update fw_status 20151020

static struct attribute *elan_attributes[] = {
	&dev_attr_reset.attr,
	&dev_attr_enable_irq.attr,
	&dev_attr_disable_irq.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_fw_id.attr,
	&dev_attr_bc_version.attr,
	&dev_attr_drv_version.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_iap_mode.attr,
	//&dev_attr_ps_status.attr,  //0828
	//&dev_attr_rawdata_result.attr,  //0829
	//&dev_attr_exec_rawdata.attr,  //0909
	&dev_attr_debug.attr,  //1014
	&dev_attr_x_resolution.attr,
	&dev_attr_y_resolution.attr,
	&dev_attr_packet_info.attr,
	&dev_attr_probe_log.attr,
	&dev_attr_fw_info.attr,
	&dev_attr_ftm_fw_check.attr,
	&dev_attr_gpio_int.attr,//0209 //colby mask 20151001
	//&dev_attr_i2c_addr.attr,//0209
	//&dev_attr_gesture.attr,//0209
	//&dev_attr_gesture_enable.attr,//0209
	//&dev_attr_gesture_disable.attr,//0209
	&dev_attr_tp_id.attr,
	&dev_attr_analog.attr,//elan add for IC frequency check 20151002
	&dev_attr_ping_TP.attr,
	&dev_attr_sen_opt.attr,
	&dev_attr_tp_power.attr,
	&dev_attr_fw_state.attr, //update fw_status 20151020
	//&dev_attr_gesture_command.attr,
	NULL
};

static struct attribute_group elan_attribute_group = {
	.name = DEVICE_NAME,
	.attrs = elan_attributes,
};




//0814 add end

//colby add start
//#if 0
#ifdef USE_ANDROID_M
static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	printk("[elan]Device Tree Tpd_irq_registration!\n");

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		private_ts->client->irq = irq_of_parse_and_map(node, 0);
		
		ret = request_irq(private_ts->client->irq, (irq_handler_t) tpd_eint_interrupt_handler, IRQF_TRIGGER_LOW,
				"TOUCH-eint", NULL);
		if (ret > 0) {
			ret = -1;
			printk("[elan]tpd request_irq IRQ LINE NOT AVAILABLE!.\n");
		}
	} else {
		printk("[elan]tpd request_irq can not find touch eint device node!.");
		ret = -1;
	}
	printk("[elan][%s]irq:%d, debounce:%d-%d:\n", __func__, private_ts->client->irq, ints[0], ints[1]);
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "%s: irq:%d, debounce:%d-%d:\n", __func__, private_ts->client->irq, ints[0], ints[1]);
	return ret;
}
#endif
//colby add end

static int elan_power_on(struct elan_ktf2k_ts_data *ts, int on)
{
    int ret;
    printk("%s++\n", __func__);
    result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_power_on: enter\n");
    if (on)
    {
        ret = regulator_enable(tpd->reg);
        if (ret) {
            result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_power_on: Regulator vdd enable failed ret=%d\n", ret);
            printk("[elan]Regulator vdd enable failed ret=%d\n", ret);			
            //dev_err(&ts->client->dev, "[elan]Regulator vdd enable failed ret=%d\n", ret);
            return ret;
        }
/*
        ret = regulator_enable(ts->vcc_i2c);
        if (ret) {
            dev_err(&ts->client->dev,
                "[elan]Regulator vcc_i2c enable failed ret=%d\n", ret);
            regulator_disable(ts->vdd);
        }
*/
    }
    else
    {
        ret = regulator_disable(tpd->reg);
        if(ret){
            result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_power_on: Regulator vdd disable failed ret=%d\n", ret);
            printk("[elan]Regulator vdd disable failed ret=%d\n", ret);
            //dev_err(&ts->client->dev, "[elan]Regulator vdd disable failed ret=%d\n", ret);
            return ret;
        }
/*
        ret = regulator_disable(ts->vcc_i2c);
        if(ret){
            dev_err(&ts->client->dev,
                "[elan]Regulator vcc_i2c disable failed ret=%d\n", ret);
            ret = regulator_enable(ts->vdd);
            if(ret){
                dev_err(&ts->client->dev,
                    "[elan]Regulator vdd enable failed ret=%d\n", ret);
            }
        }
*/
    }
    result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_power_on: leave\n");
    printk("%s--\n", __func__);
    return ret;
}

static int elan_power_config(struct elan_ktf2k_ts_data *ts)
{
    int ret = 0;

    printk("%s ++\n", __func__);
    result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_power_config: enter\n");
    tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
    if (IS_ERR(tpd->reg)) {
        ret = PTR_ERR(tpd->reg);
        result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_power_config: Regulator get failed vdd ret=%d\n", ret);
        printk("[elan]Regulator get failed vdd ret=%d\n", ret);
        //dev_err(&ts->client->dev, "[elan]Regulator get failed vdd ret=%d\n", ret);
        return ret;
    }

    if (regulator_count_voltages(tpd->reg) > 0) {
    	printk("[elan]do regulator_set_voltage\n");
        ret = regulator_set_voltage(tpd->reg, 2800000,2800000);
        if (ret) {
            result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_power_config: Regulator set_vtg failed vdd ret=%d\n", ret);
            printk("Regulator set_vtg failed vdd ret=%d\n", ret);
            //dev_err(&ts->client->dev, "Regulator set_vtg failed vdd ret=%d\n", ret);
            goto reg_vdd_put;
        }
    }

//digital power is not connected to IC
/*
    ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
    if (IS_ERR(ts->vcc_i2c)) {
        ret = PTR_ERR(ts->vcc_i2c);
        dev_err(&ts->client->dev,
            "Regulator get failed vcc_i2c ret=%d\n", ret);
        goto reg_vdd_set_vtg;
    }

    if (regulator_count_voltages(ts->vcc_i2c) > 0) {
        ret = regulator_set_voltage(ts->vcc_i2c, ELAN_I2C_VTG_MIN_UV,
                       ELAN_I2C_VTG_MAX_UV);
        if (ret) {
            dev_err(&ts->client->dev,
            "Regulator set_vtg failed vcc_i2c ret=%d\n", ret);
            goto reg_vcc_i2c_put;
        }
    }
*/
    result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_power_config: leave\n");
    printk("%s --\n", __func__);
    return 0;
/*
reg_vcc_i2c_put:
    regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
    if (regulator_count_voltages(ts->vdd) > 0)
        regulator_set_voltage(ts->vdd, 0, ELAN_VTG_MAX_UV);
*/
reg_vdd_put:
    regulator_put(tpd->reg);
    return ret;
}

#ifdef CONFIG_OF
static int elan_parse_dt(struct elan_ktf2k_ts_data *ts, struct elan_ktf2k_i2c_platform_data *pdata)
{
    //int rc, coords_size = 0;
    //uint32_t coords[4] = {0};
    //struct property *prop;
    //struct device_node *dt = ts->client->dev.of_node;

#if 0
    prop = of_find_property(dt, "elan,panel-coords", NULL);
    if (prop) {
        coords_size = prop->length / sizeof(u32);
        if (coords_size != 4)
            printk("[elan]%s:Invalid panel coords size %d", __func__, coords_size);
    }

    if (of_property_read_u32_array(dt, "elan,panel-coords", coords, coords_size) == 0) {
        pdata->abs_x_min = coords[0], pdata->abs_x_max = coords[1];
        pdata->abs_y_min = coords[2], pdata->abs_y_max = coords[3];
        printk("[elan]DT-%s:panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
                pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
    }

    prop = of_find_property(dt, "elan,display-coords", NULL);
    if (prop) {
        coords_size = prop->length / sizeof(u32);
        if (coords_size != 4)
            printk("[elan]%s:Invalid display coords size %d", __func__, coords_size);
    }
    rc = of_property_read_u32_array(dt, "elan,display-coords", coords, coords_size);
    if (rc && (rc != -EINVAL)) {
        printk("[elan]%s:Fail to read display-coords %d\n", __func__, rc);
        return rc;
    }
#endif

    //pdata->intr_gpio = of_get_named_gpio(dt, "elan,intr-gpio", 0);
    pdata->intr_gpio = 1; //colby add for temp 20151001
    printk("[elan]DT:intr_gpio value is %d\n", pdata->intr_gpio);
    if (!gpio_is_valid(pdata->intr_gpio)) {
        printk("[elan]DT:gpio_irq value is not valid\n");
        result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_parse_dt:gpio_irq value is not valid\n");
    }

    //pdata->rst_gpio = of_get_named_gpio(dt, "elan,reset-gpio", 0);
    pdata->rst_gpio = 10; //colby add for temp 20151001
    printk("[elan]DT:rst_gpio value is %d\n", pdata->rst_gpio);
    if (!gpio_is_valid(pdata->rst_gpio)) {
        printk("[elan]DT:gpio_rst value is not valid\n");
        result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_parse_dt:gpio_rst value is not valid\n");
    }

#if 0
    pdata->tp_id_gpio = of_get_named_gpio(dt, "elan,tpid-gpio", 0);
    if (!gpio_is_valid(pdata->rst_gpio)) {
        printk("[elan]DT:gpio_tpid value is not valid\n");
    }
    printk("[elan]DT:gpio_irq=%d, gpio_rst=%d, gpio_tpid=%d\n",
                            pdata->intr_gpio, pdata->rst_gpio, pdata->tp_id_gpio);
#endif

    return 0;
}
#endif
//colby add end

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
static struct elan_ktf2k_ts_data ts;
int   err = 0;
int   fw_err = 0;
int   New_FW_ID;
int   New_FW_VER;

//colby add start
#ifdef USE_ANDROID_M
	struct elan_ktf2k_i2c_platform_data *pdata;
	int ret = 0;
#endif
//colby add end

//int   i;
int   retval = TPD_OK;
struct task_struct *fw_update_thread;	// Thunder modify for update touch Firmware 20141229

result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "probe enter\n");

  client->addr |= I2C_ENEXT_FLAG;

  client->adapter->retries = I2C_RETRIES;
  printk("[elan] : client retries is %d\n",  client->adapter->retries);

 // printk("[elan] %s: client addr is %x, TPD_DEVICE = %s\n", __func__, client->addr, TPD_DEVICE );
 // printk("[elan] %s: I2C_WR_FLAG = %x, I2C_MASK_FLAG= %x, I2C_ENEXT_FLAG = %x\n", __func__, I2C_WR_FLAG, I2C_MASK_FLAG, I2C_ENEXT_FLAG );

  //printk("[elan] %x = IOCTL_I2C_INT\n", IOCTL_I2C_INT );
  //printk("[elan] %x = IOCTL_IAP_MODE_LOCK\n", IOCTL_IAP_MODE_LOCK );
  //printk("[elan] %x = IOCTL_IAP_MODE_UNLOCK\n", IOCTL_IAP_MODE_UNLOCK );

  printk("[elan] : client addr is %x, TPD_DEVICE = %s\n",  client->addr, TPD_DEVICE );
  printk("[elan] : I2C_WR_FLAG = %x, I2C_MASK_FLAG= %x, I2C_ENEXT_FLAG = %x\n",  I2C_WR_FLAG, I2C_MASK_FLAG, I2C_ENEXT_FLAG );

//  printk("[elan] %x = IOCTL_I2C_INT\n", IOCTL_I2C_INT );
//  printk("[elan] %x = IOCTL_IAP_MODE_LOCK\n", IOCTL_IAP_MODE_LOCK );
//  printk("[elan] %x = IOCTL_IAP_MODE_UNLOCK\n", IOCTL_IAP_MODE_UNLOCK );

//client->addr   &= I2C_MASK_FLAG;
    client->timing  = 400; //I2C rate=400k Hz
  //client->timing  = 100; //I2C rate=100k Hz
#if 1
  i2c_client = client;
  private_ts = &ts;
  private_ts->client = client;
//private_ts->addr = I2C_ELAN_SLAVE_ADDR;
#endif

//colby add start
#ifndef USE_ANDROID_M
/* Power configuration. */
  elan_power_enable( ELAN_ENABLE );
  msleep( 10 );
#endif
//colby add end

//colby add start
#ifdef USE_ANDROID_M
//private_ts->pdata = pdata;

#ifdef CONFIG_OF
    if(client->dev.of_node)
    {
        pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
        if (pdata == NULL) {
            err = -ENOMEM;
            //goto err_dt_platform_data_fail; //TBD
        }    
        //result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "do elan_parse_dt enter.\n");
        ret = elan_parse_dt(private_ts, pdata);
        //result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "do elan_parse_dt leave. ret = %d\n", ret);
        if (ret < 0) { 
            printk("[elan] pdata is NULL for DT\n");
            //goto err_alloc_dt_pdata_failed; //TBD
        }
	 private_ts->intr_gpio = pdata->intr_gpio;
	 private_ts->rst_gpio = pdata->rst_gpio;
	 printk("[elan] private_ts->intr_gpio value is %d\n", private_ts->intr_gpio);
	 printk("[elan] private_ts->rst_gpio value is %d\n", private_ts->rst_gpio);
	 result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "intr_gpio = %d\nrst_gpio = %d\n", private_ts->intr_gpio, private_ts->rst_gpio);
	 private_ts->pdata = pdata;
    }

#endif

    elan_power_config(private_ts);
    elan_power_on(private_ts, 1);

#endif
//colby add end

//if TP FW is updating, dont sleep in suspend - start
	wake_lock_init(&tp_lock,WAKE_LOCK_SUSPEND,"TP FW wakelock");
	printk("[elan] wake lock init done!!\n");
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "wake lock init done!!\n");
//if TP FW is updating, dont sleep in suspend - end

#if 0
  /* LDO enable */
  mt_set_gpio_mode( GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO );
  mt_set_gpio_dir( GPIO_CTP_EN_PIN, GPIO_DIR_OUT );
  mt_set_gpio_out( GPIO_CTP_EN_PIN, GPIO_OUT_ZERO );
  msleep( 50 );
  mt_set_gpio_out( GPIO_CTP_EN_PIN, GPIO_OUT_ONE );
#endif
  printk("[elan] ELAN enter tpd_probe_ ,the I2C addr = 0x%02X\n", client->addr);
  //result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "ELAN enter tpd_probe.\n");
//printk("GPIO43 =%d,GPIO_CTP_EINT_PIN =%d,GPIO_DIR_IN=%d,CUST_EINT_TOUCH_PANEL_NUM=%d\n",GPIO43,GPIO_CTP_EINT_PIN,GPIO_DIR_IN,CUST_EINT_TOUCH_PANEL_NUM);

// check HW ID start
#if CHECK_HW_ID
g_HW_ID = board_type_with_hw_id();
printk("[elan] g_HW_ID = %d\n", g_HW_ID);
result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "g_HW_ID = %d\n", g_HW_ID);

g_CURRENT_MODE = get_ftm_pin();
printk("[elan] g_CURRENT_MODE = %d\n", g_CURRENT_MODE);
result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "g_CURRENT_MODE = %d\n", g_CURRENT_MODE);
#endif
// check HW ID end

//colby add start
//#if 0
#ifdef USE_ANDROID_M
	//set reset pin
	printk("[elan] set reset pin start, GTP_RST_PORT = %d\n", GTP_RST_PORT);
	tpd_gpio_output(GTP_RST_PORT, 1);
	mdelay( 10 );
	tpd_gpio_output(GTP_RST_PORT, 0);
	mdelay( 10 );
	tpd_gpio_output(GTP_RST_PORT, 1);
	mdelay(50);
	printk("[elan] set reset pin end\n");
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "set reset pin OK!\n");

	//config int pin
	printk("[elan] config int pin start, GTP_INT_PORT = %d\n", GTP_INT_PORT);
	tpd_gpio_as_int(GTP_INT_PORT);
	msleep( 200 );
	//register irq
	tpd_irq_registration();
	printk("[elan] config int pin end\n");
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "config int pin OK!\n");
	//enable_irq(private_ts->client->irq);
#endif
//colby add end

//colby add start
#ifndef USE_ANDROID_M
/* Reset Touch Pannel */
printk("[elan] probe reset touch pannel\n");
  mt_set_gpio_mode( GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO );
  mt_set_gpio_dir( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );
  mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
  mdelay( 10 );
//#if !defined(EVB)
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
  mdelay( 10 );
//#endif
  mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
  mdelay( 50 );
#endif
//colby add end

#if defined( HAVE_TOUCH_KEY )
int retry;
  for( retry = 0; retry < 3; retry++)
  {
    input_set_capability( tpd->dev, EV_KEY, tpd_keys_local[retry] );
  }
#endif

//colby add start
#ifndef USE_ANDROID_M
/* Setup Interrupt Pin */
  mt_set_gpio_mode( GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT );
  mt_set_gpio_dir( GPIO_CTP_EINT_PIN, GPIO_DIR_IN );
  mt_set_gpio_pull_enable( GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE );
  mt_set_gpio_pull_select( GPIO_CTP_EINT_PIN, GPIO_PULL_UP );
  msleep( 200 );

  mt_eint_set_sens( CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE );
  mt_eint_set_hw_debounce( CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN );
//<<Mel - 4/10, Modify Interrupt triger to falling.
  //mt_eint_registration( CUST_EINT_TOUCH_PANEL_NUM, /*CUST_EINT_TOUCH_PANEL_TYPE*/EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1 );
  mt_eint_registration( CUST_EINT_TOUCH_PANEL_NUM, /*CUST_EINT_TOUCH_PANEL_TYPE*/EINTF_TRIGGER_LOW, tpd_eint_interrupt_handler, 0 );
//mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, tpd_eint_interrupt_handler, 1);
//Mel - 4/10, Modify Interrupt triger to falling>>.
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
//msleep(100); //0403 mark
/* End Setup Interrupt Pin */
#endif
//colby add start

  tpd_load_status = 1;
  //result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "setup INT and RESET OK!!!\n");
//
//check CTP ID start
#if CHECK_CTP_ID
printk("[elan] read CTP_ID start\n");

  tpd_gpio_tpid(TPID_GPIO);
  msleep( 10 );
  CTP_ID_g = __gpio_get_value(TPID_GPIO);
  printk("[elan] CTP_ID_g = %d\n", CTP_ID_g);
  result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "CTP_ID_g = %d\n", CTP_ID_g);

printk("[elan] read CTP_ID end\n");
#endif
//check CTP ID end

  //result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "alloc DMA MODE enter\n");
#if defined( ELAN_DMA_MODE )
  //gpDMABuf_va = (u8 *)dma_alloc_coherent( NULL, 4096, &gpDMABuf_pa, GFP_KERNEL ); //1112
  
  gpDMABuf_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 4096, &gpDMABuf_pa, GFP_KERNEL); //1112
  if( !gpDMABuf_va )
  {
    printk(KERN_INFO "[elan] Allocate DMA I2C Buffer failed\n");
  result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "alloc DMA MODE fail...\n");
    goto EXIT;
  }
  else{
  result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "alloc DMA MODE OK!\n");
  	}
#endif
  //result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "alloc DMA MODE leave\n");

//colby add start
#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
  //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
  fw_err = elan_ktf2k_ts_setup( client );
  if( fw_err < 0 )
  {
    printk( KERN_INFO "[elan] No Elan chip inside\n");
  result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_ktf2k_ts_setup fail, go EXIT!!!\n");
    goto EXIT;
  }

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
  //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
  result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "elan_ktf2k_ts_setup OK!\n");
#if 1 /* RESET RESOLUTION: tmp use ELAN_X_MAX & ELAN_Y_MAX */
  printk("[elan] RESET RESOLUTION\n");
  input_set_abs_params( tpd->dev, ABS_X, 0,  LCM_X_MAX, 0, 0 );
  input_set_abs_params( tpd->dev, ABS_Y, 0,  LCM_Y_MAX, 0, 0 );
  input_set_abs_params( tpd->dev, ABS_MT_POSITION_X, 0, LCM_X_MAX, 0, 0 );
  input_set_abs_params( tpd->dev, ABS_MT_POSITION_Y, 0, LCM_Y_MAX, 0, 0 );
//<2014/12/26-Zihwei Shen. Fixed five muti-finger bug
  input_set_abs_params( tpd->dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0); //1226
//>2014/12/26-Zihwei Shen.
#endif

#if !defined( LCT_VIRTUAL_KEY )
  set_bit( KEY_BACK,  tpd->dev->keybit );
  set_bit( KEY_HOMEPAGE,  tpd->dev->keybit );
  set_bit( KEY_MENU,  tpd->dev->keybit );
#endif

#if 0
//set bit for KEY_GESTURE
  set_bit( KEY_GESTURE,  tpd->dev->keybit );
//
#endif

  thread = kthread_run( touch_event_handler, 0, TPD_DEVICE );
  if( IS_ERR( thread ))
  {
    retval = PTR_ERR( thread );
    printk(TPD_DEVICE "[elan]  failed to create kernel thread: %d\n", retval);
    result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "failed to create kernel thread: %d\n", retval);
    goto EXIT;
  }
  printk("[elan]  ELAN Touch Panel Device Probe %s\n", ( retval < TPD_OK ) ? "FAIL" : "PASS");

/* Firmware Update */
  /* MISC */
  ts.firmware.minor = MISC_DYNAMIC_MINOR;
  ts.firmware.name = "elan-iap";
  ts.firmware.fops = &elan_touch_fops;
  ts.firmware.mode = S_IRWXUGO;

  if( misc_register( &ts.firmware ) < 0 ){
    printk("[elan] misc_register failed!!\n");
    result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "misc_register failed!!\n");
    //goto EXIT;
    }
  else{
    printk("[elan] misc_register finished!!\n");
    result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "misc_register OK!\n");
    }
/* End Firmware Update */

#if defined( ESD_CHECK ) //0604
  INIT_DELAYED_WORK( &esd_work, elan_touch_esd_func );
  esd_wq = create_singlethread_workqueue( "esd_wq" );
  if( !esd_wq )
  {
    retval = -ENOMEM;
    printk(KERN_INFO "[elan] create singlethread workqueue failed\n");
    goto EXIT;
  }

  queue_delayed_work( esd_wq, &esd_work, delay );
#endif

#if( IAP_PORTION )
  if( IAP_PORTION )// Thunder modify for update touch Firmware 20141229
  {
  	/*
    work_lock = 1;
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);  //disable_irq(CUST_EINT_TOUCH_PANEL_NUM);
    cancel_delayed_work_sync(&esd_work);
    power_lock = 1;
    printk("[elan] start fw update");
    */
/* FW ID & FW VER*/

// check sensor option/TP ID for choosing FW start
#if (1)
	if(SENSOR_OPTION == 0x9999){//HS + TR
		printk("[elan] USE (HS - TR) FW\n");
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "USE (HS - TR) FW\n");
		New_FW_ID   = ( file_fw_data[0x7D67] << 8 ) | file_fw_data[0x7D66] ;
		New_FW_VER  = ( file_fw_data[0x7D65] << 8 ) | file_fw_data[0x7D64] ;
	}
	else if(SENSOR_OPTION == 0x00BA){//CPT/PMMA + TR
	//file_fw_data_CPT_PMMA
		printk("[elan] USE (CPT/PMMA - TR) FW\n");
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "USE (CPT/PMMA - TR) FW\n");
		New_FW_ID   = ( file_fw_data_CPT_PMMA[0x7D67] << 8 ) | file_fw_data_CPT_PMMA[0x7D66] ;
		New_FW_VER  = ( file_fw_data_CPT_PMMA[0x7D65] << 8 ) | file_fw_data_CPT_PMMA[0x7D64] ;
		//goto END_FW_UPDATE;
	}
	else if(SENSOR_OPTION == 0x00BE){//CPT/Glass + TR
	//file_fw_data_CPT_GLASS
		printk("[elan] USE (CPT/Glass - TR) FW\n");
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "USE (CPT/Glass - TR) FW\n");
		New_FW_ID   = ( file_fw_data_CPT_GLASS[0x7D67] << 8 ) | file_fw_data_CPT_GLASS[0x7D66] ;
		New_FW_VER  = ( file_fw_data_CPT_GLASS[0x7D65] << 8 ) | file_fw_data_CPT_GLASS[0x7D64] ;
		//goto END_FW_UPDATE;
	}
	else if(SENSOR_OPTION == 0x00BF){//Innolux + NON AF + KD
	//file_fw_data_INX_nonAF_KD
		printk("[elan] USE (INX/GLASS_nonAF) FW\n");
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "USE (INX/GLASS_nonAF) FW\n");
		New_FW_ID   = ( file_fw_data_INX_nonAF_KD[0x7D67] << 8 ) | file_fw_data_INX_nonAF_KD[0x7D66] ;
		New_FW_VER  = ( file_fw_data_INX_nonAF_KD[0x7D65] << 8 ) | file_fw_data_INX_nonAF_KD[0x7D64] ;
		//fw_status = 3; //update fw_status 20151020
		//goto END_FW_UPDATE;
	}
	else if(SENSOR_OPTION == 0x00D2){//Innolux AF + KD
	//file_fw_data_INX_AF_KD
		printk("[elan] USE (INX/GLASS_AF_KD) FW\n");
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "USE (INX/GLASS_AF_KD) FW\n");
		New_FW_ID   = ( file_fw_data_INX_AF_KD[0x7D67] << 8 ) | file_fw_data_INX_AF_KD[0x7D66] ;
		New_FW_VER  = ( file_fw_data_INX_AF_KD[0x7D65] << 8 ) | file_fw_data_INX_AF_KD[0x7D64] ;
		//fw_status = 3; //update fw_status 20151020
		//goto END_FW_UPDATE;
	}
	else if(SENSOR_OPTION == 0x00D0){//INX/Glass
	//file_fw_data_INX_GLASS
		printk("[elan] USE (INX/Glass) FW\n");
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "USE (INX/Glass) FW\n");
		New_FW_ID   = ( file_fw_data_INX_GLASS[0x7D67] << 8 ) | file_fw_data_INX_GLASS[0x7D66] ;
		New_FW_VER  = ( file_fw_data_INX_GLASS[0x7D65] << 8 ) | file_fw_data_INX_GLASS[0x7D64] ;
		//fw_status = 3; //update fw_status 20151020
		//goto END_FW_UPDATE;
	}
	else{
		printk("[elan] UNKNOWN Sensor Option, use (HS - TR) FW\n");// because DP1 boot code version is wrong, so download DP1 FW if sensor otion is UNKNOWN
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "UNKNOWN Sensor Option, use (HS - TR) FW\n");
		New_FW_ID   = ( file_fw_data[0x7D67] << 8 ) | file_fw_data[0x7D66] ;
		New_FW_VER  = ( file_fw_data[0x7D65] << 8 ) | file_fw_data[0x7D64] ;
	}
#else
	New_FW_ID   = ( file_fw_data[0x7D67] << 8 ) | file_fw_data[0x7D66] ;
	New_FW_VER  = ( file_fw_data[0x7D65] << 8 ) | file_fw_data[0x7D64] ;
#endif
// check sensor option/TP ID for choosing FW end

#if 0
  /* for ektf31xx 2 wire ice ex: 2wireice -b xx.bin */
    printk(" [7c16]=0x%02x,  [7c17]=0x%02x, [7c18]=0x%02x, [7c19]=0x%02x\n",  file_fw_data[31766],file_fw_data[31767],file_fw_data[31768],file_fw_data[31769]);
    New_FW_ID   = ( file_fw_data[31769] << 8 )  | file_fw_data[31768] ;
    New_FW_VER  = ( file_fw_data[31767] << 8 )  | file_fw_data[31766] ;
#endif
#if 0
  /* for ektf31xx iap ekt file   */
    printk(" [7bd8]=0x%02x,  [7bd9]=0x%02x, [7bda]=0x%02x, [7bdb]=0x%02x\n",  file_fw_data[31704],file_fw_data[31705],file_fw_data[31706],file_fw_data[31707]);
    New_FW_ID   = ( file_fw_data[31707] << 8 ) | file_fw_data[31706] ;
    New_FW_VER  = ( file_fw_data[31705] << 8 ) | file_fw_data[31704] ;
#endif

    printk("[elan] FW_ID=0x%x,   New_FW_ID=0x%x \n",  FW_ID, New_FW_ID);
    printk("[elan] FW_VERSION=0x%x,   New_FW_VER=0x%x \n",  FW_VERSION  , New_FW_VER);

/* for firmware auto-upgrade  */

  //fw_update_thread = kthread_run(Update_FW_One, NULL, "elan_update");	
#if FW_UPDATE_PROBE_CEI
result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "FW_UPDATE_PROBE_CEI enter\n");
result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "FW_VERSION =  0x%X, New_FW_VER = 0x%X\n", FW_VERSION, New_FW_VER);

// check HW ID start
#if 0 //no need to check FTM pin, just do FW update if need.
//#if CHECK_HW_ID
//update fw_status 20151020 start
//in FTM CURRENT check mode, dont do FW update
if(g_CURRENT_MODE == 1){
	printk("[elan] IN CURRENT check mode, dont do FW update!\n");
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "IN CURRENT check mode, dont do FW update!\n");
	fw_status = 3; //update fw_status 20151020
	goto END_FW_UPDATE;
}
//update fw_status 20151020 end
#endif
// check HW ID end

  //if( New_FW_ID == FW_ID ) //CEI will use GPIO TP_ID1/TP_ID2
  //{
      if( New_FW_VER > ( FW_VERSION ))
      {
		printk("[elan] do FW update!\n");
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "do FW update!\n");
		fw_status = 1; //update fw_status 20151020
		//Update_FW_One( client, RECOVERY );
		//fw_update_thread = kthread_run(Update_FW_in_Driver, NULL, "elan_update");
		//fw_update_thread = kthread_run(Update_FW_One, NULL, "elan_update");	// Thunder modify for update touch Firmware 20141229 //colby mask 20151001
		fw_update_thread = kthread_run(Update_FW_One, (void *)NULL, "elan_update");	// Thunder modify for update touch Firmware 20141229
		if(IS_ERR(fw_update_thread)){
			retval = PTR_ERR(fw_update_thread);
			printk(TPD_DEVICE "[elan]  failed to create kernel thread: %d\n", retval);
			fw_status = 4; //update fw_status 20151020
			}
      }
  //}
	else if(RECOVERY==0x80)
	{
		printk("[elan] In RECOVERY mode!\n");
		result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "In RECOVERY mode!\n");
		fw_status = 1; //update fw_status 20151020
		//Update_FW_One( client, RECOVERY );
		//fw_update_thread = kthread_run(Update_FW_in_Driver, NULL, "elan_update");
		//fw_update_thread = kthread_run(Update_FW_One, NULL, "elan_update");	// Thunder modify for update touch Firmware 20141229 //colby mask 20151001
		fw_update_thread = kthread_run(Update_FW_One, (void *)NULL, "elan_update");	// Thunder modify for update touch Firmware 20141229
		if(IS_ERR(fw_update_thread)){
			retval = PTR_ERR(fw_update_thread);
			printk(TPD_DEVICE "[elan]  failed to create kernel thread: %d\n", retval);
			fw_status = 4; //update fw_status 20151020
			}
	}
    else
    {
      //printk("[Elan] FW_ID is different!\n");
      printk("[elan] No need to do FW update!\n");
      result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "No need to do FW update!\n");
      fw_status = 3; //update fw_status 20151020
    }
result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "FW_UPDATE_PROBE_CEI leave\n");
#endif
  //20140521 tracy modify for update touch Fireware
   //Update_FW_One( client, RECOVERY );	// Thunder removed for update touch Firmware 20141229
   /*
    work_lock = 0;
    mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );  //enable_irq(CUST_EINT_TOUCH_PANEL_NUM);
    queue_delayed_work( esd_wq, &esd_work, delay );
    */
  }
#endif /* End.. (IAP_PORTION) */
//END_FW_UPDATE:

//add device node, start
        if (sysfs_create_group(&client->dev.kobj, &elan_attribute_group))
        	printk("[elan]sysfs create group error\n");
        else
        	printk("[elan]sysfs create group OK!!\n");
//add device node, end
    //sprintf(result_lines_probe, "probe OK!!!!!!!!!!!!!!!!\n");
    result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "probe OK!\n");
printk("[elan]probe ok!\n");
    return 0;

//error handling, start
EXIT:
//add device node, start
        if (sysfs_create_group(&client->dev.kobj, &elan_attribute_group))
        	printk("[elan]sysfs create group error\n");
        else
        	printk("[elan]sysfs create group OK!!\n");
//add device node, end

//if TP FW is updating, dont sleep in suspend - start
	wake_lock_destroy(&tp_lock);
	printk("[elan] wake lock destroy done!!\n");
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "wake lock destroy done!!\n");
//if TP FW is updating, dont sleep in suspend - end

#if defined( ELAN_DMA_MODE )
  if( gpDMABuf_va )
  {
    dma_free_coherent( NULL, 4096, gpDMABuf_va, gpDMABuf_pa );
    gpDMABuf_va = NULL;
    //gpDMABuf_pa = NULL; //colby mask 20151001
    gpDMABuf_pa = 0;
  }
#endif

//colby add start
#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
    //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );

//colby add start
#ifdef USE_ANDROID_M
	tpd_gpio_output(GTP_RST_PORT, 0);
#else
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
#endif
//colby add end
    //mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
    //mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    printk("[elan] probe fail, return -1\n");
    //sprintf(result_lines_probe, "probe fail\n");
    result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "probe fail\n");
    return -1;
//error handling, end
}

#if defined( ESD_CHECK ) //0604
static void elan_touch_esd_func(struct work_struct *work)
{
//int   res;

  printk("[elan] %s: enter\n", __FUNCTION__);  /* elan_dlx */

  if( have_interrupts == 1 )
  {
    //printk("[elan esd] %s: had interrup not need check\n", __func__);
    printk("[elan] %s: had int, no need check\n", __FUNCTION__);
  }
  else
  {
//colby add start
#ifdef USE_ANDROID_M
//
	//printk("[elan] elan_touch_esd_func start, GTP_RST_PORT = %d\n", GTP_RST_PORT);
	printk("[elan] %s: no int received, reset TPIC\n", __FUNCTION__);
	tpd_gpio_output(GTP_RST_PORT, 1);
	mdelay( 10 );
	tpd_gpio_output(GTP_RST_PORT, 0);
	mdelay( 10 );
	tpd_gpio_output(GTP_RST_PORT, 1);
	mdelay(50);
	//printk("[elan] elan_touch_esd_func end\n");
#else
	printk("[elan] esd reset enter\n");
    mt_set_gpio_mode( GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO );  /* 0 */
    mt_set_gpio_dir( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );  /* 1 */
    mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ZERO );
    msleep( 10 );

  /* for enable/reset pin */
    mt_set_gpio_mode( GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO );  /* 0 */
    mt_set_gpio_dir( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );  /* 1 */
    mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
    msleep( 100 );
#endif
//colby add end
  }

  have_interrupts = 0;
  queue_delayed_work( esd_wq, &esd_work, delay );
  printk("[elan] %s: exit\n", __FUNCTION__ );  /* elan_dlx */
}
#endif


static int tpd_remove(struct i2c_client *client)

{
  printk("[elan] TPD removed\n");

//add device node, start
	sysfs_remove_group(&client->dev.kobj, &elan_attribute_group);
//add device node, end

//if TP FW is updating, dont sleep in suspend - start
	wake_lock_destroy(&tp_lock);
	printk("[elan] wake lock destroy done!!\n");
	result_lines_index += snprintf(&result_lines_probe[result_lines_index], sizeof(result_lines_probe) - result_lines_index, "wake lock destroy done!!\n");
//if TP FW is updating, dont sleep in suspend - end

#if defined( ELAN_DMA_MODE )
  if( gpDMABuf_va )
  {
    dma_free_coherent( NULL, 4096, gpDMABuf_va, gpDMABuf_pa );
    gpDMABuf_va = NULL;
    //gpDMABuf_pa = NULL; //colby mask 20151001
    gpDMABuf_pa = 0;
  }
#endif

  return 0;
}


//static void tpd_suspend(struct i2c_client *client, pm_message_t message) //colby mask 20151001
static void tpd_suspend(struct device *h)
{
//int retval = TPD_OK; //colby mask 20151001
//static char data = 0x3; //colby mask 20151001
uint8_t cmd[] = { CMD_W_PKT, 0x50, 0x00, 0x01 };
uint8_t cmd_gesture[] = { 0x54, 0x40, 0x01, 0x01 };// gesture mode

  printk("[elan] tpd_suspend++\n");

if(gesture_en){
  //enter gesture mode
  printk("[elan] TP enter into gesture mode, retry = %d\n", private_ts ->client ->adapter->retries);

  if(( i2c_master_send(private_ts->client, cmd_gesture, sizeof( cmd_gesture ))) != sizeof( cmd_gesture ))
  {
    printk("[elan] : i2c_master_send failed\n");
    //return -retval;  //colby mask 20151001
  }
  else
    printk("[elan] : i2c_master_send success\n");
}
else{
  //enter deep sleep mode  
  printk("[elan] TP enter into sleep mode, retry = %d\n", private_ts ->client ->adapter->retries);

//colby add start
#ifdef ESD_CHECK
  cancel_delayed_work_sync(&esd_work);
#endif

#ifdef USE_ANDROID_M
  disable_irq(private_ts->client->irq);
#else
  mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
//colby add end
  //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );

//if TP FW is updating, dont sleep in suspend - start
if(fw_status == 1 || fw_status == 2){
	fw_lock_status = fw_lock_suspend; //into sleep mode after TP FW update done
	printk("[elan] FW in updating, dont into sleep mode.\n");
}
else{
	fw_lock_status = fw_lock_disable;

  if(( i2c_master_send(private_ts->client, cmd, sizeof( cmd ))) != sizeof( cmd ))
  {
    //printk("[elan] %s: i2c_master_send failed\n", __func__);
    printk("[elan] i2c_master_send failed\n");
    //return -retval; //colby mask 20151001
  }
}
//if TP FW is updating, dont sleep in suspend - end

  //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
}

#if 0
  elan_power_enable( ELAN_DISABLE );
#endif
  printk("[elan] tpd_suspend--\n");
  //return retval; //colby mask 20151001
}


//static void tpd_resume(struct i2c_client *client) //colby mask 20151001
static void tpd_resume(struct device *h)
{
//int retval = TPD_OK; //colby mask 20151001
//uint8_t cmd[] = {CMD_W_PKT, 0x58, 0x00, 0x01}; //colby mask 20151001

  printk("[elan] tpd_resume++\n");

  //printk("[elan] %s wake up\n", __func__);
  //printk("[elan]  wake up\n");
//add different TP source info when resume, start
	if(SENSOR_OPTION == 0x9999){//HS + TR
		printk("[elan] (HS - TR) wake up\n");
	}
	else if(SENSOR_OPTION == 0x00BA){//CPT/PMMA + TR
		printk("[elan] (CPT/PMMA - TR) wake up\n");
	}
	else if(SENSOR_OPTION == 0x00BE){//CPT/Glass + TR
		printk("[elan] (CPT/Glass - TR) wake up\n");
	}
	else if(SENSOR_OPTION == 0x00BF){//Innolux + NON AF + KD
		printk("[elan] (INX/GLASS_nonAF) wake up\n");
	}
	else if(SENSOR_OPTION == 0x00D2){//Innolux AF + KD
		printk("[elan] (INX/GLASS_AF_KD) wake up\n");
	}
	else if(SENSOR_OPTION == 0x00D0){//INX/Glass
		printk("[elan] (INX/Glass) wake up\n");
	}
	else{
		printk("[elan] (UNKNOWN) wake up\n");
		}
//add different TP source info when resume, end
#if 0
  elan_power_enable( ELAN_ENABLE );
#endif

#if 1
/* Reset Touch Pannel */

//colby add start
//if TP FW is updating, dont sleep in suspend - start
if(fw_status == 1 || fw_status == 2){
	fw_lock_status = fw_lock_resume;
	printk("[elan] FW in updating, dont reset.\n");
}
else{
	fw_lock_status = fw_lock_disable;

#ifdef USE_ANDROID_M
//
	printk("[elan] set reset pin start, GTP_RST_PORT = %d\n", GTP_RST_PORT);
	tpd_gpio_output(GTP_RST_PORT, 1);
	mdelay( 10 );
	tpd_gpio_output(GTP_RST_PORT, 0);
	mdelay( 10 );
	tpd_gpio_output(GTP_RST_PORT, 1);
	//mdelay(50); //no need to wait hello packet
	printk("[elan] set reset pin end\n");
#else
  mt_set_gpio_mode( GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO );
  mt_set_gpio_dir( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );
  mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
  mdelay(10);
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO );
  mdelay(10);
  mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
  //mdelay(120); // no need to wait hello packet
#endif
}
//if TP FW is updating, dont sleep in suspend - end
//colby add end

#else
  if(( i2c_master_send( private_ts->client, cmd, sizeof( cmd ))) != sizeof( cmd ))
  {
    printk("[elan] %s: i2c_master_send failed\n", __func__);
    //return -retval; //colby mask 20151001
  }
  msleep(200);
#endif

#ifdef USE_ANDROID_M
  enable_irq(private_ts->client->irq);
#else
  mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
#endif
  //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );

#ifdef ESD_CHECK
  queue_delayed_work(esd_wq, &esd_work, delay);
#endif

  printk("[elan] tpd_resume--\n");
  //return retval; //colby mask 20151001
}

static int tpd_local_init(void)
{
  printk("[elan]: I2C Touchscreen Driver init\n");
  if(i2c_add_driver( &tpd_i2c_driver ) != 0)
  {
    printk("[elan]: unable to add i2c driver.\n");
    return -1;
  }

  if( tpd_load_status == 0 )
  {
    printk("ektf2152 add error touch panel driver.\n");
    i2c_del_driver( &tpd_i2c_driver );
    return -1;
  }

#if defined( TPD_HAVE_BUTTON )
#if defined( LCT_VIRTUAL_KEY )
  tpd_button_setting( TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local );  /* initialize tpd button data */
#endif
#endif
#if ( defined( TPD_WARP_START ) && defined( TPD_WARP_END ))
  TPD_DO_WARP = 1;
  memcpy( tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4 );
  memcpy( tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4 );
#endif
#if ( defined( TPD_HAVE_CALIBRATION ) && !defined( TPD_CUSTOM_CALIBRATION ))
  memcpy( tpd_calmat, tpd_def_calmat_local, 8 * 4 );
  memcpy( tpd_def_calmat, tpd_def_calmat_local, 8 * 4 );
#endif
  printk("[elan] end %s, %d\n", __FUNCTION__, __LINE__ );
  tpd_type_cap = 1;
  return 0;
}


static struct tpd_driver_t tpd_device_driver =
{
  .tpd_device_name = "ektf2k_mtk",
  .tpd_local_init = tpd_local_init,
  .suspend = tpd_suspend,
  .resume = tpd_resume,
//colby add start
#if 1
#if defined( TPD_HAVE_BUTTON )
  .tpd_have_button = 1,
#else
  .tpd_have_button = 0,
#endif
#endif
//colby add end
};

static int __init tpd_driver_init(void)
{
  printk("[elan]: Driver Verison MTK0005 for MTK67XX serial\n");
#if defined( ELAN_MTK6577 ) || defined( ELAN_MTK6752 )
  printk("[elan] Enable ELAN_MTK6752\n");
  i2c_register_board_info( I2C_ELAN_BUS, &i2c_tpd, 1 );
#endif
  if( tpd_driver_add( &tpd_device_driver ) < 0 )
  {
    //printk("[elan]: %s driver failed\n", __func__);
    printk("[elan]: driver failed\n");
  }

  return 0;
}

static void __exit tpd_driver_exit(void)
{
  printk("[elan]: %s elan touch panel driver exit\n", __func__ );
  tpd_driver_remove( &tpd_device_driver );
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);




