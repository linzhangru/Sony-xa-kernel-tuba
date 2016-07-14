#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"


#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif
#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt_gpio_core.h>

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))
#define UDELAY(n) 											(lcm_util.udelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

#define set_gpio_lcd_enp(cmd) 		lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enn(cmd) 		lcm_util.set_gpio_lcd_enn_bias(cmd)
#define set_gpio_lcd_bkl(cmd) 		lcm_util.set_gpio_lcm_backlight(cmd)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
/***************************************************************************** 
 * Define
 *****************************************************************************/
#ifndef CONFIG_FPGA_EARLY_PORTING
#define KTD_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL//for I2C channel 0
#define I2C_ID_NAME "ktd2151"
#define KTD_ADDR 0x3E
/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
 #if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info ktd2151_board_info __initdata = {I2C_BOARD_INFO(I2C_ID_NAME, KTD_ADDR)};
#endif
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
		{.compatible = "mediatek,I2C_LCD_BIAS"},
		{},
};
#endif
/*static struct i2c_client *ktd2151_i2c_client = NULL;*/
struct i2c_client *ktd2151_i2c_client;

/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int ktd2151_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ktd2151_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

 struct ktd2151_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id ktd2151_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif
static struct i2c_driver ktd2151_iic_driver = {
	.id_table	= ktd2151_id,
	.probe		= ktd2151_probe,
	.remove		= ktd2151_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ktd2151",
#if !defined(CONFIG_MTK_LEGACY)
		.of_match_table = lcm_of_match,
#endif
	},
 
};
/***************************************************************************** 
 * Extern Area
 *****************************************************************************/ 
 
 

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int ktd2151_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk( "*********hx8394f ktd2151_iic_probe\n");
	printk("*********hx8394f KTD: info==>name=%s addr=0x%x\n",client->name,client->addr);
	ktd2151_i2c_client  = client;		
	return 0;      
}


static int ktd2151_remove(struct i2c_client *client)
{  	
	printk( "*********hx8394f ktd2151_remove\n");
	ktd2151_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}


 /*static int ktd2151_write_bytes(unsigned char addr, unsigned char value)*/
int ktd2151_write_bytes(unsigned char addr, unsigned char value) 
{	
	int ret = 0;
	struct i2c_client *client = ktd2151_i2c_client;
	char write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
	printk("*********hx8394f ktd2151 write data fail !!\n");	
	return ret ;
}



/*
 * module load/unload record keeping
 */

static int __init ktd2151_iic_init(void)
{

   printk( "*********hx8394f ktd2151_iic_init\n");
#if defined(CONFIG_MTK_LEGACY)
   i2c_register_board_info(KTD_I2C_BUSNUM, &ktd2151_board_info, 1);
#endif
   printk( "*********hx8394f ktd2151_iic_init2\n");
   i2c_add_driver(&ktd2151_iic_driver);
   printk( "*********hx8394f ktd2151_iic_init success\n");	
   return 0;
}

static void __exit ktd2151_iic_exit(void)
{
  printk( "*********hx8394f ktd2151_iic_exit\n");
  i2c_del_driver(&ktd2151_iic_driver);  
}


module_init(ktd2151_iic_init);
module_exit(ktd2151_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK KTD2151 I2C Driver");
MODULE_LICENSE("GPL"); 
#endif
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE  0

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)
#define GPIO_TP_ID 23
#define GPIO_LCM_ID 54
#define GPIO_RESET_PIN 158

#define REGFLAG_DELAY             								0xFC
#define REGFLAG_UDELAY             								0xFB

#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER
#define REGFLAG_RESET_LOW       								0xFE
#define REGFLAG_RESET_HIGH      								0xFF


#define HX8394F_HD_ID  (0x94)

//static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef BUILD_LK
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------



#define _LCM_DEBUG_

#ifdef BUILD_LK
#define printk printf
#endif

#ifdef _LCM_DEBUG_
#define lcm_debug(fmt, args...) printk(fmt, ##args)
#else
#define lcm_debug(fmt, args...) do { } while (0)
#endif

#ifdef _LCM_INFO_
#define lcm_info(fmt, args...) printk(fmt, ##args)
#else
#define lcm_info(fmt, args...) do { } while (0)
#endif
#define lcm_err(fmt, args...) printk(fmt, ##args)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

extern int get_cci_hw_id(void);

static void init_lcm_registers(void)
{
	unsigned int data_array[16];

	printk("%s - start\n",__func__);
	//0x00 , 0x92
	//0xff,0x10,0x02
	//data_array[0] =  0x92001500;
	//dsi_set_cmdq(data_array, 1, 1);

	//data_array[0] = 0x00033902;
	//data_array[1] = 0x000210FF;
	//dsi_set_cmdq(data_array, 2, 1);


	data_array[0] = 0x00043902;
	data_array[1] = 0x9483FFB9;
	dsi_set_cmdq(data_array, 2, 1);

	//0xB1,0x50,0x15,0x75
	//0x09,0x32,0x44,0x71
	//0x31,0x55,0x2F
	data_array[0] = 0x000B3902;
	data_array[1] = 0x751550B1;
	data_array[2] = 0x71443209;
	data_array[3] = 0x002F5531;
	dsi_set_cmdq(data_array, 4, 1);

	// 0xBA,0x63,0x03,0x68
	// 0x6B,0xB2,0xC0
	data_array[0] = 0x00073902;
	data_array[1] = 0x680363BA;
	data_array[2] = 0x00C0B26B;
	dsi_set_cmdq(data_array, 3, 1);

	// 0xD2,0x88
	data_array[0] =  0x88D21500;
	dsi_set_cmdq(data_array, 1, 1);

	// 0xB2,0x00,0x80,0x64
	// 0x10,0x07
	data_array[0] = 0x00063902;
	data_array[1] = 0x648000B2;
	data_array[2] = 0x00000710;
	dsi_set_cmdq(data_array, 3, 1);

	// 0xB4,0x01,0x65,0x01
	// 0x65,0x01,0x65,0x01
	// 0x05,0x7E,0x35,0x00
	// 0x3F,0x01,0x65,0x01
	// 0x65,0x01,0x65,0x01
	// 0x05,0x7E
	data_array[0] = 0x00163902;
	data_array[1] = 0x016501B4;
	data_array[2] = 0x01650165;
	data_array[3] = 0x00357E05;
	data_array[4] = 0x0165013F;
	data_array[5] = 0x01650165;
	data_array[6] = 0x00007E05;
	dsi_set_cmdq(data_array, 7, 1);

	// 0xD3,0x00,0x00,0x06
	// 0x06,0x40,0x1A,0x08
	// 0x00,0x32,0x10,0x07
	// 0x00,0x07,0x54,0x15
	// 0x0F,0x05,0x04,0x02
	// 0x12,0x10,0x05,0x07
	// 0x33,0x33,0x0B,0x0B
	// 0x37,0x10,0x07,0x07
	// 0x10,0x40
	data_array[0] = 0x00223902;
	data_array[1] = 0x060000D3;
	data_array[2] = 0x081A4006;
	data_array[3] = 0x07103200;
	data_array[4] = 0x15540700;
	data_array[5] = 0x0204050F;
	data_array[6] = 0x07051012;
	data_array[7] = 0x0B0B3333;
	data_array[8] = 0x07071037;
	data_array[9] = 0x00004010;
	dsi_set_cmdq(data_array, 10, 1);

	//Set GIP
	// 0xD5,0x19,0x19,0x18
	// 0x18,0x1B,0x1B,0x1A
	// 0x1A,0x04,0x05,0x06
	// 0x07,0x00,0x01,0x02
	// 0x03,0x20,0x21,0x18
	// 0x18,0x22,0x23,0x18
	// 0x18,0x18,0x18,0x18
	// 0x18,0x18,0x18,0x18
	// 0x18,0x18,0x18,0x18
	// 0x18,0x18,0x18,0x18
	// 0x18,0x18,0x18,0x18
	// 0x18
	data_array[0] = 0x002D3902;
	data_array[1] = 0x181919D5;
	data_array[2] = 0x1A1B1B18;
	data_array[3] = 0x0605041A;
	data_array[4] = 0x02010007;
	data_array[5] = 0x18212003;
	data_array[6] = 0x18232218;
	data_array[7] = 0x18181818;
	data_array[8] = 0x18181818;
	data_array[9] = 0x18181818;
	data_array[10] = 0x18181818;
	data_array[11] = 0x18181818;
	data_array[12] = 0x00000018;
	dsi_set_cmdq(data_array, 13, 1);

	//Set D6
	// 0xD6,0x18,0x18,0x19
	// 0x19,0x1B,0x1B,0x1A
	// 0x1A,0x03,0x02,0x01
	// 0x00,0x07,0x06,0x05
	// 0x04,0x23,0x22,0x18
	// 0x18,0x21,0x20,0x18
	// 0x18,0x18,0x18,0x18
	// 0x18,0x18,0x18,0x18
	// 0x18,0x18,0x18,0x18
	// 0x18,0x18,0x18,0x18
	// 0x18,0x18,0x18,0x18
	// 0x18
	data_array[0] = 0x002D3902;
	data_array[1] = 0x191818D6;
	data_array[2] = 0x1A1B1B19;
	data_array[3] = 0x0102031A;
	data_array[4] = 0x05060700;
	data_array[5] = 0x18222304;
	data_array[6] = 0x18202118;
	data_array[7] = 0x18181818;
	data_array[8] = 0x18181818;
	data_array[9] = 0x18181818;
	data_array[10] = 0x18181818;
	data_array[11] = 0x18181818;
	data_array[12] = 0x00000018;
	dsi_set_cmdq(data_array, 13, 1);

	//Set Gamma
	// 0xE0,0x00,0x01,0x07
	// 0x0B,0x0D,0x11,0x13
	// 0x11,0x25,0x35,0x48
	// 0x4A,0x57,0x6D,0x76
	// 0x7C,0x8C,0x91,0x8E
	// 0x9F,0xB0,0x57,0x57
	// 0x5A,0x60,0x64,0x6A
	// 0x72,0x7F,0x00,0x01
	// 0x07,0x0B,0x0D,0x10
	// 0x13,0x11,0x25,0x35
	// 0x48,0x4A,0x57,0x6D
	// 0x76,0x7C,0x8C,0x91
	// 0x8E,0x9F,0xB0,0x57
	// 0x57,0x5A,0x60,0x64
	// 0x6A,0x72,0x7F
	data_array[0] = 0x003B3902;
	data_array[1] = 0x070100E0;
	data_array[2] = 0x13110D0B;
	data_array[3] = 0x48352511;
	data_array[4] = 0x766D574A;
	data_array[5] = 0x8E918C7C;
	data_array[6] = 0x5757B09F;
	data_array[7] = 0x6A64605A;
	data_array[8] = 0x01007F72;
	data_array[9] = 0x100D0B07;
	data_array[10] = 0x35251113;
	data_array[11] = 0x6D574A48;
	data_array[12] = 0x918C7C76;
	data_array[13] = 0x57B09F8E;
	data_array[14] = 0x64605A57;
	data_array[15] = 0x007F726A;
	dsi_set_cmdq(data_array, 16, 1);

	//Set Panel
	// 0xCC,0x03
	data_array[0] =  0x0BCC1500;
	dsi_set_cmdq(data_array, 1, 1);

	//Set C0
	//0xC0,0x1F,0x73
	data_array[0] = 0x00033902;
	data_array[1] = 0x00731FC0;
	dsi_set_cmdq(data_array, 2, 1);

	//Set VCOM
	//0xB6,0x3F,0x3F
	data_array[0] = 0x00033902;
	data_array[1] = 0x008383B6;
	dsi_set_cmdq(data_array, 2, 1);

	//Set D4h
	// 0xD4,0x02
	data_array[0] =  0x02D41500;
	dsi_set_cmdq(data_array, 1, 1);

	//Sleep Out
	//0x11
	data_array[0]=0x00110500; // sleep out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	//Set Power Option
	//0xBF,0x40,0x81,0x50
	//0x02,0x1A,0xFC,0x02
	//data_array[0] = 0x00083902;
	//data_array[1] = 0x508140BF;
	//data_array[2] = 0x02FC1A02;
	//dsi_set_cmdq(data_array, 3, 1);

	//Display On
	data_array[0] = 0x00290500; // Display on
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);   
	printk("%s - end\n",__func__);
}

static void init_lcm_registers_cpt(void)
{
	unsigned int data_array[16];
	
	printk("%s - start\n",__func__);
	//0xB9,0xFF,0x83,0x94;
	data_array[0] = 0x00043902;
	data_array[1] = 0x9483FFB9;
	dsi_set_cmdq(data_array, 2, 1);

	//0xBA,0x63,0x03,0x68,0x6B,0xB2,0xC0
	data_array[0] = 0x00073902;
	data_array[1] = 0x680363BA;
	data_array[2] = 0x00C0B26B;
	dsi_set_cmdq(data_array, 3, 1);

	//0xB1,0x50,0x12,0x72,
	//0x09,0x33,0x54,0x71,
	//0x51,0x50,0x34
	data_array[0] = 0x000B3902;
	data_array[1] = 0x721250B1;
	data_array[2] = 0x71333309;
	data_array[3] = 0x00345031;
	dsi_set_cmdq(data_array, 4, 1);

	//0xB2,0x00,0x80,0x64,
	//0x10,0x07,0x2F
	data_array[0] = 0x00073902;
	data_array[1] = 0x648000B2;
	data_array[2] = 0x002F0710;
	dsi_set_cmdq(data_array, 3, 1);

	//0xBC,0x07
	data_array[0] =  0x07BC1500;
	dsi_set_cmdq(data_array, 1, 1);

	/*0xB4,0x74,0x76,0x0D,
	  0x5C,0x5A,0x5B,0x01,
	  0x01,0x7E,0x85,0x00,
	  0x3F,0x74,0x76,0x0D,
	  0x5C,0x5A,0x5B,0x01,
	  0x01,0x7E*/
	data_array[0] = 0x00163902;
	data_array[1] = 0x507674B4;
	data_array[2] = 0x015C505C;
	data_array[3] = 0x00857E0C;
	data_array[4] = 0x0D76743F;
	data_array[5] = 0x015B5A5C;
	data_array[6] = 0x00007E01;
	dsi_set_cmdq(data_array, 7, 1);

	/*0xD2,0x44*/
	data_array[0] =  0x44D21500;
	dsi_set_cmdq(data_array, 1, 1);

	/*0xE0,0x00,0x0C,0x19,
	  0x1F,0x26,0x2C,0x2D,
	  0x2D,0x57,0x69,0x74,
	  0x74,0x78,0x87,0x8B,
	  0x8F,0x9C,0xA0,0xA4,
	  0xAC,0xB8,0x5A,0x5C,
	  0x60,0x68,0x6C,0x70,
	  0x78,0x7F,0x00,0x0C,
	  0x18,0x1F,0x26,0x2C,
	  0x2D,0x2D,0x57,0x69,
	  0x74,0x74,0x78,0x87,
	  0x8B,0x8F,0x9C,0xA0,
	  0xA4,0xAC,0xB8,0x5A,
	  0x5C,0x60,0x68,0x6C,
	  0x70,0x78,0x7F*/
	data_array[0] = 0x003B3902;
	data_array[1] = 0x1A0E00E0;
	data_array[2] = 0x2B292521;
	data_array[3] = 0x7567572B;
	data_array[4] = 0x98928075;
	data_array[5] = 0x9EA9A497;
	data_array[6] = 0x6162C6B1;
	data_array[7] = 0x716C6965;
	data_array[8] = 0x0E007373;
	data_array[9] = 0x2925211A;
	data_array[10] = 0x67572B2B;
	data_array[11] = 0x92807575;
	data_array[12] = 0xB5AEA198;
	data_array[13] = 0x62C6C1B2;
	data_array[14] = 0x6C696561;
	data_array[15] = 0x007F7371;
	dsi_set_cmdq(data_array, 16, 1);

	/*0xD3,0x00,0x00,0x06,
	  0x06,0x40,0x07,0x10,
	  0x10,0x32,0x10,0x01,
	  0x00,0x01,0x52,0x15,
	  0x0F,0x05,0x0F,0x32,
	  0x10,0x00,0x00,0x00,
	  0x67,0x44,0x05,0x05,
	  0x37,0x0E,0x0E,0x27,
	  0x0A,0x40*/
	data_array[0] = 0x00223902;
	data_array[1] = 0x0C0000D3;
	data_array[2] = 0x1007400C;
	data_array[3] = 0x01103210;
	data_array[4] = 0x15520100;
	data_array[5] = 0x320F050F;
	data_array[6] = 0x00000010;
	data_array[7] = 0x05054467;
	data_array[8] = 0x270E0E37;
	data_array[9] = 0x0000400A;
	dsi_set_cmdq(data_array, 10, 1);

	/*0xD5,0x24,0x25,0x18,
	  0x18,0x19,0x19,0x18,
	  0x18,0x18,0x18,0x00,
	  0x01,0x04,0x05,0x02,
	  0x03,0x18,0x18,0x18,
	  0x18,0x06,0x07,0x20,
	  0x21,0x18,0x18,0x18,
	  0x18,0x18,0x18,0x18,
	  0x18,0x18,0x18,0x18,
	  0x18,0x18,0x18,0x18,
	  0x18,0x18,0x18,0x18,
	  0x18*/
	data_array[0] = 0x002D3902;
	data_array[1] = 0x182524D5;
	data_array[2] = 0x18191918;
	data_array[3] = 0x00181818;
	data_array[4] = 0x02050401;
	data_array[5] = 0x18181803;
	data_array[6] = 0x20070618;
	data_array[7] = 0x18181821;
	data_array[8] = 0x18181818;
	data_array[9] = 0x18181818;
	data_array[10] = 0x18181818;
	data_array[11] = 0x18181818;
	data_array[12] = 0x00000018;
	dsi_set_cmdq(data_array, 13, 1);

	/*0xD6,0x21,0x20,0x19,
	  0x19,0x18,0x18,0x18,
	  0x18,0x18,0x18,0x07,
	  0x06,0x03,0x02,0x05,
	  0x04,0x18,0x18,0x18,
	  0x18,0x01,0x00,0x25,
	  0x24,0x18,0x18,0x18,
	  0x18,0x18,0x18,0x18,
	  0x18,0x18,0x18,0x18,
	  0x18,0x18,0x18,0x18,
	  0x18,0x18,0x18,0x18,
	  0x18*/
	data_array[0] = 0x002D3902;
	data_array[1] = 0x192021D6;
	data_array[2] = 0x18181819;
	data_array[3] = 0x07181818;
	data_array[4] = 0x05020306;
	data_array[5] = 0x18181804;
	data_array[6] = 0x25000118;
	data_array[7] = 0x18181824;
	data_array[8] = 0x18181818;
	data_array[9] = 0x18181818;
	data_array[10] = 0x18181818;
	data_array[11] = 0x18181818;
	data_array[12] = 0x00000018;
	dsi_set_cmdq(data_array, 13, 1);

  	//Set Panel
	// 0xCC,0x03
	data_array[0] =  0x0BCC1500;
	dsi_set_cmdq(data_array, 1, 1);
 
	//0xC0,0x1F,0x73
	data_array[0] = 0x00033902;
	data_array[1] = 0x00731FC0;
	dsi_set_cmdq(data_array, 2, 1);

	//0xB6,0x94,0x94
	data_array[0] = 0x00033902;
	data_array[1] = 0x009090B6;
	dsi_set_cmdq(data_array, 2, 1);

	//0xC7,0x00,0xC0,0x40,
	//0xC0
	data_array[0] = 0x00053902;
	data_array[1] = 0x000000C7;
	data_array[2] = 0x00000000;
	dsi_set_cmdq(data_array, 3, 1);

	//Modify for black screen issue - part1 S 
	// 0xBD,0x01
	data_array[0] =  0x01BD1500;
	dsi_set_cmdq(data_array, 1, 1);
	//0xB1,0x00,0x00
	data_array[0] = 0x00033902;
	data_array[1] = 0x000000B1;
	dsi_set_cmdq(data_array, 2, 1);
	// 0xBD,0x00
	data_array[0] =  0x00BD1500;
	dsi_set_cmdq(data_array, 1, 1);
	//Modify for black screen issue - part1 E
	
	data_array[0]=0x00110500; // sleep out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	//Modify for black screen issue - part2 S
	data_array[0] = 0x000D3902;
	data_array[1] = 0x648000B2;
	data_array[2] = 0x002F0710;
	data_array[3] = 0xC0000000;
	data_array[4] = 0x00000018;
	dsi_set_cmdq(data_array, 5, 1);
	//Modify for black screen issue - part2 E
	
	//Display On
	data_array[0] = 0x00290500; // Display on
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);
	
	printk("%s - end\n",__func__);
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		int hw_version=0;
		hw_version = get_cci_hw_id();
		hw_version = (hw_version & 0x07);
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		params->physical_width=62;
   		params->physical_height=110;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
		params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
		params->dsi.switch_mode = CMD_MODE;
#endif
		params->dsi.switch_mode_enable = 0;
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		if(hw_version == 1 || hw_version == 0)
		{
			params->dsi.vertical_sync_active				= 2;
			params->dsi.vertical_backporch					= 16;
			params->dsi.vertical_frontporch					= 6;
			params->dsi.vertical_active_line				= FRAME_HEIGHT; 

			params->dsi.horizontal_sync_active				= 20;
			params->dsi.horizontal_backporch				= 70;
			params->dsi.horizontal_frontporch				= 100;
			params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		}
		else {
			params->dsi.vertical_sync_active				= 6;
			params->dsi.vertical_backporch					= 10;
			params->dsi.vertical_frontporch					= 12;
			params->dsi.vertical_active_line				= FRAME_HEIGHT; 

			params->dsi.horizontal_sync_active				= 5;
			params->dsi.horizontal_backporch				= 50;
			params->dsi.horizontal_frontporch				= 70;
			params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		}
	    //params->dsi.LPX=8; 

		// Bit rate calculation
		params->dsi.PLL_CLOCK = 240;

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x53;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;

}

#ifdef BUILD_LK
#ifndef CONFIG_FPGA_EARLY_PORTING
#define KTD2151_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t KTD2151_i2c;

static int KTD2151_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    KTD2151_i2c.id = I2C_I2C_LCD_BIAS_CHANNEL;//I2C2;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    KTD2151_i2c.addr = (KTD2151_SLAVE_ADDR_WRITE >> 1);
    KTD2151_i2c.mode = ST_MODE;
    KTD2151_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&KTD2151_i2c, write_data, len);
    //printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

#else
  
//	extern int mt8193_i2c_write(u16 addr, u32 data);
//	extern int mt8193_i2c_read(u16 addr, u32 *data);
	
//	#define TPS65132_write_byte(add, data)  mt8193_i2c_write(add, data)
//	#define TPS65132_read_byte(add)  mt8193_i2c_read(add)
  
#endif
#endif


static void lcm_init_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
	//pmic_set_register_value(MT6351_PMIC_RG_VLDO28_EN_1,1);
	//printk("%s, enable VLDO28\n", __func__);
#else
	//printk("%s, begin\n", __func__);
	//hwPowerOn(MT6351_POWER_LDO_VLDO28, VOL_2800, "LCM_DRV");	
	//printk("%s, end\n", __func__);
#endif
#endif
	printk("%s no call\n", __func__);
}

static void lcm_suspend_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
	//pmic_set_register_value(MT6351_PMIC_RG_VLDO28_EN_1,0);
	//printk("%s, disable VLDO28\n", __func__);
#else
	//printk("%s, begin\n", __func__);
	//hwPowerDown(MT6351_POWER_LDO_VLDO28, "LCM_DRV");	
	//printk("%s, end\n", __func__);
#endif
#endif
	printk("%s no call\n", __func__);
}

static void lcm_resume_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
	//pmic_set_register_value(MT6351_PMIC_RG_VLDO28_EN_1,1);
	//printk("%s, enable VLDO28\n", __func__);
#else
	//printk("%s, begin\n", __func__);
	//hwPowerOn(MT6351_POWER_LDO_VLDO28, VOL_2800, "LCM_DRV");	
	//printk("%s, end\n", __func__);
#endif
#endif
	printk("%s no call\n", __func__);
}


static void lcm_init(void)
{
	int hw_version=0;

	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;

	hw_version = get_cci_hw_id();
	hw_version = (hw_version & 0x07);

#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef CONFIG_MTK_LEGACY
	mt_set_gpio_mode(GPIO_KTD2151_ENP_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_KTD2151_ENP_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_KTD2151_ENP_EN, GPIO_OUT_ONE);
	MDELAY(5);
	mt_set_gpio_mode(GPIO_KTD2151_ENN_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_KTD2151_ENN_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_KTD2151_ENN_EN, GPIO_OUT_ONE);
	MDELAY(5);
#else
	set_gpio_lcd_enp(1);
	MDELAY(5);
	set_gpio_lcd_enn(1);
	MDELAY(5);
#endif
	cmd=0x00;
	data=0x0F; //modify VSP to +5.5V
	//data=0x0A;VSP=5V,//data=0x0E;VSP=5.4V
#ifdef BUILD_LK
	ret=KTD2151_write_byte(cmd,data);
    if(ret)    	
    dprintf(0, "[LK]Gate drive-----ktd2151----cmd=%0x--i2c write error----\n",cmd);    	
	else
	dprintf(0, "[LK]Gate drive----ktd2151----cmd=%0x--i2c write success----\n",cmd);    		
#else
	ret=ktd2151_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]Gate drive-----ktd2151---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]Gate drive-----ktd2151---cmd=%0x-- i2c write success-----\n",cmd);
#endif
	
	cmd=0x01;
	data=0x0F; //modify VSN to -5.5V
	//data=0x0A;VSN=-5V//data=0x0E;VSN=-5.4V
#ifdef BUILD_LK
	ret=KTD2151_write_byte(cmd,data);
    if(ret)    	
	dprintf(0, "[LK]Gate drive-----ktd2151----cmd=%0x--i2c write error----\n",cmd);    	
	else
	dprintf(0, "[LK]Gate drive----ktd2151----cmd=%0x--i2c write success----\n",cmd);   
#else
	ret=ktd2151_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]Gate drive-----ktd2151---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]Gate drive-----ktd2151---cmd=%0x-- i2c write success-----\n",cmd);
#endif
#endif
	lcm_debug("%s %d\n", __func__,__LINE__);
	MDELAY(2);
	//SET_RESET_PIN(1);
    //SET_RESET_PIN(0);
	mt_set_gpio_mode(GPIO_RESET_PIN, 0);
	mt_set_gpio_dir(GPIO_RESET_PIN, 1);
	mt_set_gpio_out(GPIO_RESET_PIN, 1);

	mt_set_gpio_mode(GPIO_RESET_PIN, 0);
	mt_set_gpio_dir(GPIO_RESET_PIN, 1);
	mt_set_gpio_out(GPIO_RESET_PIN, 0);
	MDELAY(1);
    //SET_RESET_PIN(1);
	mt_set_gpio_mode(GPIO_RESET_PIN, 0);
	mt_set_gpio_dir(GPIO_RESET_PIN, 1);
	mt_set_gpio_out(GPIO_RESET_PIN, 1);
	MDELAY(50);

	if(hw_version == 1 || hw_version == 0)
		init_lcm_registers();
	else
		init_lcm_registers_cpt();

	MDELAY(20);
	set_gpio_lcd_bkl(1);

	lcm_debug("%s %d:GPIO_LCD_LED_EN set 1\n", __func__,__LINE__);
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
	
	set_gpio_lcd_bkl(0);
	lcm_debug("[KERNEL]%s %d:GPIO_LCD_LED_EN set 0\n", __func__,__LINE__);
	MDELAY(1);
	data_array[0] = 0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(34);
	set_gpio_lcd_enn(0);
	//SET_RESET_PIN(0);
	MDELAY(1);
	set_gpio_lcd_enp(0);  
	MDELAY(6);
}

static void lcm_resume(void)
{
	lcm_debug("%s %d:[DISP] Truly LCM init sequence:Start\n", __func__,__LINE__);
	lcm_init();
	lcm_debug("%s %d:[DISP] Truly LCM init sequence:End\n", __func__,__LINE__);
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

#ifdef BUILD_LK
static unsigned int lcm_compare_id(void)
{
	#if 0
	char  buffer;
	unsigned int data_array[2];

	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00023902;
	data_array[1]= (0x33<<8)|0xba;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00013700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0xf4, &buffer, 1);

	#ifdef BUILD_LK
		printf("%s, LK debug: hx8394f id = 0x%08x\n", __func__, buffer);
    #else
		printk("%s, kernel debug: hx8394f id = 0x%08x\n", __func__, buffer);
    #endif

	return (buffer == HX8394F_HD_ID ? 1 : 0);
	#endif
	int  lcm_source_pin=2;
	int hw_version=0;
	hw_version = get_cci_hw_id();
	hw_version = (hw_version & 0x07);
	//Modify for different compare mechanism
	if( hw_version < 6 )
	{
		//We use TP ID to identify main or second source LCM module
		mt_set_gpio_mode( GPIO_TP_ID, 0 );
	           mt_set_gpio_dir( GPIO_TP_ID, 0 );
	           lcm_source_pin = mt_get_gpio_in( GPIO_TP_ID );
		printf("%s, [LK]: LCM identify id-TP_ID =%d\n", __func__, lcm_source_pin);
		if(lcm_source_pin == 0) {
			printf("%s, [LK]: It is hx8394f 1st source-Truly\n", __func__);
			return 1;
		}
		else
			return 0;

	}
	else if ( hw_version == 6 )
	{
		//We use LCM ID to identify main or second source LCM module in PQ build
		mt_set_gpio_mode( GPIO_LCM_ID, 0 );
	           mt_set_gpio_dir( GPIO_LCM_ID, 0 );
	           lcm_source_pin = mt_get_gpio_in( GPIO_LCM_ID );
		printf("%s, [LK]: LCM identify id-LCM_ID=%d\n", __func__, lcm_source_pin);
		if(lcm_source_pin == 1) {
			printf("%s, [LK]: It is hx8394f 1st source-Truly\n", __func__);
			return 1;
		}
		else
			return 0;

	}
	else
	{
		printf("%s, [LK]: LCM compare id fail...\n", __func__);
		return 0;
	}
}
#endif


LCM_DRIVER hx8394f_hd_dsi_vdo_truly_lcm_drv = 
{
    .name			= "hx8394f_hd_dsi_vdo_truly",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	#ifdef BUILD_LK
	.compare_id     = lcm_compare_id,
	#endif
     .init_power		= lcm_init_power,
     .resume_power = lcm_resume_power,
     .suspend_power = lcm_suspend_power,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
