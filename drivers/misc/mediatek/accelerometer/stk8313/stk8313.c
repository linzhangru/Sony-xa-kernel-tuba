/* drivers/i2c/chips/stk8313.c - stk8313 motion sensor driver
 *
 *
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
 */

#define SENSOR_STK8313

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
//#include <linux/earlysuspend.h>   //androidM
#include <linux/platform_device.h>
#include <asm/atomic.h>


#include <cust_acc.h>
//#include <linux/hwmsensor.h>//androidM
//#include <linux/hwmsen_dev.h>//androidM
//#include <linux/sensors_io.h> //androidM
#include "stk8313.h"
//#include <linux/hwmsen_helper.h> //androidM

//copy from Grace
#ifdef MT6516
	#include <mach/mt6516_devs.h>
	#include <mach/mt6516_typedefs.h>
	#include <mach/mt6516_gpio.h>
	#include <mach/mt6516_pll.h>
#elif defined MT6573
	#include <mach/mt6573_devs.h>
	#include <mach/mt6573_typedefs.h>
	#include <mach/mt6573_gpio.h>
	#include <mach/mt6573_pll.h>
#elif defined MT6575
	#include <mach/mt6575_devs.h>
	#include <mach/mt6575_typedefs.h>
	#include <mach/mt6575_gpio.h>
	#include <mach/mt6575_pm_ldo.h>
#elif defined MT6577
	#include <mach/mt6577_devs.h>
	#include <mach/mt6577_typedefs.h>
	#include <mach/mt6577_gpio.h>
	#include <mach/mt6577_pm_ldo.h>
#else	
//#if (defined(MT6589) || defined(MT6572) || defined(MT6575))
	//#include <mach/mt_devs.h>
//	#include <mach/mt_typedefs.h>   //androidM
//	#include <mach/mt_gpio.h>   //androidM
//	#include <mach/mt_pm_ldo.h> //androidM
#endif

#ifdef MT6516
	#define POWER_NONE_MACRO MT6516_POWER_NONE
#else
	//#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif
//copy from Grace End


#if 1   //androidM copy from mpu6515.c
#include <accel.h>
#endif

#if 1   // copy from colby
#include <linux/regulator/consumer.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif
#endif

//#define POWER_NONE_MACRO MT65XX_POWER_NONE    //androidM


//
//henry: copy from sensortek
//
#define STK831X_DRIVER_VERSION	"0.9.5"

//#define STK_DEBUG_PRINT
#define STK831X_HOLD_ODR
#define STK831X_INIT_ODR 3	/* 1: 200Hz, 2:100Hz, 3: 50Hz, 4: 25Hz */
#define STK_PERMISSION_THREAD
#define CONFIG_STK831X_LOWPASS
//#define STK_ZG_FILTER
//#define STK_TUNE  //henry mark
#define STK_DEBUG_CALI


/*---------------------  Static Definitions -------------------------*/
#define HH_DEBUG 1   //0:disable, 1:enable
#if(HH_DEBUG)
    #define Printhh(string, args...)    printk("HH(K)=> "string, ##args);
#else
    #define Printhh(string, args...)
#endif

#define HH_TIP 0 //give RD information. Set 1 if develop,and set 0 when release.
#if(HH_TIP)
    #define PrintTip(string, args...)    printk("HH(K)=> "string, ##args);
#else
    #define PrintTip(string, args...)
#endif
/*---------------------  Static Classes  ----------------------------*/
int g_iDelayStk = 500;
int g_iMiscTaXyz[4] = {-1, -1, -1, -1};
int g_iAddSkipCheckId = 1;

static int gsensor_init_flag = -1;	/* 0<==>OK -1 <==> fail */
#if 1   //androidM add
static int gsensor_local_init(void);
static int gsensor_remove(void);

static struct acc_init_info stk8313_init_info = {
	.name = "stk8313",
	.init = gsensor_local_init,
	.uninit = gsensor_remove,
};

    #if 1   //new arch, copy from mpu6515.c
    static DEFINE_MUTEX(gsensor_mutex);
    static bool enable_status;
    #endif
#endif


#if 1	//hh: protect client->addr 
static struct mutex stk8313_i2c_mutex;
#endif

/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_STK8313 345
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
#define CONFIG_STK8313_LOWPASS   /*apply low pass filter on output*/       
/*----------------------------------------------------------------------------*/
#define STK8313_AXIS_X          0
#define STK8313_AXIS_Y          1
#define STK8313_AXIS_Z          2
#define STK8313_AXES_NUM        3
#define STK8313_DATA_LEN        6
#define STK8313_DEV_NAME        "STK8313"
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id stk8313_i2c_id[] = {{STK8313_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_stk8313={ I2C_BOARD_INFO("STK8313", 0x22)};   //androidM
//static struct i2c_board_info __initdata i2c_stk8313={ I2C_BOARD_INFO("STK8313", 0x48)};
/*the adapter id will be available in customization*/
//static unsigned short stk8313_force[] = {0x00, STK8313_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const stk8313_forces[] = { stk8313_force, NULL };
//static struct i2c_client_address_data stk8313_addr_data = { .forces = stk8313_forces,};

/*----------------------------------------------------------------------------*/
static int stk8313_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int stk8313_i2c_remove(struct i2c_client *client);
//static int stk8313_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#if !defined(CONFIG_HAS_EARLYSUSPEND)   

static int stk8313_suspend(struct i2c_client *client, pm_message_t msg) ;
static int stk8313_resume(struct i2c_client *client);
#endif
static int STK831X_SetDelay(struct i2c_client *client, u8 delay);
static int STK831X_SetVD(struct i2c_client *client);

static int STK8313_SetPowerMode(struct i2c_client *client, bool enable);


/*------------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
    ADX_TRC_REGXYZ	= 0X20,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][STK8313_AXES_NUM];
    int sum[STK8313_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct stk8313_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[STK8313_AXES_NUM+1];

    /*data*/
#if 0 // calibration value not set to ic
    s8                      offset[STK8313_AXES_NUM+1];  /*+1: for 4-byte alignment*/
#else
    s16                      offset[STK8313_AXES_NUM+1];  /*+1: for 4-byte alignment*/
#endif
    s16                     data[STK8313_AXES_NUM+1];

#if defined(CONFIG_STK8313_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
    //atomic_t				event_since_en;
};
/*----------------------------------------------------------------------------*/
#define STK8313_MAX_DRIVER_OFFSET	512
#define STK_SAMPLE_NO				10
#define STK_ACC_CALI_VER0			0x3D
#define STK_ACC_CALI_VER1			0x02
#define STK_ACC_CALI_FILE 			"/data/misc/stkacccali.conf"
#define STK_ACC_CALI_FILE_SIZE 		10

#define STK_K_SUCCESS_TUNE			0x04
#define STK_K_SUCCESS_FT2			0x03
#define STK_K_SUCCESS_FT1			0x02
#define STK_K_SUCCESS_FILE			0x01
#define STK_K_NO_CALI				0xFF
#define STK_K_RUNNING				0xFE
#define STK_K_FAIL_LRG_DIFF			0xFD
#define STK_K_FAIL_OPEN_FILE			0xFC
#define STK_K_FAIL_W_FILE				0xFB
#define STK_K_FAIL_R_BACK				0xFA
#define STK_K_FAIL_R_BACK_COMP		0xF9
#define STK_K_FAIL_I2C				0xF8
#define STK_K_FAIL_K_PARA				0xF7
#define STK_K_FAIL_OUT_RG			0xF6
#define STK_K_FAIL_ENG_I2C			0xF5
#define STK_K_FAIL_FT1_USD			0xF4
#define STK_K_FAIL_FT2_USD			0xF3
#define STK_K_FAIL_WRITE_NOFST		0xF2
#define STK_K_FAIL_OTP_5T				0xF1
#define STK_K_FAIL_PLACEMENT			0xF0


#define POSITIVE_Z_UP		0
#define NEGATIVE_Z_UP	1
#define POSITIVE_X_UP		2
#define NEGATIVE_X_UP	3
#define POSITIVE_Y_UP		4
#define NEGATIVE_Y_UP	5
//static unsigned char stk831x_placement = POSITIVE_Z_UP;
#ifdef STK_TUNE
static char stk_tune_offset_record[3] = {0};
static int stk_tune_offset[3] = {0};
static int stk_tune_sum[3] = {0};
static int stk_tune_max[3] = {0};
static int stk_tune_min[3] = {0};
static int stk_tune_index = 0;
static int stk_tune_done = 0;
#endif
//static bool first_enable;
//static atomic_t cali_status;

/*----------------------------------------------------------------------------*/


#if 1   //androidM add
static const struct of_device_id gsensor_of_match2[] = {
	{ .compatible = "mediatek,gsensor"},
	{},
};
#endif

static struct i2c_driver stk8313_i2c_driver = {
    .driver = {
 //       .owner          = THIS_MODULE,
        .name           = STK8313_DEV_NAME,
	.of_match_table = gsensor_of_match2,    //androidM add
    },
	.probe      		= stk8313_i2c_probe,
	.remove    	= stk8313_i2c_remove,
//	.detect		= stk8313_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
//henry: enter here!
    .suspend            = stk8313_suspend,
    .resume             = stk8313_resume,
#endif
	.id_table = stk8313_i2c_id,
	//.address_data = &stk8313_addr_data,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *stk8313_i2c_client = NULL;
//static struct platform_driver stk8313_gsensor_driver; //androidM remove
static struct stk8313_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
static struct GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;
static char selftestRes[10] = {0};


/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
//#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_FUN(f)               printk(KERN_ERR GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
//#define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution stk8313_data_resolution[] = {
 /*8 combination by {FULL_RES,RANGE}*/
    {{ 3, 9}, 256},   /*+/-2g  in 10-bit resolution:  3.9 mg/LSB*/    
    {{ 3, 9}, 256},   /*+/-4g  in 11-bit resolution:  3.9 mg/LSB*/
    {{3, 9},  256},   /*+/-8g  in 12-bit resolution: 3.9 mg/LSB*/
    {{ 7, 8}, 128},   /*+/-16g  in 12-bit resolution:  7.8 mg/LSB (full-resolution)*/       
};
/*----------------------------------------------------------------------------*/
static struct data_resolution stk8313_offset_resolution = {{3, 9}, 256};

/*--------------------ADXL power control function----------------------------------*/

int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, u8 *data)
{
   u8 buf;
    int ret = 0;
	
    //hh: protect client->addr  
    mutex_lock(&stk8313_i2c_mutex);
    //Printhh("[%s] In lock >> \n", __FUNCTION__);

    //client->addr = client->addr & I2C_MASK_FLAG | I2C_WR_FLAG |I2C_RS_FLAG;
    #if 1   //androidM TBD
    client->addr &= I2C_MASK_FLAG;
    client->addr |= I2C_WR_FLAG;
    client->addr |= I2C_RS_FLAG;
    #endif
    buf = addr;
	ret = i2c_master_send(client, (const char*)&buf, 1<<8 | 1);
    //ret = i2c_master_send(client, (const char*)&buf, 1);
    if (ret < 0) {
        HWM_ERR("send command error!!\n");
        mutex_unlock(&stk8313_i2c_mutex);
        //Printhh("[%s] Unlock >> \n", __FUNCTION__);
        return -EFAULT;
    }

    *data = buf;
	client->addr = client->addr& I2C_MASK_FLAG;
    mutex_unlock(&stk8313_i2c_mutex);
    //Printhh("[%s] Unlock >> \n", __FUNCTION__);
    
    return 0;
}

void dumpReg(struct i2c_client *client)
{
  int i=0;
  u8 addr = 0x00;
  u8 regdata=0;
  //for(i=0; i<49 ; i++)
  for(i=0; i<0x40 ; i++)
  {
    //dump all
    hwmsen_read_byte_sr(client,addr,&regdata);
	//HWM_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
        Printhh("[%s] Reg addr=%#x regdata=%#x \n", __FUNCTION__, addr, regdata);

	addr++;
#if 0
	if(addr ==01)
		addr=addr+0x06;
	if(addr==0x09)
		addr++;
	if(addr==0x0A)
		addr++;
#endif
  }
  
  /*
  for(i=0; i<5 ; i++)
  {
    //dump ctrol_reg1~control_reg5
    hwmsen_read_byte_sr(client,addr,regdata);
	HWM_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++;
  }
  
  addr = STK8313_REG_OFSX;
  for(i=0; i<5 ; i++)
  {
    //dump offset
    hwmsen_read_byte_sr(client,addr,regdata);
	HWM_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++
  }
  */
}

int hwmsen_read_block_sr(struct i2c_client *client, u8 addr, u8 *data)
{
   u8 buf[10];
    int ret = 0;
	memset(buf, 0, sizeof(u8)*10); 
	
    //client->addr = client->addr& I2C_MASK_FLAG | I2C_WR_FLAG |I2C_RS_FLAG;
    #if 1   //androidM TBD
    client->addr &= I2C_MASK_FLAG;
    client->addr |= I2C_WR_FLAG;
    client->addr |= I2C_RS_FLAG;
    #endif
    
    buf[0] = addr;
	ret = i2c_master_send(client, (const char*)&buf, 6<<8 | 1);
    //ret = i2c_master_send(client, (const char*)&buf, 1);
    if (ret < 0) {
        HWM_ERR("send command error!!\n");
        return -EFAULT;
    }

    //*data = buf;  //androidM
    data = buf;
	client->addr = client->addr& I2C_MASK_FLAG;
    return 0;
}

static void STK8313_power(struct acc_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//Printhh("[%s] hw->power_id = %d on = %d\n", __FUNCTION__, hw->power_id, on);
	//Printhh("[%s] hw->power_vol = %d\n", __FUNCTION__, hw->power_vol);
	//Printhh("[%s] POWER_NONE_MACRO = %d \n", __FUNCTION__, (int)POWER_NONE_MACRO);

#if 0   //androidM remove
	//if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if( (!(hwPowerOn(hw->power_id, hw->power_vol, "STK8313"))) )
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if ( (!(hwPowerDown(hw->power_id, "STK8313"))) )
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
#endif
	power_on = on;    
}
/*----------------------------------------------------------------------------*/
//this function here use to set resolution and choose sensitivity
static int STK8313_SetDataResolution(struct i2c_client *client ,u8 dataresolution)
{
	int err;
	u8  dat, reso=0;
    //u8 databuf[10];    
    //int res = 0;
	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);
    //int ii = 0;
	
    GSE_LOG("fwq set resolution  dataresolution= %d!\n", dataresolution);
    //choose sensitivity depend on resolution and detect range
	//read detect range
	if( (err = hwmsen_read_byte_sr(client, STK8313_REG_XYZ_DATA_CFG, &dat)) )
	{
		GSE_ERR("read detect range  fail!!\n");
		return err;
	}	
	GSE_LOG("dat=%x!! OK \n",dat);
	dat = (dat&0xc0)>>6;
	GSE_LOG("dat=%x!! OK \n",dat);
	GSE_LOG("reso=%x!! OK \n",reso);
#if 0    
    if(dat & STK8313_RANGE_2G)
    {
      reso = reso + STK8313_RANGE_2G;
    }
	if(dat & STK8313_RANGE_4G)
    {
      reso = reso + STK8313_RANGE_4G;
    }
	if(dat & STK8313_RANGE_8G)
    {
      reso = reso + STK8313_RANGE_8G;
    }
    if(dat & STK8313_RANGE_16G)
    {
      reso = reso + STK8313_RANGE_16G;
    }
#endif
    switch(dat)
    {
        case STK831X_RANGE_2G:
                reso = STK831X_RANGE_2G;
                break;
        case STK831X_RANGE_4G:
                reso = STK831X_RANGE_4G;
                break;
        case STK831X_RANGE_8G:
                reso = STK831X_RANGE_8G;
                break;
        case STK831X_RANGE_16G:
                reso = STK831X_RANGE_16G;
                break;
        default:
                reso = 0;
    } 

    Printhh("[%s] reso=%d \n", __FUNCTION__, reso);
    //ii = (sizeof(stk8313_data_resolution)/sizeof(stk8313_data_resolution[0]));
    //Printhh("[%s] array size =%d \n", __FUNCTION__, ii );
    
	if(reso < (sizeof(stk8313_data_resolution)/sizeof(stk8313_data_resolution[0])) )
	{        
		obj->reso = &stk8313_data_resolution[reso];
		GSE_LOG("reso=%x!! OK \n",reso);
		return 0;
	}
	else
	{   
	    GSE_ERR("choose sensitivity  fail!!\n");
		return -EINVAL;
	}
}
/*----------------------------------------------------------------------------*/
//int g_iX[2] = {0, 0}, g_iY[2] = {0, 0}, g_iZ[2] = {0, 0};

static int STK8313_ReadData(struct i2c_client *client, s16 data[STK8313_AXES_NUM])
{
	struct stk8313_i2c_data *priv = i2c_get_clientdata(client);        
	//u8 addr = STK8313_REG_DATAX0;
	u8 buf[STK8313_DATA_LEN] = {0};
	int err = 0;
#if 1
	int acc_xyz[3] = {0};
	struct stk8313_i2c_data *obj;

        obj = priv;
#endif



	if(NULL == client)
	{
		err = -EINVAL;
	}
	else
		
	{
	  // hwmsen_read_block(client, addr, buf, 0x06);
       //dumpReg(client);
	
	    //hh: protect client->addr  
	    mutex_lock(&stk8313_i2c_mutex);
	    //Printhh("[%s] In lock >> \n", __FUNCTION__);
		buf[0] = STK8313_REG_DATAX0;
	    //client->addr = client->addr& I2C_MASK_FLAG | I2C_WR_FLAG |I2C_RS_FLAG;
            #if 1   //androidM TBD
	    client->addr &= I2C_MASK_FLAG;
	    client->addr |= I2C_WR_FLAG;
	    client->addr |= I2C_RS_FLAG;
            #endif
    
	    i2c_master_send(client, (const char*)&buf, 6<<8 | 1);
	    client->addr = client->addr& I2C_MASK_FLAG;
            mutex_unlock(&stk8313_i2c_mutex);

            //Printhh("[%s] Unlock >> \n", __FUNCTION__);

		data[STK8313_AXIS_X] = (s16)((buf[STK8313_AXIS_X*2] << 8) |
		         (buf[STK8313_AXIS_X*2+1]));
		data[STK8313_AXIS_Y] = (s16)((buf[STK8313_AXIS_Y*2] << 8) |
		         (buf[STK8313_AXIS_Y*2+1]));
		data[STK8313_AXIS_Z] = (s16)((buf[STK8313_AXIS_Z*2] << 8) |
		         (buf[STK8313_AXIS_Z*2+1]));
// henry: this is raw data	    
		//Printhh("[%s] x=%#x  y=%#x  z=%#x \n", __FUNCTION__, data[STK8313_AXIS_X], data[STK8313_AXIS_Y], data[STK8313_AXIS_Z]);

		if(atomic_read(&priv->trace) & ADX_TRC_REGXYZ)
		{
			GSE_LOG("raw from reg(SR) [%08X %08X %08X] => [%5d %5d %5d]\n", data[STK8313_AXIS_X], data[STK8313_AXIS_Y], data[STK8313_AXIS_Z],
		                               data[STK8313_AXIS_X], data[STK8313_AXIS_Y], data[STK8313_AXIS_Z]);
		}
		//GSE_LOG("raw from reg(SR) [%08X %08X %08X] => [%5d %5d %5d]\n", data[MMA8452Q_AXIS_X], data[MMA8452Q_AXIS_Y], data[MMA8452Q_AXIS_Z],
		  //                             data[MMA8452Q_AXIS_X], data[MMA8452Q_AXIS_Y], data[MMA8452Q_AXIS_Z]);
		//add to fix data, refer to datasheet
		
		data[STK8313_AXIS_X] = data[STK8313_AXIS_X]>>4;
		data[STK8313_AXIS_Y] = data[STK8313_AXIS_Y]>>4;
		data[STK8313_AXIS_Z] = data[STK8313_AXIS_Z]>>4;

		//Printhh("[%s] data x=%#x  y=%#x  z=%#x \n", __FUNCTION__, data[STK8313_AXIS_X], data[STK8313_AXIS_Y], data[STK8313_AXIS_Z]);
        
	    //Printhh("[%s] cali_sw x=%#x  y=%#x  z=%#x \n", __FUNCTION__, priv->cali_sw[STK8313_AXIS_X], priv->cali_sw[STK8313_AXIS_Y], priv->cali_sw[STK8313_AXIS_Z]);
		
		data[STK8313_AXIS_X] += priv->cali_sw[STK8313_AXIS_X];
		data[STK8313_AXIS_Y] += priv->cali_sw[STK8313_AXIS_Y];
		data[STK8313_AXIS_Z] += priv->cali_sw[STK8313_AXIS_Z];
		//Printhh("[%s] priv->cali_sw[x]=%#x [y]=%#x [z]=%#x\n", __FUNCTION__, priv->cali_sw[STK8313_AXIS_X], priv->cali_sw[STK8313_AXIS_Y], priv->cali_sw[STK8313_AXIS_Z]);
#if 1 // calibration value not set to ic, new add
		//Printhh("[%s] priv->offset x=%#x  y=%#x  z=%#x \n", __FUNCTION__, priv->offset[STK8313_AXIS_X], priv->offset[STK8313_AXIS_Y], priv->offset[STK8313_AXIS_Z]);
		data[STK8313_AXIS_X] += priv->offset[STK8313_AXIS_X];
		data[STK8313_AXIS_Y] += priv->offset[STK8313_AXIS_Y];
		data[STK8313_AXIS_Z] += priv->offset[STK8313_AXIS_Z];
#endif

#if 1
		acc_xyz[STK8313_AXIS_X] = (int) data[STK8313_AXIS_X];
		acc_xyz[STK8313_AXIS_Y] = (int) data[STK8313_AXIS_Y];
		acc_xyz[STK8313_AXIS_Z] = (int) data[STK8313_AXIS_Z];
		//Printhh("[%s] obj->filter = %d \n", __FUNCTION__, atomic_read(&obj->filter));

		if(atomic_read(&obj->filter))
		{
			//Printhh("[%s] obj->fir_en = %d  obj->suspend = %d\n", __FUNCTION__, atomic_read(&obj->fir_en), atomic_read(&obj->suspend));

			if(atomic_read(&obj->fir_en) && !atomic_read(&obj->suspend))
			{
				int idx, firlen = atomic_read(&obj->firlen);   

				//Printhh("[%s] firlen = %d\n", __FUNCTION__, firlen);
				//Printhh("[%s] obj->fir.num = %d\n", __FUNCTION__, obj->fir.num);
				//Printhh("[%s] obj->fir.idx = %d\n", __FUNCTION__, obj->fir.idx);

				if(obj->fir.num < firlen)
				{                
					obj->fir.raw[obj->fir.num][STK8313_AXIS_X] = (s16) acc_xyz[STK8313_AXIS_X];
					obj->fir.raw[obj->fir.num][STK8313_AXIS_Y] = (s16) acc_xyz[STK8313_AXIS_Y];
					obj->fir.raw[obj->fir.num][STK8313_AXIS_Z] = (s16) acc_xyz[STK8313_AXIS_Z];
					obj->fir.sum[STK8313_AXIS_X] += acc_xyz[STK8313_AXIS_X];
					obj->fir.sum[STK8313_AXIS_Y] += acc_xyz[STK8313_AXIS_Y];
					obj->fir.sum[STK8313_AXIS_Z] += acc_xyz[STK8313_AXIS_Z];
                                        #if 0
					if(atomic_read(&obj->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", obj->fir.num,
						obj->fir.raw[obj->fir.num][STK8313_AXIS_X], obj->fir.raw[obj->fir.num][STK8313_AXIS_Y], obj->fir.raw[obj->fir.num][STK8313_AXIS_Z],
						obj->fir.sum[STK8313_AXIS_X], obj->fir.sum[STK8313_AXIS_Y], obj->fir.sum[STK8313_AXIS_Z]);
					}
                                        #endif
					obj->fir.num++;
					obj->fir.idx++;
				}
				else
				{
					idx = obj->fir.idx % firlen;
					obj->fir.sum[STK8313_AXIS_X] -= obj->fir.raw[idx][STK8313_AXIS_X];
					obj->fir.sum[STK8313_AXIS_Y] -= obj->fir.raw[idx][STK8313_AXIS_Y];
					obj->fir.sum[STK8313_AXIS_Z] -= obj->fir.raw[idx][STK8313_AXIS_Z];
					obj->fir.raw[idx][STK8313_AXIS_X] = (s16) acc_xyz[STK8313_AXIS_X];
					obj->fir.raw[idx][STK8313_AXIS_Y] = (s16) acc_xyz[STK8313_AXIS_Y];
					obj->fir.raw[idx][STK8313_AXIS_Z] = (s16) acc_xyz[STK8313_AXIS_Z];
					obj->fir.sum[STK8313_AXIS_X] += acc_xyz[STK8313_AXIS_X];
					obj->fir.sum[STK8313_AXIS_Y] += acc_xyz[STK8313_AXIS_Y];
					obj->fir.sum[STK8313_AXIS_Z] += acc_xyz[STK8313_AXIS_Z];
					obj->fir.idx++;

					acc_xyz[STK8313_AXIS_X] = obj->fir.sum[STK8313_AXIS_X]/firlen;
					acc_xyz[STK8313_AXIS_Y] = obj->fir.sum[STK8313_AXIS_Y]/firlen;
					acc_xyz[STK8313_AXIS_Z] = obj->fir.sum[STK8313_AXIS_Z]/firlen;
					#if 0
					if(atomic_read(&obj->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						obj->fir.raw[idx][STK8313_AXIS_X], obj->fir.raw[idx][STK8313_AXIS_Y], obj->fir.raw[idx][STK8313_AXIS_Z],
						obj->fir.sum[STK8313_AXIS_X], obj->fir.sum[STK8313_AXIS_Y], obj->fir.sum[STK8313_AXIS_Z],
						acc_xyz[STK8313_AXIS_X], acc_xyz[STK8313_AXIS_Y], acc_xyz[STK8313_AXIS_Z]);
					}
                                        #endif
				}
			}
		}
		data[STK8313_AXIS_X] = (s16) acc_xyz[STK8313_AXIS_X];
		data[STK8313_AXIS_Y] = (s16) acc_xyz[STK8313_AXIS_Y];
		data[STK8313_AXIS_Z] = (s16) acc_xyz[STK8313_AXIS_Z];
#endif

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("raw >>6it:[%08X %08X %08X] => [%5d %5d %5d]\n", data[STK8313_AXIS_X], data[STK8313_AXIS_Y], data[STK8313_AXIS_Z],
		                               data[STK8313_AXIS_X], data[STK8313_AXIS_Y], data[STK8313_AXIS_Z]);
		}
		    
	}
	return err;
}
/*----------------------------------------------------------------------------*/
#if 0 // calibration value not set to ic
static int STK8313_ReadOffset(struct i2c_client *client, s8 ofs[STK8313_AXES_NUM])
{    
	int err;
    GSE_ERR("fwq read offset+: \n");
	if(err = hwmsen_read_byte_sr(client, STK8313_REG_OFSX, &ofs[STK8313_AXIS_X]))
	{
		GSE_ERR("error: %d\n", err);
	}
	if(err = hwmsen_read_byte_sr(client, STK8313_REG_OFSY, &ofs[STK8313_AXIS_Y]))
	{
		GSE_ERR("error: %d\n", err);
	}
	if(err = hwmsen_read_byte_sr(client, STK8313_REG_OFSZ, &ofs[STK8313_AXIS_Z]))
	{
		GSE_ERR("error: %d\n", err);
	}
	Printhh("[%s] read off:  offX=%#x ,offY=%#x ,offZ=%#x \n", __FUNCTION__, ofs[STK8313_AXIS_X],ofs[STK8313_AXIS_Y],ofs[STK8313_AXIS_Z]);

	GSE_LOG("fwq read off:  offX=%x ,offY=%x ,offZ=%x\n",ofs[STK8313_AXIS_X],ofs[STK8313_AXIS_Y],ofs[STK8313_AXIS_Z]);
	
	return err;    
}
#else
static int STK8313_ReadOffset(struct i2c_client *client, s16 ofs[STK8313_AXES_NUM])
{    
	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
    
	GSE_ERR("fwq read offset+: \n");
	ofs[STK8313_AXIS_X] = obj->offset[STK8313_AXIS_X];
	ofs[STK8313_AXIS_Y] = obj->offset[STK8313_AXIS_Y];
	ofs[STK8313_AXIS_Z] = obj->offset[STK8313_AXIS_Z];
	Printhh("[%s] read off:  offX=%#x ,offY=%#x ,offZ=%#x \n", __FUNCTION__, ofs[STK8313_AXIS_X],ofs[STK8313_AXIS_Y],ofs[STK8313_AXIS_Z]);

	GSE_LOG("fwq read off:  offX=%x ,offY=%x ,offZ=%x\n",ofs[STK8313_AXIS_X],ofs[STK8313_AXIS_Y],ofs[STK8313_AXIS_Z]);
	
	return err;    
}
#endif

/*----------------------------------------------------------------------------*/
static int STK8313_ResetCalibration(struct i2c_client *client)
{
	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);
	s8 ofs[STK8313_AXES_NUM] = {0x00, 0x00, 0x00};
	int err = 0;

	Printhh("[%s] org obj->cali_sw[0]=%#x , [1]=%#x , [2]=%#x \n", __FUNCTION__, obj->cali_sw[0], obj->cali_sw[1], obj->cali_sw[2]);
	Printhh("[%s] org obj->offset[0]=%#x , [1]=%#x , [2]=%#x \n", __FUNCTION__, obj->offset[0], obj->offset[1], obj->offset[2]);

#if 0 // calibration value not set to ic
	//goto standby mode to clear cali
	STK8313_SetPowerMode(obj->client,false);
	if(err = hwmsen_write_block(client, STK8313_REG_OFSX, ofs, STK8313_AXES_NUM))
	{
		GSE_ERR("error: %d\n", err);
	}
    STK8313_SetPowerMode(obj->client,true);
#else
	obj->offset[STK8313_AXIS_X] = 0;
	obj->offset[STK8313_AXIS_Y] = 0;
	obj->offset[STK8313_AXIS_Z] = 0;

        #if 1   //fix SP_build calibration_fail issue.
        //We need call STK831X_SetVD();
        //But only call this can not work, so we set (0,0,0) to IC to trigger STK831X_SetVD().
        STK8313_SetPowerMode(obj->client,false);
        if( (err = hwmsen_write_block(obj->client, STK8313_REG_OFSX, ofs, STK8313_AXES_NUM)) )	
        {
            GSE_ERR("error: %d\n", err);
        }
        STK8313_SetPowerMode(obj->client,true);
        #endif    
#endif
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return err;    
}
/*----------------------------------------------------------------------------*/
static int STK8313_ReadCalibration(struct i2c_client *client, int dat[STK8313_AXES_NUM])
{
    struct stk8313_i2c_data *obj = i2c_get_clientdata(client);
    int err;
    int mul;
    
    if ((err = STK8313_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }    
    
    //mul = obj->reso->sensitivity/mma8452q_offset_resolution.sensitivity;
    mul = stk8313_offset_resolution.sensitivity/obj->reso->sensitivity;
    dat[obj->cvt.map[STK8313_AXIS_X]] = obj->cvt.sign[STK8313_AXIS_X]*(obj->offset[STK8313_AXIS_X]/mul);
    dat[obj->cvt.map[STK8313_AXIS_Y]] = obj->cvt.sign[STK8313_AXIS_Y]*(obj->offset[STK8313_AXIS_Y]/mul);
    dat[obj->cvt.map[STK8313_AXIS_Z]] = obj->cvt.sign[STK8313_AXIS_Z]*(obj->offset[STK8313_AXIS_Z]/mul);                        
    Printhh("[%s] read cali  offX=%#x ,offY=%#x ,offZ=%#x \n", __FUNCTION__, obj->offset[STK8313_AXIS_X],obj->offset[STK8313_AXIS_Y],obj->offset[STK8313_AXIS_Z]);
    GSE_LOG("fwq:read cali offX=%x ,offY=%x ,offZ=%x\n",obj->offset[STK8313_AXIS_X],obj->offset[STK8313_AXIS_Y],obj->offset[STK8313_AXIS_Z]);
	//GSE_LOG("fwq:read cali swX=%x ,swY=%x ,swZ=%x\n",obj->cali_sw[MMA8452Q_AXIS_X],obj->cali_sw[MMA8452Q_AXIS_Y],obj->cali_sw[MMA8452Q_AXIS_Z]);
    return 0;
}
/*----------------------------------------------------------------------------*/
static int STK8313_ReadCalibrationEx(struct i2c_client *client, int act[STK8313_AXES_NUM], int raw[STK8313_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;

	if( (err = STK8313_ReadOffset(client, obj->offset)) )
	{
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}    

	mul = stk8313_offset_resolution.sensitivity/obj->reso->sensitivity;
	raw[STK8313_AXIS_X] = obj->offset[STK8313_AXIS_X]/mul + obj->cali_sw[STK8313_AXIS_X];
	raw[STK8313_AXIS_Y] = obj->offset[STK8313_AXIS_Y]/mul + obj->cali_sw[STK8313_AXIS_Y];
	raw[STK8313_AXIS_Z] = obj->offset[STK8313_AXIS_Z]/mul + obj->cali_sw[STK8313_AXIS_Z];

	act[obj->cvt.map[STK8313_AXIS_X]] = obj->cvt.sign[STK8313_AXIS_X]*raw[STK8313_AXIS_X];
	act[obj->cvt.map[STK8313_AXIS_Y]] = obj->cvt.sign[STK8313_AXIS_Y]*raw[STK8313_AXIS_Y];
	act[obj->cvt.map[STK8313_AXIS_Z]] = obj->cvt.sign[STK8313_AXIS_Z]*raw[STK8313_AXIS_Z];                        
	                       
	return 0;
}
/*----------------------------------------------------------------------------*/
static int STK8313_WriteCalibration(struct i2c_client *client, int dat[STK8313_AXES_NUM])
{
	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);
	//u8 testdata=0;
	int err;
	int cali[STK8313_AXES_NUM], raw[STK8313_AXES_NUM];
	int lsb = stk8313_offset_resolution.sensitivity;
	//u8 databuf[2]; 
	//int res = 0;
#if 1 //fix SP_build calibration_fail issue.
	s8 ofs[STK8313_AXES_NUM] = {0x00, 0x00, 0x00};
#endif
	//int divisor = obj->reso->sensitivity/lsb;
	int divisor = lsb/obj->reso->sensitivity;
	GSE_LOG("fwq obj->reso->sensitivity=%d\n", obj->reso->sensitivity);
	GSE_LOG("fwq lsb=%d\n", lsb);
	

//static int STK8313_ReadCalibrationEx(struct i2c_client *client, int act[STK8313_AXES_NUM], int raw[STK8313_AXES_NUM])
	if( (err = STK8313_ReadCalibrationEx(client, cali, raw)) )	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		raw[STK8313_AXIS_X], raw[STK8313_AXIS_Y], raw[STK8313_AXIS_Z],
		obj->offset[STK8313_AXIS_X], obj->offset[STK8313_AXIS_Y], obj->offset[STK8313_AXIS_Z],
		obj->cali_sw[STK8313_AXIS_X], obj->cali_sw[STK8313_AXIS_Y], obj->cali_sw[STK8313_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[STK8313_AXIS_X] += dat[STK8313_AXIS_X];
	cali[STK8313_AXIS_Y] += dat[STK8313_AXIS_Y];
	cali[STK8313_AXIS_Z] += dat[STK8313_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[STK8313_AXIS_X], dat[STK8313_AXIS_Y], dat[STK8313_AXIS_Z]);

#if 0 // calibration value not set to ic
	obj->offset[STK8313_AXIS_X] = (s8)(obj->cvt.sign[STK8313_AXIS_X]*(cali[obj->cvt.map[STK8313_AXIS_X]])*(divisor));
	obj->offset[STK8313_AXIS_Y] = (s8)(obj->cvt.sign[STK8313_AXIS_Y]*(cali[obj->cvt.map[STK8313_AXIS_Y]])*(divisor));
	obj->offset[STK8313_AXIS_Z] = (s8)(obj->cvt.sign[STK8313_AXIS_Z]*(cali[obj->cvt.map[STK8313_AXIS_Z]])*(divisor));
#else
	obj->offset[STK8313_AXIS_X] = (s16)(obj->cvt.sign[STK8313_AXIS_X]*(cali[obj->cvt.map[STK8313_AXIS_X]])*(divisor));
	obj->offset[STK8313_AXIS_Y] = (s16)(obj->cvt.sign[STK8313_AXIS_Y]*(cali[obj->cvt.map[STK8313_AXIS_Y]])*(divisor));
	obj->offset[STK8313_AXIS_Z] = (s16)(obj->cvt.sign[STK8313_AXIS_Z]*(cali[obj->cvt.map[STK8313_AXIS_Z]])*(divisor));
#endif
	/*convert software calibration using standard calibration*/
	obj->cali_sw[STK8313_AXIS_X] =0; //obj->cvt.sign[MMA8452Q_AXIS_X]*(cali[obj->cvt.map[MMA8452Q_AXIS_X]])%(divisor);
	obj->cali_sw[STK8313_AXIS_Y] =0; //obj->cvt.sign[MMA8452Q_AXIS_Y]*(cali[obj->cvt.map[MMA8452Q_AXIS_Y]])%(divisor);
	obj->cali_sw[STK8313_AXIS_Z] =0;// obj->cvt.sign[MMA8452Q_AXIS_Z]*(cali[obj->cvt.map[MMA8452Q_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[STK8313_AXIS_X] + obj->cali_sw[STK8313_AXIS_X], 
		obj->offset[STK8313_AXIS_Y] + obj->cali_sw[STK8313_AXIS_Y], 
		obj->offset[STK8313_AXIS_Z] + obj->cali_sw[STK8313_AXIS_Z], 
		obj->offset[STK8313_AXIS_X], obj->offset[STK8313_AXIS_Y], obj->offset[STK8313_AXIS_Z],
		obj->cali_sw[STK8313_AXIS_X], obj->cali_sw[STK8313_AXIS_Y], obj->cali_sw[STK8313_AXIS_Z]);
#if 0
            printk("HH(K)=>obj->cvt.map[STK8313_AXIS_X]] = %d, obj->cvt.map[STK8313_AXIS_Y]] = %d, obj->cvt.map[STK8313_AXIS_Z]] = %d, divisor = %d\n", 
        obj->cvt.map[STK8313_AXIS_X], obj->cvt.map[STK8313_AXIS_Y], obj->cvt.map[STK8313_AXIS_Z], divisor); 

        printk("HH(K)=> cvt.signX = 0x%x, cvt.signY = 0x%x, cvt.signZ = 0x%x, cvt.mapX = 0x%x, cvt.mapY = 0x%x, cvt.mapZ = 0x%x\n",   
        obj->cvt.sign[STK8313_AXIS_X], obj->cvt.sign[STK8313_AXIS_Y], obj->cvt.sign[STK8313_AXIS_Z], 
        cali[obj->cvt.map[STK8313_AXIS_X]], cali[obj->cvt.map[STK8313_AXIS_Y]], cali[obj->cvt.map[STK8313_AXIS_Z]]);
#endif

#if 1   //fix SP_build calibration_fail issue.
    //We need call STK831X_SetVD();
    //But only call this can not work, so we set (0,0,0) to IC to trigger STK831X_SetVD().
    STK8313_SetPowerMode(obj->client,false);
    if( (err = hwmsen_write_block(obj->client, STK8313_REG_OFSX, ofs, STK8313_AXES_NUM)) )
    {
        GSE_ERR("write offset fail: %d\n", err);
        return err;
    }
    STK8313_SetPowerMode(obj->client,true);
#endif

#if 0 // calibration value not set to ic, remove
	//
	//go to standby mode to set cali
    STK8313_SetPowerMode(obj->client,false);
	if(err = hwmsen_write_block(obj->client, STK8313_REG_OFSX, obj->offset, STK8313_AXES_NUM))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	STK8313_SetPowerMode(obj->client,true);
#endif	
	//
	/*
	STK8313_SetPowerMode(obj->client,false);
	msleep(20);
	if(err = hwmsen_write_byte(obj->client, STK8313_REG_OFSX, obj->offset[STK8313_AXIS_X]))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
    msleep(20);
	hwmsen_read_byte_sr(obj->client,STK8313_REG_OFSX,&testdata);
	GSE_LOG("write offsetX: %x\n", testdata);
	
	if(err = hwmsen_write_byte(obj->client, STK8313_REG_OFSY, obj->offset[STK8313_AXIS_Y]))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	msleep(20);
	hwmsen_read_byte_sr(obj->client,STK8313_REG_OFSY,&testdata);
	GSE_LOG("write offsetY: %x\n", testdata);
	
	if(err = hwmsen_write_byte(obj->client, STK8313_REG_OFSZ, obj->offset[STK8313_AXIS_Z]))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	msleep(20);
	hwmsen_read_byte_sr(obj->client,STK8313_REG_OFSZ,&testdata);
	GSE_LOG("write offsetZ: %x\n", testdata);
	STK8313_SetPowerMode(obj->client,true);
*/
	return err;
}
/*----------------------------------------------------------------------------*/
static int STK8313_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = STK8313_REG_DEVID;    

	res = hwmsen_read_byte_sr(client,STK8313_REG_DEVID,databuf);
	GSE_LOG("fwq stk8313 id %x!\n",databuf[0]);
        Printhh("[%s] fwq stk8313 id %x!\n", __FUNCTION__, databuf[0]);
	
	if(databuf[0]!=STK8313_FIXED_DEVID)
	{
		return STK8313_ERR_IDENTIFICATION;
	}

    	//exit_STK8313_CheckDeviceID:   //androidM
	if (res < 0)
	{
		return STK8313_ERR_I2C;
	}
	
	return STK8313_SUCCESS;
}

static int STK8313_SetReset(struct i2c_client *client)
{
	u8 databuf[2];    
	int res = 0;
	//u8 addr = STK8313_REG_RESET;
	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);
	
	GSE_FUN();
		

	databuf[0] = 0;
	
	
	databuf[1] = databuf[0];
	databuf[0] = STK8313_REG_RESET;
	
        //hh: protect client->addr
        mutex_lock(&stk8313_i2c_mutex);
        Printhh("[%s] after protected,  fwq stk8313 addr %x! \n", __FUNCTION__, client->addr);
        //Printhh("[%s] call i2c_master_send(addr= %#x) \n", __FUNCTION__, client->addr);
	res = i2c_master_send(client, databuf, 0x2);
        mutex_unlock(&stk8313_i2c_mutex);

	if(res <= 0)
	{
		GSE_LOG("fwq set reset failed!\n");
		return STK8313_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		GSE_LOG("fwq set reset ok %d!\n", databuf[1]);
	}

	
	return STK8313_SUCCESS;    
}


//henry add
#if 1
int stk831x_hwmsen_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr; 
	struct i2c_msg msgs[2] = 
	{
		{
			.addr = client->addr,	 
			.flags = 0,
			.len = 1,				 
			.buf= &beg,
			.timing = 400
		},
		{
			.addr = client->addr,	 
			.flags = I2C_M_RD,
			.len = len, 			 
			.buf = data,
			.timing = 400
		}
	};
	int err;

#if 0	//henry add for 400KHz
	.timing = 400
#endif
	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) 
	{		 
		GSE_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) 
	{
		GSE_LOG("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	}
	else 
	{
		err = 0;/*no error*/
	}
	return err;
}


static int STK_i2c_Rx(char *rxData, int length)
{
//	struct i2c_client *client = stk831x_i2c_client; 
	struct i2c_client *client = stk8313_i2c_client; 
	u8 addr = rxData[0];
	int iRet = 0;


        //hh: protect client->addr 
        mutex_lock(&stk8313_i2c_mutex);
	//Printhh("[%s] call i2c_master_send(addr= %#x) \n", __FUNCTION__, client->addr);
	iRet = stk831x_hwmsen_read_block(client, addr, rxData, length);
        mutex_unlock(&stk8313_i2c_mutex);
	
	return iRet;
}

static int STK_i2c_Tx(char *txData, int length)
{
//	struct i2c_client *client = stk831x_i2c_client;
	struct i2c_client *client = stk8313_i2c_client;
	int res = 0;
    
        //hh: protect client->addr 
        mutex_lock(&stk8313_i2c_mutex);
	//Printhh("[%s] call i2c_master_send(addr= %#x) \n", __FUNCTION__, client->addr);
        res = i2c_master_send(client, txData, length);
        mutex_unlock(&stk8313_i2c_mutex);
	
	return res;	
}

#define STK_K_FAIL_OTP_5T				0xF1
#define STK_K_FAIL_ENG_I2C			0xF5

/* [CY33] S- BUG#1402 Grace_Chang Reduce G-sensor resume duration */
#if 0	// no needed
static int STK831x_ReadByteOTP(char rReg, char *value)
{
	int redo = 0;
	int result;
	char buffer[2] = "";
	*value = 0;
	
	buffer[0] = 0x3D;
	buffer[1] = rReg;
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:write 0x3D failed\n", __func__);
		goto eng_i2c_r_err;
	}
	buffer[0] = 0x3F;
	buffer[1] = 0x02;
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:write 0x3F failed\n", __func__);
		goto eng_i2c_r_err;
	}
	
	do {
		msleep(2);
		buffer[0] = 0x3F;
		result = STK_i2c_Rx(buffer, 1);	
		if (result < 0) 
		{
			printk(KERN_ERR "%s:Read 0x3F failed\n", __func__);
			goto eng_i2c_r_err;
		}
		if(buffer[0]& 0x80)
		{
			break;
		}		
		redo++;
	}while(redo < 10);
	
	if(redo == 10)
	{
		printk(KERN_ERR "%s:OTP read repeat read 10 times! Failed!\n", __func__);
		return -STK_K_FAIL_OTP_5T;
	}	
	buffer[0] = 0x3E;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:Read 0x3E failed\n", __func__);
		goto eng_i2c_r_err;
	}	
	*value = buffer[0];
#ifdef STK_DEBUG_CALI		
	GSE_LOG("%s: read 0x%x=0x%x\n", __func__, rReg, *value);
#endif	
	return 0;
	
eng_i2c_r_err:	
	return -STK_K_FAIL_ENG_I2C;
}
#endif
/* [CY33] E- BUG#1402 Grace_Chang Reduce G-sensor resume duration */
/*----------------------------------------------------------------------------*/
static int STK831X_SetPowerModeToWrite(struct i2c_client *client, bool enable)
{
	u8 databuf[2];    
	int res = 0;
	//u8 addr = STK831X_REG_MODE;
	//struct stk831x_i2c_data *obj = i2c_get_clientdata(client);    //henry modify
    	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);

	//int k_status = atomic_read(&cali_status); //androidM
	
	//GSE_FUN();	
	printk("%s: enable=%d", __func__, enable);	
	if(enable == true)
	{
		databuf[1] = STK831X_ACTIVE_MODE;
	}
	else
	{
		databuf[1] = STK831X_STANDBY_MODE;
	}
	
	databuf[0] = STK831X_REG_MODE;

        //hh: protect client->addr  
        mutex_lock(&stk8313_i2c_mutex);
	//Printhh("[%s] call i2c_master_send(addr= %#x) \n", __FUNCTION__, client->addr);
	res = i2c_master_send(client, databuf, 0x2);
        mutex_unlock(&stk8313_i2c_mutex);
	
	if(res <= 0)
	{
		GSE_LOG("fwq set power mode failed!\n");
		return STK831X_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		GSE_LOG("fwq set power mode ok %d!\n", databuf[1]);
	}
	
	if(enable)
	{
		STK831X_SetVD(client);
		//atomic_set(&obj->event_since_en, 0);  //henry mark
#ifdef STK_TUNE		
		if((k_status&0xF0) != 0 && stk_tune_done == 0)
		{
			stk_tune_index = 0;
			STK831x_ResetPara();
		}
#endif
	}
	return STK831X_SUCCESS;    	
}

static int STK831X_SBtoWriteReg(struct i2c_client *client, bool standby)
{
	if(sensor_power)
		STK831X_SetPowerModeToWrite(client, !standby);		
	
	return STK831X_SUCCESS;
}

static int STK831X_SetVD(struct i2c_client *client)
{
	int result;
	char buffer[2] = "";
	//char reg24 = 0;
	//char readvalue = 0;
	/* [CY33] S- BUG#1402 Grace_Chang Reduce G-sensor resume duration */
	char reg24 = 0xcc;
	int i;
	/* [CY33] E- BUG#1402 Grace_Chang Reduce G-sensor resume duration */


	Printhh("[%s] enter (simple init)..\n", __FUNCTION__);
	msleep(2);
	
	/*
	result = STK831x_ReadByteOTP(0x66, &reg24);
	if(result < 0)
	{
		printk(KERN_ERR "%s: read back 0x66 error, result=%d\n", __func__, result);
		return result;
	}
	GSE_LOG("%s:Read 0x66 = 0x%x\n",  __func__, reg24);
	
	if(reg24 != 0)
	{
		buffer[0] = 0x24;
		buffer[1] = reg24;
		GSE_LOG("%s:write 0x%x to 0x24\n",  __func__, buffer[1]);
		result = STK_i2c_Tx(buffer, 2);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:write 0x24 failed\n", __func__);
			return result;
		}
		
		for(i=0;i<6;i++)
		{	
			result = STK831x_ReadByteOTP(OTPReg[i], &readvalue);
			if(result < 0)
			{
				printk(KERN_ERR "%s: read back 0x%x error, result=%d\n", __func__, OTPReg[i], result);
				return result;
			}

			buffer[0] = EngReg[i];
			buffer[1] = readvalue;
			GSE_LOG("%s:write 0x%x to 0x%x\n",  __func__, buffer[1], buffer[0]);
			result = STK_i2c_Tx(buffer, 2);
			if (result < 0) 
			{
				printk(KERN_ERR "%s:write 0x%x failed\n", __func__, buffer[0]);
				return result;
			}
		}
	}
	else
	{
	*/
/* [CY33] S- BUG#1402 Grace_Chang Reduce G-sensor resume duration */
#if 0
		result = STK831x_ReadByteOTP(0x70, &reg24);
		if(result < 0)
		{
			printk(KERN_ERR "%s: read back 0x70 error, result=%d\n", __func__, result);
#ifdef SENSOR_STK8312	
			reg24 = 0xdc;
#elif defined SENSOR_STK8313
			reg24 = 0xcc;
#endif	
		//	return result;
		}
		
		GSE_LOG("%s:Read 0x70 = 0x%x\n",  __func__, reg24);
		if(reg24 != 0)
		{
			buffer[0] = 0x24;
			buffer[1] = reg24;
			GSE_LOG("%s:write 0x%x to 0x24\n",  __func__, buffer[1]);
			result = STK_i2c_Tx(buffer, 2);
			if (result < 0) 
			{
				printk(KERN_ERR "%s:write 0x24 failed\n", __func__);
				return result;
			}
		}	
		else
		{
			GSE_LOG("%s: reg24=0, do nothing\n", __func__);
			return 0;
		}
//	}
#else
	buffer[0] = 0x24;
	buffer[1] = reg24;
	GSE_LOG("%s:write 0x%x to 0x24\n",  __func__, buffer[1]);
	result = STK_i2c_Tx(buffer, 2);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:write 0x24 failed\n", __func__);
		return result;
	}
#endif
/* [CY33] E- BUG#1402 Grace_Chang Reduce G-sensor resume duration */
	buffer[0] = 0x24;
	result = STK_i2c_Rx(buffer, 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:Read 0x24 failed\n", __func__);
		return result;
	}				

	if(buffer[0] != reg24)
	{
		printk(KERN_ERR "%s: error, reg24=0x%x, read=0x%x\n", __func__, reg24, buffer[0]);
/* [CY33] S- BUG#1402 Grace_Chang Reduce G-sensor resume duration */
// Grace , retry to SetVD for avoiding abnormal sensor data
#if 1
		for (i=1; i<=5; i++)
		{
			buffer[0] = 0x24;
			buffer[1] = reg24;
			GSE_LOG("%s: [retry-%d] write 0x%x to 0x24\n",  __func__, i, buffer[1]);
			result = STK_i2c_Tx(buffer, 2);
			if (result < 0)
			{
				printk(KERN_ERR "%s:write 0x24 failed\n", __func__);
				return result;
			}
			buffer[0] = 0x24;
			result = STK_i2c_Rx(buffer, 1);
			if (result < 0)
			{
				printk(KERN_ERR "%s:Read 0x24 failed\n", __func__);
				return result;
			}

			if(buffer[0] == reg24)
			{
				GSE_LOG("%s: [retry-%d] read 0x24 = 0x%x\n", __func__, i, buffer[0]);
				GSE_LOG("%s: [retry-%d] successfully\n",  __func__, i);
				return 0;
			}
		}
		printk(KERN_ERR "%s: [retry 5 times] error, reg24=0x%x, read=0x%x\n", __func__, reg24, buffer[0]);
#endif
/* [CY33] E- BUG#1402 Grace_Chang Reduce G-sensor resume duration */
		return -1;
	}
	GSE_LOG("%s: read 0x24 = 0x%x\n", __func__, buffer[0]);
	GSE_LOG("%s: successfully\n", __func__);
	
	return 0;	
}

static int STK831X_SetDelay(struct i2c_client *client, u8 delay)
{
	u8 databuf[2];    
	int err;
	u8 dat;


	Printhh("[%s] enter..\n", __FUNCTION__);
	
	STK831X_SBtoWriteReg(client, true);
	if((err = hwmsen_read_byte_sr(client, STK831X_REG_SR, &dat)))
		GSE_ERR("%s:error: %d\n", __func__, err);	
		
	databuf[0] = STK831X_REG_SR;
	databuf[1] = (dat & 0xF8) | (delay & 0x07);

        //hh: protect client->addr  
        mutex_lock(&stk8313_i2c_mutex);
        //Printhh("[%s] call i2c_master_send(addr= %#x) \n", __FUNCTION__, client->addr);
	err = i2c_master_send(client, databuf, 0x2);	
        mutex_unlock(&stk8313_i2c_mutex);
        
	if(err <= 0)
	{
		GSE_LOG("%s: failed!\n", __func__);
		return STK831X_ERR_I2C;
	}
	STK831X_SBtoWriteReg(client, false);	
	return STK831X_SUCCESS;	
}

#endif

/*----------------------------------------------------------------------------*/
//normal
//High resolution
//low noise low power
//low power

/*---------------------------------------------------------------------------*/
static int STK8313_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];    
	int res = 0;
	u8 addr = STK8313_REG_MODE;
	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);


	Printhh("[%s] enter..\n", __FUNCTION__);
	
	GSE_FUN();
	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status need not to be set again!!!\n");
		return STK8313_SUCCESS;
	}

	if(hwmsen_read_byte_sr(client, addr, databuf))
	{
		GSE_ERR("read power ctl register err!\n");
		return STK8313_ERR_I2C;
	}

	databuf[0] &= ~STK8313_MEASURE_MODE;
	
	//if(enable == TRUE)
	if(enable == 1)//androidM
	{
		databuf[0] |= STK8313_MEASURE_MODE;
		//atomic_set(&obj->event_since_en, 0);
	}
	else
	{
		// do nothing
	}
	databuf[1] = databuf[0];
	databuf[0] = STK8313_REG_MODE;
	
        //hh: protect client->addr  
        mutex_lock(&stk8313_i2c_mutex);
        //Printhh("[%s] call i2c_master_send(addr= %#x) \n", __FUNCTION__, client->addr);
	res = i2c_master_send(client, databuf, 0x2);
        mutex_unlock(&stk8313_i2c_mutex);	

	if(res <= 0)
	{
		GSE_LOG("fwq set power mode failed!\n");
		return STK8313_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		GSE_LOG("fwq set power mode ok %d!\n", databuf[1]);
	}
    
    Printhh("[%s] enable = %d call STK831X_SetVD()..\n", __FUNCTION__, enable);

//henry add
	if(enable)
	{
		STK831X_SetVD(client);
	}
    
	sensor_power = enable;
	
	return STK8313_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
//set detect range

static int STK8313_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
    
	//struct stk8313_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = STK8313_REG_XYZ_DATA_CFG;    
	databuf[1] = (dataformat<<6)|0x02;

        //hh: protect client->addr  
        mutex_lock(&stk8313_i2c_mutex);
        //Printhh("[%s] call i2c_master_send(addr= %#x) \n", __FUNCTION__, client->addr);
	res = i2c_master_send(client, databuf, 0x2);
        mutex_unlock(&stk8313_i2c_mutex);

	if(res <= 0)
	{
		return STK8313_ERR_I2C;
	}

	return 0;

	//return MMA8452Q_SetDataResolution(obj,dataformat);    
}

#if 0//androidM remove
static int STK8313_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = STK8313_REG_INT;    
	databuf[1] = intenable;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return STK8313_ERR_I2C;
	}
	
	return STK8313_SUCCESS;    
}
#endif

/*----------------------------------------------------------------------------*/
static int STK8313_Init(struct i2c_client *client, int reset_cali)
{
	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
    
	GSE_LOG("2010-11-03-11:43 fwq stk8313 addr %x!\n",client->addr);

	//Printhh("[%s] enter..\n", __FUNCTION__);

	res = STK8313_SetReset(client);
	if(res != STK8313_SUCCESS)
	{
	    GSE_LOG("fwq stk8313 set reset error\n");
		return res;
	}
	res = STK8313_CheckDeviceID(client); 
	if(res != STK8313_SUCCESS)
	{
	    GSE_LOG("fwq stk8313 check id error\n");
            //henry tt: g-sensor not work
            #if 1
            if(g_iAddSkipCheckId ==1)
            {
                Printhh("[%s] skip check device id..\n", __FUNCTION__);
            }
            else{
		return res;
            }
            #endif
            //return res;
	}	

	//Printhh("[%s] STK8313_SetDataFormat(%#x)(0x2:8G, 0x0:2G)..\n", __FUNCTION__, STK8313_RANGE_2G);
	//res = STK8313_SetDataFormat(client, STK8313_RANGE_2G);

	Printhh("[%s] STK8313_SetDataFormat(%#x)(0x2:8G, 0x0:2G)..\n", __FUNCTION__, STK8313_RANGE_8G);
	res = STK8313_SetDataFormat(client, STK8313_RANGE_8G);
    
	if(res != STK8313_SUCCESS)
	{
	    GSE_LOG("fwq stk8313 set data format error\n");
		return res;
	}
	//add by fwq
	//Printhh("[%s] STK8313_SetDataResolution(%#x)(0x2:STK8313_12BIT_RES, 0x0:STK8313_10BIT_RES)..\n", __FUNCTION__, STK8313_10BIT_RES);
	//res = STK8313_SetDataResolution(client, STK8313_10BIT_RES);
    
	Printhh("[%s] STK8313_SetDataResolution(%#x)(0x2:STK8313_12BIT_RES, 0x0:STK8313_10BIT_RES)..\n", __FUNCTION__, STK8313_12BIT_RES);
	res = STK8313_SetDataResolution(client, STK8313_12BIT_RES);
	if(res != STK8313_SUCCESS) 
	{
	    GSE_LOG("fwq stk8313 set data reslution error\n");
		return res;
	}

	//Printhh("[%s] call STK831X_SetDelay()\n", __FUNCTION__);
	//res = STK831X_SetDelay(client, STK831X_INIT_ODR);		
	res = STK831X_SetDelay(client, 2);	//100 HZ
	Printhh("[%s] call STK831X_SetDelay(2:100Hz)\n", __FUNCTION__);
	if(res != STK831X_SUCCESS)
	{
	    GSE_LOG("fwq stk831x set delay error\n");
		return res;		
	}

	res = STK8313_SetPowerMode(client, false);
	if(res != STK8313_SUCCESS)
	{
	    GSE_LOG("fwq stk8313 set power error\n");
		return res;
	}
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;
/*//we do not use interrupt
	res = STK8313_SetIntEnable(client, STK8313_DATA_READY);        
	if(res != STK8313_SUCCESS)//0x2E->0x80
	{
		return res;
	}
*/
    
	//if(NULL != reset_cali)
	if(0 != reset_cali) //androidM
	{ 
		/*reset calibration only in power on*/
		GSE_LOG("fwq stk8313  set cali\n");
		res = STK8313_ResetCalibration(client);
		if(res != STK8313_SUCCESS)
		{
		    GSE_LOG("fwq stk8313 set cali error\n");
			return res;
		}
	}
	//mdelay(g_iDelayStk);
	//Printhh("[%s] stk8313 Init OK..\n", __FUNCTION__);

	//Printhh("[%s] reset obj->fir sizeof()=%d\n", __FUNCTION__, (int)sizeof(obj->fir));
	memset(&obj->fir, 0x00, sizeof(obj->fir));


    GSE_LOG("fwq stk8313 Init OK\n");
	return STK8313_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int STK8313_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "STK8313 Chip");
        dumpReg(client);
    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int STK8313_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct stk8313_i2c_data *obj = (struct stk8313_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[STK8313_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

    //Printhh("[%s] enter..\n", __FUNCTION__);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	//if(sensor_power == FALSE)
	if(sensor_power == false)   //androidM
	{
		res = STK8313_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on stk8313 error %d!\n", res);
		}
	}

	if( (res = STK8313_ReadData(client, obj->data)) )
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		//Printhh("[%s] obj->cali_sw[x]=%#x [y]=%#x [z]=%#x\n", __FUNCTION__, obj->cali_sw[STK8313_AXIS_X], obj->cali_sw[STK8313_AXIS_Y], obj->cali_sw[STK8313_AXIS_Z]);

		obj->data[STK8313_AXIS_X] += obj->cali_sw[STK8313_AXIS_X];
		obj->data[STK8313_AXIS_Y] += obj->cali_sw[STK8313_AXIS_Y];
		obj->data[STK8313_AXIS_Z] += obj->cali_sw[STK8313_AXIS_Z];
		
		/*remap coordinate*/
		acc[obj->cvt.map[STK8313_AXIS_X]] = obj->cvt.sign[STK8313_AXIS_X]*obj->data[STK8313_AXIS_X];
		acc[obj->cvt.map[STK8313_AXIS_Y]] = obj->cvt.sign[STK8313_AXIS_Y]*obj->data[STK8313_AXIS_Y];
		acc[obj->cvt.map[STK8313_AXIS_Z]] = obj->cvt.sign[STK8313_AXIS_Z]*obj->data[STK8313_AXIS_Z];

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[MMA8452Q_AXIS_X], acc[MMA8452Q_AXIS_Y], acc[MMA8452Q_AXIS_Z]);

		//Out put the mg
		acc[STK8313_AXIS_X] = acc[STK8313_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[STK8313_AXIS_Y] = acc[STK8313_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[STK8313_AXIS_Z] = acc[STK8313_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		

		sprintf(buf, "%04x %04x %04x", acc[STK8313_AXIS_X], acc[STK8313_AXIS_Y], acc[STK8313_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
			GSE_LOG("gsensor data:  sensitivity x=%d \n",gsensor_gain.z);
			 
		}
	}
	//Printhh("[%s] OK..\n", __FUNCTION__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int STK8313_ReadRawData(struct i2c_client *client, char *buf)
{
	struct stk8313_i2c_data *obj = (struct stk8313_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	//Printhh("[%s] enter..\n", __FUNCTION__);

	if (!buf || !client)
	{
		return EINVAL;
	}

//	if(sensor_power == FALSE)
	if(sensor_power == false)
	{
		res = STK8313_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on stk8313 error %d!\n", res);
		}
	}
	
	if( (res = STK8313_ReadData(client, obj->data)) )
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "%04x %04x %04x", obj->data[STK8313_AXIS_X], 
			obj->data[STK8313_AXIS_Y], obj->data[STK8313_AXIS_Z]);
	
	}
	GSE_LOG("gsensor data: %s!\n", buf);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int STK8313_InitSelfTest(struct i2c_client *client)
{
	int res = 0;
	//u8  data;
	u8 databuf[10];
    
	GSE_LOG("fwq init self test\n");
/*
	res = STK8313_SetPowerMode(client,true);
	if(res != STK8313_SUCCESS ) //
	{
		return res;
	}
	*/
	
	
	res = STK8313_SetDataFormat(client, STK8313_RANGE_2G);
	if(res != STK8313_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}
	res = STK8313_SetDataResolution(client, STK8313_10BIT_RES);
	if(res != STK8313_SUCCESS) 
	{
	    GSE_LOG("fwq stk8313 set data reslution error\n");
		return res;
	}

	//set self test reg
	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = STK8313_REG_MODE;//set self test    
	if(hwmsen_read_byte_sr(client, STK8313_REG_MODE, databuf))
	{
		GSE_ERR("read power ctl register err!\n");
		return STK8313_ERR_I2C;
	}

	databuf[0] &=~0x02;//clear original    	
	databuf[0] |= 0x02; //set self test
	
	databuf[1]= databuf[0];
	databuf[0]= STK8313_REG_MODE;

	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
	    GSE_LOG("fwq set selftest error\n");
		return STK8313_ERR_I2C;
	}
	
	GSE_LOG("fwq init self test OK\n");
	return STK8313_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int STK8313_JudgeTestResult(struct i2c_client *client, s32 prv[STK8313_AXES_NUM], s32 nxt[STK8313_AXES_NUM])
{
    struct criteria {
        int min;
        int max;
    };
	
    struct criteria self[6][3] = {
        {{-2, 11}, {-2, 16}, {-20, 105}},
        {{-10, 89}, {0, 125}, {0, 819}},
        {{12, 135}, {-135, -12}, {19, 219}},            
        {{ 6,  67}, {-67,  -6},  {10, 110}},
        {{ 6,  67}, {-67,  -6},  {10, 110}},
        {{ 50,  540}, {-540,  -50},  {75, 875}},
    };
    struct criteria (*ptr)[3] = NULL;
    u8 detectRage;
	u8 tmp_resolution;
    int res;
	GSE_LOG("fwq judge test result\n");
    if( (res = hwmsen_read_byte_sr(client, STK8313_REG_XYZ_DATA_CFG, &detectRage)) )
        return res;
	switch((detectRage&0xc0)>>6)
		{
		   case 0x00:
		   	    tmp_resolution = STK8313_10BIT_RES ;
				break;
		   case 0x01:
		   	    tmp_resolution = STK8313_11BIT_RES ;		
				break;
		   case 0x02:
		   	    tmp_resolution = STK8313_12BIT_RES ;		
				break;
		   case 0x03:
		   	    tmp_resolution = STK8313_12BIT_RES ;		
				break;
				default:
					tmp_resolution = STK8313_10BIT_RES ;
					GSE_LOG("fwq judge test detectRage read error !!! \n");
					break;
		}
	GSE_LOG("fwq tmp_resolution=%x , detectRage=%x\n",tmp_resolution,detectRage);
	detectRage = detectRage >>6;
	if((tmp_resolution&STK8313_12BIT_RES) && (detectRage==0x00))
		ptr = &self[0];
	else if((tmp_resolution&STK8313_12BIT_RES) && (detectRage==STK8313_RANGE_4G))
	{
		ptr = &self[1];
		GSE_LOG("fwq self test choose ptr1\n");
	}
	else if((tmp_resolution&STK8313_12BIT_RES) && (detectRage==STK8313_RANGE_8G))
		ptr = &self[2];
	else if(detectRage==STK8313_RANGE_2G)//8 bit resolution
		ptr = &self[3];
	else if(detectRage==STK8313_RANGE_4G)//8 bit resolution
		ptr = &self[4];
	else if(detectRage==STK8313_RANGE_8G)//8 bit resolution
		ptr = &self[5];
	

    if (!ptr) {
        GSE_ERR("null pointer\n");
		GSE_LOG("fwq ptr null\n");
        return -EINVAL;
    }

    if (((nxt[STK8313_AXIS_X] - prv[STK8313_AXIS_X]) > (*ptr)[STK8313_AXIS_X].max) ||
        ((nxt[STK8313_AXIS_X] - prv[STK8313_AXIS_X]) < (*ptr)[STK8313_AXIS_X].min)) {
        GSE_ERR("X is over range\n");
        res = -EINVAL;
    }
    if (((nxt[STK8313_AXIS_Y] - prv[STK8313_AXIS_Y]) > (*ptr)[STK8313_AXIS_Y].max) ||
        ((nxt[STK8313_AXIS_Y] - prv[STK8313_AXIS_Y]) < (*ptr)[STK8313_AXIS_Y].min)) {
        GSE_ERR("Y is over range\n");
        res = -EINVAL;
    }
    if (((nxt[STK8313_AXIS_Z] - prv[STK8313_AXIS_Z]) > (*ptr)[STK8313_AXIS_Z].max) ||
        ((nxt[STK8313_AXIS_Z] - prv[STK8313_AXIS_Z]) < (*ptr)[STK8313_AXIS_Z].min)) {
        GSE_ERR("Z is over range\n");
        res = -EINVAL;
    }
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = stk8313_i2c_client;
	char strbuf[STK8313_BUFSIZE];


        GSE_LOG("fwq show_chipinfo_value \n");
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	STK8313_ReadChipInfo(client, strbuf, STK8313_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = stk8313_i2c_client;
	char strbuf[STK8313_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	STK8313_ReadSensorData(client, strbuf, STK8313_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = stk8313_i2c_client;
	struct stk8313_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[STK8313_AXES_NUM];

        GSE_LOG("fwq show_cali_value \n");
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);


	if( (err = STK8313_ReadOffset(client, obj->offset)) )
	{
		return -EINVAL;
	}
	else if( (err = STK8313_ReadCalibration(client, tmp)) )
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/stk8313_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[STK8313_AXIS_X], obj->offset[STK8313_AXIS_Y], obj->offset[STK8313_AXIS_Z],
			obj->offset[STK8313_AXIS_X], obj->offset[STK8313_AXIS_Y], obj->offset[STK8313_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[STK8313_AXIS_X], obj->cali_sw[STK8313_AXIS_Y], obj->cali_sw[STK8313_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[STK8313_AXIS_X]*mul + obj->cali_sw[STK8313_AXIS_X],
			obj->offset[STK8313_AXIS_Y]*mul + obj->cali_sw[STK8313_AXIS_Y],
			obj->offset[STK8313_AXIS_Z]*mul + obj->cali_sw[STK8313_AXIS_Z],
			tmp[STK8313_AXIS_X], tmp[STK8313_AXIS_Y], tmp[STK8313_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
//static ssize_t store_cali_value(struct device_driver *ddri, char *buf, size_t count)
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)//androidM
{
	struct i2c_client *client = stk8313_i2c_client;  
	int err, x, y, z;
	int dat[STK8313_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if( (err = STK8313_ResetCalibration(client)) )
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[STK8313_AXIS_X] = x;
		dat[STK8313_AXIS_Y] = y;
		dat[STK8313_AXIS_Z] = z;
		if( (err = STK8313_WriteCalibration(client, dat)) )
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}
/*----------------------------------------------------------------------------*/


#if 1
static ssize_t show_cali_miscTa_value(struct device_driver *ddri, char *buf)
{
    int len = 0;


    Printhh("[%s] enter..\n", __FUNCTION__);
    len += snprintf(buf+len, PAGE_SIZE-len, "(X: %#x, Y: %#x, Z: %#x)\n", g_iMiscTaXyz[0], g_iMiscTaXyz[1], g_iMiscTaXyz[2]);

    Printhh("[%s] (X: %#x, Y: %#x, Z: %#x)\n", __FUNCTION__, g_iMiscTaXyz[0], g_iMiscTaXyz[1], g_iMiscTaXyz[2]);

    Printhh("[%s] len = %d\n", __FUNCTION__, len);
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_miscTa_value(struct device_driver *ddri, const char *buf, size_t count)//androidM
{

    int iXyz[4];
    long err = 0;
    int cali[3];
    struct i2c_client *client = stk8313_i2c_client;  
    struct stk8313_i2c_data *obj = i2c_get_clientdata(client);


    sscanf(buf, "%d %d %d\n", &iXyz[0], &iXyz[1], &iXyz[2]);

    Printhh("[%s] iXyz[0] = %#x, iXyz[1] = %#x, iXyz[2] = %#x\n", __FUNCTION__, iXyz[0], iXyz[1], iXyz[2] );
    g_iMiscTaXyz[0] =  iXyz[0];
    g_iMiscTaXyz[1] =  iXyz[1];
    g_iMiscTaXyz[2] =  iXyz[2];

    Printhh("[%s] call STK8313_ResetCalibration() \n", __FUNCTION__);
    err = STK8313_ResetCalibration(client);

    cali[STK8313_AXIS_X] = g_iMiscTaXyz[0] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
    cali[STK8313_AXIS_Y] = g_iMiscTaXyz[1] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
    cali[STK8313_AXIS_Z] = g_iMiscTaXyz[2] * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
    Printhh("[%s] obj->reso->sensitivity = %#x \n", __FUNCTION__, obj->reso->sensitivity);
    Printhh("[%s] cali[x] = %#x cali[y] = %#x cali[z] = %#x\n", __FUNCTION__, cali[STK8313_AXIS_X], cali[STK8313_AXIS_Y], cali[STK8313_AXIS_Z]);
    Printhh("[%s] call STK8313_WriteCalibration() \n", __FUNCTION__);
    err = STK8313_WriteCalibration(client, cali);			 
	
    return count;
}
/*----------------------------------------------------------------------------*/
#endif

#if 1//skip check device id
static ssize_t show_add_skip_checkId(struct device_driver *ddri, char *buf)
{
    int len = 0;


    Printhh("[%s] g_iAddSkipCheckId = %d \n", __FUNCTION__, g_iAddSkipCheckId);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", g_iAddSkipCheckId);
    Printhh("[%s] len = %d\n", __FUNCTION__, len);
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_add_skip_checkId(struct device_driver *ddri, const char *buf, size_t count)//androidM
{

    int64_t iVal = 0;
    int ret = 0;

    ret = kstrtoll(buf, 10, &iVal);
    if (ret != 0) {
        ACC_ERR("invalid format!!\n");
        Printhh("[%s] invalid format!! \n", __FUNCTION__);
        return count;
    }

    g_iAddSkipCheckId = iVal;
    Printhh("[%s] g_iAddSkipCheckId = %d \n", __FUNCTION__, g_iAddSkipCheckId);
    return count;
}
/*----------------------------------------------------------------------------*/
#endif

static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = stk8313_i2c_client;
	//struct stk8313_i2c_data *obj;
	//int result =0;
	int ii = 0;

	Printhh("[%s] enter..\n", __FUNCTION__);
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	for(ii = 0 ; ii < 10; ii++)
		Printhh("[%s] selftestRes[%d] = %#x\n", __FUNCTION__, ii, selftestRes[ii]);

	GSE_LOG("fwq  selftestRes value =%s\n",selftestRes); 
	return snprintf(buf, 10, "%s\n", selftestRes);
}
/*----------------------------------------------------------------------------*/
//static ssize_t store_selftest_value(struct device_driver *ddri, char *buf, size_t count)
static ssize_t store_selftest_value(struct device_driver *ddri, const char *buf, size_t count)//androidM
{   /*write anything to this register will trigger the process*/
	struct item{
	s16 raw[STK8313_AXES_NUM];
	};
	
	struct i2c_client *client = stk8313_i2c_client;  
	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);
	int idx, res, num;
	struct item *prv = NULL, *nxt = NULL;
	s32 avg_prv[STK8313_AXES_NUM] = {0, 0, 0};
	s32 avg_nxt[STK8313_AXES_NUM] = {0, 0, 0};
	//u8 databuf[10];

	Printhh("[%s] enter..\n", __FUNCTION__);

	if(1 != sscanf(buf, "%d", &num))
	{
		GSE_ERR("parse number fail\n");
		return count;
	}
	else if(num == 0)
	{
		GSE_ERR("invalid data count\n");
		return count;
	}

	prv = kzalloc(sizeof(*prv) * num, GFP_KERNEL);
	nxt = kzalloc(sizeof(*nxt) * num, GFP_KERNEL);
	if (!prv || !nxt)
	{
		goto exit;
	}

	res = STK8313_SetPowerMode(client,true);
	if(res != STK8313_SUCCESS ) //
	{
		return res;
	}

	GSE_LOG("NORMAL:\n");
	for(idx = 0; idx < num; idx++)
	{
		if( (res = STK8313_ReadData(client, prv[idx].raw)) )
		{            
			GSE_ERR("read data fail: %d\n", res);
			goto exit;
		}
		
		avg_prv[STK8313_AXIS_X] += prv[idx].raw[STK8313_AXIS_X];
		avg_prv[STK8313_AXIS_Y] += prv[idx].raw[STK8313_AXIS_Y];
		avg_prv[STK8313_AXIS_Z] += prv[idx].raw[STK8313_AXIS_Z];        
		GSE_LOG("[%5d %5d %5d]\n", prv[idx].raw[STK8313_AXIS_X], prv[idx].raw[STK8313_AXIS_Y], prv[idx].raw[STK8313_AXIS_Z]);
	}
	
	avg_prv[STK8313_AXIS_X] /= num;
	avg_prv[STK8313_AXIS_Y] /= num;
	avg_prv[STK8313_AXIS_Z] /= num; 

	res = STK8313_SetPowerMode(client,false);
	if(res != STK8313_SUCCESS ) //
	{
		return res;
	}

	/*initial setting for self test*/
	STK8313_InitSelfTest(client);
	GSE_LOG("SELFTEST:\n");  
/*
	STK8313_ReadData(client, nxt[0].raw);
	GSE_LOG("nxt[0].raw[STK8313_AXIS_X]: %d\n", nxt[0].raw[STK8313_AXIS_X]);
	GSE_LOG("nxt[0].raw[STK8313_AXIS_Y]: %d\n", nxt[0].raw[STK8313_AXIS_Y]);
	GSE_LOG("nxt[0].raw[STK8313_AXIS_Z]: %d\n", nxt[0].raw[STK8313_AXIS_Z]);
	*/
	for(idx = 0; idx < num; idx++)
	{
		if( (res = STK8313_ReadData(client, nxt[idx].raw)) )
		{            
			GSE_ERR("read data fail: %d\n", res);
			goto exit;
		}
		avg_nxt[STK8313_AXIS_X] += nxt[idx].raw[STK8313_AXIS_X];
		avg_nxt[STK8313_AXIS_Y] += nxt[idx].raw[STK8313_AXIS_Y];
		avg_nxt[STK8313_AXIS_Z] += nxt[idx].raw[STK8313_AXIS_Z];        
		GSE_LOG("[%5d %5d %5d]\n", nxt[idx].raw[STK8313_AXIS_X], nxt[idx].raw[STK8313_AXIS_Y], nxt[idx].raw[STK8313_AXIS_Z]);
	}

	//softrestet

	
	// 
	STK8313_Init(client, 0);

	avg_nxt[STK8313_AXIS_X] /= num;
	avg_nxt[STK8313_AXIS_Y] /= num;
	avg_nxt[STK8313_AXIS_Z] /= num;    

	GSE_LOG("X: %5d - %5d = %5d \n", avg_nxt[STK8313_AXIS_X], avg_prv[STK8313_AXIS_X], avg_nxt[STK8313_AXIS_X] - avg_prv[STK8313_AXIS_X]);
	GSE_LOG("Y: %5d - %5d = %5d \n", avg_nxt[STK8313_AXIS_Y], avg_prv[STK8313_AXIS_Y], avg_nxt[STK8313_AXIS_Y] - avg_prv[STK8313_AXIS_Y]);
	GSE_LOG("Z: %5d - %5d = %5d \n", avg_nxt[STK8313_AXIS_Z], avg_prv[STK8313_AXIS_Z], avg_nxt[STK8313_AXIS_Z] - avg_prv[STK8313_AXIS_Z]); 

	if(!STK8313_JudgeTestResult(client, avg_prv, avg_nxt))
	{
		GSE_LOG("SELFTEST : PASS\n");
		atomic_set(&obj->selftest, 1); 
		strcpy(selftestRes,"y");
		
	}	
	else
	{
		GSE_LOG("SELFTEST : FAIL\n");
		atomic_set(&obj->selftest, 0);
		strcpy(selftestRes,"n");
	}

	Printhh("[%s] exit selftestRes[0] = %c\n", __FUNCTION__, selftestRes[0]);

	exit:
	/*restore the setting*/    
	STK8313_Init(client, 0);
	kfree(prv);
	kfree(nxt);
	return count;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
    GSE_LOG("fwq show_firlen_value \n");


	return snprintf(buf, PAGE_SIZE, "not support\n");

}
/*----------------------------------------------------------------------------*/
//static ssize_t store_firlen_value(struct device_driver *ddri, char *buf, size_t count)
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)    //androidM
{
    GSE_LOG("fwq store_firlen_value \n");

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct stk8313_i2c_data *obj = obj_i2c_data;
    
	GSE_LOG("fwq show_trace_value \n");
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
//static ssize_t store_trace_value(struct device_driver *ddri, char *buf, size_t count)
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count) //androidM
{
	struct stk8313_i2c_data *obj = obj_i2c_data;
	int trace;
    
	GSE_LOG("fwq store_trace_value \n");
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
//henry tt
		//GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;    
    struct stk8313_i2c_data *obj = obj_i2c_data;
    u8 databuf[2] = {0};
	
    GSE_LOG("fwq show_status_value \n");
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	//henry: monitor g-sensor invalid data issue.
	hwmsen_read_byte_sr(obj->client, STK8313_REG_MODE, databuf);
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   

		//henry: monitor g-sensor invalid data issue.
		len += scnprintf(buf+len, PAGE_SIZE-len, "power status= %d, power reg= 0x%x\n", sensor_power,databuf[0]);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(selftest,       S_IWUSR | S_IRUGO, show_selftest_value,          store_selftest_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(cali_miscTa,       S_IWUSR | S_IRUGO, show_cali_miscTa_value,          store_cali_miscTa_value);
static DRIVER_ATTR(addskipcheckid,	 S_IWUSR | S_IRUGO, show_add_skip_checkId, store_add_skip_checkId);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *stk8313_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_selftest,         /*self test demo*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,        
	&driver_attr_cali_miscTa,        
	&driver_attr_addskipcheckid,
};
/*----------------------------------------------------------------------------*/
static int stk8313_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(stk8313_attr_list)/sizeof(stk8313_attr_list[0]));

	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if( (err = driver_create_file(driver, stk8313_attr_list[idx])) )
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", stk8313_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int stk8313_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(stk8313_attr_list)/sizeof(stk8313_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, stk8313_attr_list[idx]);
	}
	

	return err;
}

/*----------------------------------------------------------------------------*/

#if 0   //old arch
int gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value = 0;	
	//int sample_delay;	
	struct stk8313_i2c_data *priv = (struct stk8313_i2c_data*)self;
	struct hwm_sensor_data* gsensor_data;
	char buff[STK8313_BUFSIZE];
	
	//GSE_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
#if 1 //henry add
			#ifdef STK831X_HOLD_ODR
			Printhh("[%s] HOLD ODR = %d\n", __FUNCTION__, STK831X_INIT_ODR);
			break;
			#endif
#endif
			//GSE_LOG("fwq set delay\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 5)
				{
					//sample_delay = MMA8452Q_BW_200HZ;
				}
				else if(value <= 10)
				{
					//sample_delay = MMA8452Q_BW_100HZ;
				}
				else
				{
					//sample_delay = MMA8452Q_BW_50HZ;
				}
				
				//err = MMA8452Q_SetBWRate(priv->client, MMA8452Q_BW_100HZ); //err = MMA8452Q_SetBWRate(priv->client, sample_delay);
				if(err != STK8313_SUCCESS ) //0x2C->BW=100Hz
				{
					GSE_ERR("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{					
					
				}
			}
			break;

		case SENSOR_ENABLE:
			Printhh("[%s] SENSOR_ENABLE\n", __FUNCTION__);
			GSE_LOG("fwq sensor enable gsensor\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
					err = STK8313_SetPowerMode( priv->client, !sensor_power);
				}
			}
            		Printhh("[%s] SENSOR_ENABLE = %d \n", __FUNCTION__, value);

			break;

		case SENSOR_GET_DATA:
            		Printhh("[%s] SENSOR_GET_DATA \n", __FUNCTION__);
			//GSE_LOG("fwq sensor operate get data\n");
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (struct hwm_sensor_data *)buff_out;
				STK8313_ReadSensorData(priv->client, buff, STK8313_BUFSIZE);
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
					&gsensor_data->values[1], &gsensor_data->values[2]);				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = 1000;
				//GSE_LOG("X :%d,Y: %d, Z: %d\n",gsensor_data->values[0],gsensor_data->values[1],gsensor_data->values[2]);
				Printhh("[%s] X :%#x,Y: %#x, Z: %#x\n", __FUNCTION__, gsensor_data->values[0],gsensor_data->values[1],gsensor_data->values[2]);
			}
			break;
		default:
            		Printhh("[%s] gsensor operate function no this parameter %#x \n", __FUNCTION__, command);
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

#endif  //old arch

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int stk8313_open(struct inode *inode, struct file *file)
{
	file->private_data = stk8313_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int stk8313_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int stk8313_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
static long stk8313_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct stk8313_i2c_data *obj = (struct stk8313_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[STK8313_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];
        int iRet = 0;


	//Printhh("[%s] enter..cmd=%#x\n", __FUNCTION__, cmd);

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			Printhh("[%s] GSENSOR_IOCTL_INIT..cmd=%#x\n", __FUNCTION__, cmd);

			//GSE_LOG("fwq GSENSOR_IOCTL_INIT\n");
			STK8313_Init(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			Printhh("[%s] GSENSOR_IOCTL_READ_CHIPINFO..cmd=%#x\n", __FUNCTION__, cmd);

			//GSE_LOG("fwq GSENSOR_IOCTL_READ_CHIPINFO\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			STK8313_ReadChipInfo(client, strbuf, STK8313_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			//Printhh("[%s] GSENSOR_IOCTL_READ_SENSORDATA..\n", __FUNCTION__);
			//henry: T2 will call this

			//GSE_LOG("fwq GSENSOR_IOCTL_READ_SENSORDATA\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			#if 0
			if(atomic_read(&obj->event_since_en) < 15){// wait 1000ms, auto-rotation 1000/66 = 15
				atomic_add(1, &obj->event_since_en);	
				return 0;
			}
			#endif

			mutex_lock(&gsensor_mutex);
			//Printhh("[%s] >> enter mutex_lock..\n", __FUNCTION__);
			STK8313_ReadSensorData(client, strbuf, STK8313_BUFSIZE);
			mutex_unlock(&gsensor_mutex);
			//Printhh("[%s] << exit mutex_lock..\n", __FUNCTION__);
			
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			Printhh("[%s] GSENSOR_IOCTL_READ_GAIN..cmd=%#x\n", __FUNCTION__, cmd);

			//GSE_LOG("fwq GSENSOR_IOCTL_READ_GAIN\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_OFFSET:
			Printhh("[%s] GSENSOR_IOCTL_READ_OFFSET..cmd=%#x\n", __FUNCTION__, cmd);

			//GSE_LOG("fwq GSENSOR_IOCTL_READ_OFFSET\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			if(copy_to_user(data, &gsensor_offset, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			Printhh("[%s] GSENSOR_IOCTL_READ_RAW_DATA..cmd=%#x\n", __FUNCTION__, cmd);

			//GSE_LOG("fwq GSENSOR_IOCTL_READ_RAW_DATA\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			//STK8313_ReadRawData(client, &strbuf);
			iRet = STK8313_ReadRawData(client, strbuf);//androidM
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			//Printhh("[%s] GSENSOR_IOCTL_SET_CALI..cmd=%#x\n", __FUNCTION__, cmd);
			Printhh("[%s] GSENSOR_IOCTL_SET_CALI..\n", __FUNCTION__);

			//GSE_LOG("fwq GSENSOR_IOCTL_SET_CALI!!\n");
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{   
			    GSE_LOG("fwq going to set cali\n");
				cali[STK8313_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[STK8313_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[STK8313_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				Printhh("[%s] cali[x] = %#x cali[y] = %#x cali[z] = %#x\n", __FUNCTION__, cali[STK8313_AXIS_X], cali[STK8313_AXIS_Y], cali[STK8313_AXIS_Z]);
				err = STK8313_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			Printhh("[%s] GSENSOR_IOCTL_CLR_CALI..cmd=%#x\n", __FUNCTION__, cmd);

			//GSE_LOG("fwq GSENSOR_IOCTL_CLR_CALI!!\n");
			err = STK8313_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			//GSE_LOG("fwq GSENSOR_IOCTL_GET_CALI\n");
			//Printhh("[%s] GSENSOR_IOCTL_GET_CALI..cmd=%#x\n", __FUNCTION__, cmd);
			Printhh("[%s] GSENSOR_IOCTL_GET_CALI..\n", __FUNCTION__);

			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if( (err = STK8313_ReadCalibration(client, cali)) )
			{
				break;
			}
			Printhh("[%s] cali[x] = %#x cali[y] = %#x cali[z] = %#x\n", __FUNCTION__, cali[STK8313_AXIS_X], cali[STK8313_AXIS_Y], cali[STK8313_AXIS_Z]);
			
			sensor_data.x = cali[STK8313_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[STK8313_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[STK8313_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}

//henry add
#ifdef CONFIG_COMPAT
static long stk831x_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;

	void __user *arg32 = compat_ptr(arg);

	//Printhh("[%s] enter..11\n", __FUNCTION__);

	if(!file->f_op || !file->f_op->unlocked_ioctl)
	{
		return -ENOTTY;
	}

	switch(cmd)
	{
		case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
			//GSE_LOG("fwq GSENSOR_IOCTL_READ_SENSORDATA\n");
			
			if(arg32 == NULL)
			{
				ret = -EINVAL;
				break;	  
			}
			ret = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
			// == stk8313_unlocked_ioctl()
			if(ret)
			{
				GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed!\n");
			}
			break;
#if 1   //henry add for set cali
		case COMPAT_GSENSOR_IOCTL_SET_CALI:
			Printhh("[%s] COMPAT_GSENSOR_IOCTL_SET_CALI \n", __FUNCTION__);
			
			if(arg32 == NULL)
			{
				ret = -EINVAL;
				break;	  
			}
			ret = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
			// == stk8313_unlocked_ioctl()
			if(ret)
			{
				Printhh("[%s] GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed!\n", __FUNCTION__);
			}
			break;

#endif
		default:
            GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
            Printhh("[%s] unknown IOCTL: 0x%08x\n", __FUNCTION__, cmd);

            ret = -ENOIOCTLCMD;
       	 break;
	}
	
	return ret;
}
#endif


/*----------------------------------------------------------------------------*/
static struct file_operations stk8313_fops = {
//	.owner = THIS_MODULE,
	.open = stk8313_open,
	.release = stk8313_release,
	.unlocked_ioctl = stk8313_unlocked_ioctl,

//henry add
#ifdef CONFIG_COMPAT
	.compat_ioctl = stk831x_compat_ioctl,
#endif	
};
/*----------------------------------------------------------------------------*/
static struct miscdevice stk8313_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &stk8313_fops,
};
/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
//henry: enter here!
/*----------------------------------------------------------------------------*/
static int stk8313_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	u8  dat=0;
    
	GSE_FUN();    
        //Printhh("[%s] enter ..\n", __FUNCTION__);
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		Printhh("[%s] msg.event == PM_EVENT_SUSPEND\n", __FUNCTION__);
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			Printhh("[%s] null pointer!!\n", __FUNCTION__);
			return -EINVAL;
		}
		//read old data
		if ((err = hwmsen_read_byte_sr(client, STK8313_REG_MODE, &dat))) 
		{
           GSE_ERR("read ctl_reg1  fail!!\n");
			Printhh("[%s] ead ctl_reg1  fail!!\n", __FUNCTION__);
           return err;
        }
		dat = dat&0b11111110;//stand by mode
		atomic_set(&obj->suspend, 1);
		if( (err = hwmsen_write_byte(client, STK8313_REG_MODE, dat)) )
		{
			GSE_ERR("write power control fail!!\n");
			Printhh("[%s] write power control fail!!\n", __FUNCTION__);
			return err;
		}        
		STK8313_power(obj->hw, 0);
	}
	else{
		Printhh("[%s] msg.event != PM_EVENT_SUSPEND\n", __FUNCTION__);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/
static int stk8313_resume(struct i2c_client *client)
{
	struct stk8313_i2c_data *obj = i2c_get_clientdata(client);        
	int err;

	GSE_FUN();
        //Printhh("[%s] enter ..\n", __FUNCTION__);

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		Printhh("[%s] null pointer!!\n", __FUNCTION__);
		return -EINVAL;
	}

	STK8313_power(obj->hw, 1);
	if( (err = STK8313_Init(client, 0)) )
	{
		GSE_ERR("initialize client fail!!\n");
		Printhh("[%s] initialize client fail!!\n", __FUNCTION__);
		return err;        
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
//henry: not enter here!
/*----------------------------------------------------------------------------*/
static void stk8313_early_suspend(struct early_suspend *h) 
{
	struct stk8313_i2c_data *obj = container_of(h, struct stk8313_i2c_data, early_drv);   
	int err;
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 
	/*
	if(err = hwmsen_write_byte(obj->client, STK8313_REG_POWER_CTL, 0x00))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}  
	*/
	if(err = STK8313_SetPowerMode(obj->client, false))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;
	
	STK8313_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void stk8313_late_resume(struct early_suspend *h)
{
	struct stk8313_i2c_data *obj = container_of(h, struct stk8313_i2c_data, early_drv);         
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	STK8313_power(obj->hw, 1);
	if(err = STK8313_Init(obj->client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return;        
	}
	atomic_set(&obj->suspend, 0);    
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
#if 0   //androidM remove
//static int stk8313_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
static int stk8313_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, STK8313_DEV_NAME);
	return 0;
}
#endif

#if 1   //AndroidM add
/* Maintain  cust info here */
struct acc_hw accel_cust2;
static struct acc_hw *hw = &accel_cust2;

/* For  driver get cust info */
static struct acc_hw *get_cust_acc(void)
{
	return &accel_cust2;
}
#endif


#if 1//new arch
static int gsensor_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */

	//Printhh("[%s] enter..11\n", __FUNCTION__);
	return 0;
}

static int gsensor_enable_nodata(int en)
{
	int err = 0;

	Printhh("[%s] enter..en=%d\n", __FUNCTION__, en);


	//Printhh("[%s] call mutex_lock(gsensor_mutex)\n", __FUNCTION__);
	mutex_lock(&gsensor_mutex);
	if (((en == 0) && (sensor_power == false)) || ((en == 1) && (sensor_power == true))) {
		enable_status = sensor_power;
		//Printhh("[%s] Gsensor device have updated!\n", __FUNCTION__);
		GSE_LOG("Gsensor device have updated!\n");
	} else {
		enable_status = !sensor_power;
		if (atomic_read(&obj_i2c_data->suspend) == 0) {
			err = STK8313_SetPowerMode(obj_i2c_data->client, enable_status);
			GSE_LOG
			    ("Gsensor not in suspend gsensor_SetPowerMode!, enable_status = %d\n",
			     enable_status);
			//Printhh("[%s] Gsensor not in suspend gsensor_SetPowerMode!, enable_status = %d\n", __FUNCTION__, enable_status);

		} else {
			GSE_LOG
			    ("Gsensor in suspend and can not enable or disable!enable_status = %d\n",
			     enable_status);
			//Printhh("[%s] Gsensor in suspend and can not enable or disable!enable_status = %d\n", __FUNCTION__, enable_status);
		}
	}
	mutex_unlock(&gsensor_mutex);
	//Printhh("[%s] call mutex_unlock(gsensor_mutex)\n", __FUNCTION__);

	if (err != STK8313_SUCCESS) {
		GSE_ERR("gsensor_enable_nodata fail!\n");
		Printhh("[%s] gsensor_enable_nodata fail!\n", __FUNCTION__);
		return -1;
	}

	GSE_LOG("gsensor_enable_nodata OK!!!\n");
	//Printhh("[%s] gsensor_enable_nodata OK!!!\n", __FUNCTION__);
	return 0;

}

static int gsensor_set_delay(u64 ns)
{
	int value;
	//int sample_delay;


	Printhh("[%s] enter..ns=%d \n", __FUNCTION__, (int)ns);

	value = (int)ns / 1000 / 1000;

	if (value <= 5){
		//sample_delay = MPU6515_BW_184HZ;
	    }
	else if (value <= 10){
		//sample_delay = MPU6515_BW_92HZ;
	    }
	else{
		//sample_delay = MPU6515_BW_41HZ;
	    }


	//mutex_lock(&gsensor_mutex);
	//err = MPU6515_SetBWRate(obj_i2c_data->client, sample_delay);
	//mutex_unlock(&gsensor_mutex);

#if 0   //henry tt
	if (value >= 50) {
		atomic_set(&obj_i2c_data->filter, 0);
	} else {
	}
#endif

#if 1
	//Printhh("[%s] reset  fir.num idx sum[0-2]\n", __FUNCTION__);
	obj_i2c_data->fir.num = 0;
	obj_i2c_data->fir.idx = 0;
	obj_i2c_data->fir.sum[STK8313_AXIS_X] = 0;
	obj_i2c_data->fir.sum[STK8313_AXIS_Y] = 0;
	obj_i2c_data->fir.sum[STK8313_AXIS_Z] = 0;
	//atomic_set(&obj_i2c_data->filter, 1);
#endif

	GSE_LOG("gsensor_set_delay (%d)\n", value);
	Printhh("[%s] gsensor_set_delay (%d) \n", __FUNCTION__, value);
	return 0;
}


static int gsensor_get_data(int *x, int *y, int *z, int *status)
{
	char buff[STK8313_BUFSIZE];
	//struct stk8313_i2c_data *obj = obj_i2c_data;
	//int ii = 0;


	//Printhh("[%s] enter..11\n", __FUNCTION__);


#if 0
	//ii = atomic_read(&obj->event_since_en);
	if(atomic_read(&obj->event_since_en) < 15){
		//Printhh("[%s] ii = %d...\n", __FUNCTION__, ii);	
		atomic_add(1, &obj->event_since_en);
		//Printhh("[%s] data not ready...\n", __FUNCTION__);	
		return 0;
	}
#endif

	mutex_lock(&gsensor_mutex);
	//Printhh("[%s] >> enter mutex_lock..\n", __FUNCTION__);
	STK8313_ReadSensorData(obj_i2c_data->client, buff, STK8313_BUFSIZE);
	mutex_unlock(&gsensor_mutex);
	//Printhh("[%s] << exit mutex_lock..\n", __FUNCTION__);

	if (sscanf(buff, "%x %x %x", x, y, z) != 3){
		Printhh("[%s] error sscanf\n", __FUNCTION__);
		GSE_ERR("error sscanf\n");
	    }
	//Printhh("[%s] X :%#x,Y: %#x, Z: %#x\n", __FUNCTION__, *x, *y, *z);

	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}
#endif //new arch


/*----------------------------------------------------------------------------*/
static int stk8313_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct stk8313_i2c_data *obj;
#if 0   //old arch  
	struct hwmsen_object sobj;
#endif
	int err = 0;

#if 1   //new arch
	struct acc_control_path ctl = { 0 };
	struct acc_data_path data = { 0 };
#endif
	
	GSE_FUN();
	Printhh("[%s] enter..11\n", __FUNCTION__);
	//mdelay(g_iDelayStk);
	Printhh("[%s] enter..22\n", __FUNCTION__);

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct stk8313_i2c_data));

	//obj->hw = get_cust_acc_hw();  // AndroidM
	obj->hw = get_cust_acc();
	
	if( (err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)) )
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		//goto exit;
		//henry fixed: Coverity CID:89787
		goto exit_kfree;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
#if 1	//henry add for 400KHz
	new_client->timing = 400;
#endif
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

#if 1
	Printhh("[%s] enable firlen(12) fir_en filter\n", __FUNCTION__);
	atomic_set(&obj->firlen, 12);
	//atomic_set(&obj->firlen, 16);
	atomic_set(&obj->fir_en, 1);
	atomic_set(&obj->filter, 1);
#endif

	//atomic_set(&obj->event_since_en, 0);
	stk8313_i2c_client = new_client;	

#if 1//hh: protect client->addr 
	mutex_init(&stk8313_i2c_mutex);
#endif

	if( (err = STK8313_Init(new_client, 1)) )
	{
		goto exit_init_failed;
	}
	

	//Printhh("[%s] call misc_register()\n", __FUNCTION__);
	if( (err = misc_register(&stk8313_device)) )
	{
		GSE_ERR("stk8313_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	//Printhh("[%s] call stk8313_create_attr()\n", __FUNCTION__);
#if 0
	if( (err = stk8313_create_attr(&stk8313_gsensor_driver.driver)) )
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
#else   //androidM add
	err = stk8313_create_attr(&stk8313_init_info.platform_diver_addr->driver);
	if (err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
#endif



#if 1   //new arch
	ctl.open_report_data = gsensor_open_report_data;
	ctl.enable_nodata = gsensor_enable_nodata;
	ctl.set_delay = gsensor_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = false;

	//Printhh("[%s] call acc_register_control_path()\n", __FUNCTION__);
	err = acc_register_control_path(&ctl);
	if (err) {
		GSE_ERR("register acc control path err\n");
		goto exit_create_attr_failed;
	}

	data.get_data = gsensor_get_data;
	data.vender_div = 1000;
	//Printhh("[%s] call acc_register_data_path()\n", __FUNCTION__);
	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path err\n");
		goto exit_create_attr_failed;
	}

	err = batch_register_support_info(ID_ACCELEROMETER, ctl.is_support_batch, 102, 0);/* divisor is 1000/9.8 */
	if (err) {
		GSE_ERR("register gsensor batch support err = %d\n", err);
		goto exit_create_attr_failed;
	}
#endif

#if 0   //old arch
	sobj.self = obj;
    sobj.polling = 1;
    sobj.sensor_operate = gsensor_operate;
	if( (err = hwmsen_attach(ID_ACCELEROMETER, &sobj)) )
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}
#endif  //old arch


#ifdef CONFIG_HAS_EARLYSUSPEND
//henry: not enter here.
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = stk8313_early_suspend,
	obj->early_drv.resume   = stk8313_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 

	Printhh("[%s] OK\n", __FUNCTION__);
	GSE_LOG("%s: OK\n", __func__);
    	gsensor_init_flag = 0;

	return 0;

	exit_create_attr_failed:
	misc_deregister(&stk8313_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
	GSE_ERR("%s: err = %d\n", __func__, err);        
	gsensor_init_flag = -1;
	return err;
}

/*----------------------------------------------------------------------------*/
static int stk8313_i2c_remove(struct i2c_client *client)
{
	int err = 0;	

#if 0	
	if( (err = stk8313_delete_attr(&stk8313_gsensor_driver.driver)) )
	{
		GSE_ERR("stk8313_delete_attr fail: %d\n", err);
	}
#else   //androidM add
	if( (err = stk8313_delete_attr(&stk8313_init_info.platform_diver_addr->driver)) )
	{
		GSE_ERR("stk8313_delete_attr fail: %d\n", err);
	}

#endif
	if( (err = misc_deregister(&stk8313_device)) )
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	if( (err = hwmsen_detach(ID_ACCELEROMETER)) )
	    

	stk8313_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/

#if 0 //androidM remove
static int stk8313_probe(struct platform_device *pdev) 
{
	//struct acc_hw *hw = get_cust_acc_hw();    //androidM
	struct acc_hw *hw = get_cust_acc();
	GSE_FUN();
	Printhh("[%s] enter..11\n", __FUNCTION__);
	mdelay(g_iDelayStk);

	Printhh("[%s] enter..22\n", __FUNCTION__);
        Printhh("[%s] hw->direction = %d\n", __FUNCTION__, hw->direction);

	STK8313_power(hw, 1);
	//stk8313_force[0] = hw->i2c_num;
	if(i2c_add_driver(&stk8313_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk8313_remove(struct platform_device *pdev)
{
    struct acc_hw *hw = get_cust_acc();

    GSE_FUN();    
    STK8313_power(hw, 0);    
    i2c_del_driver(&stk8313_i2c_driver);
    return 0;
}
#endif
/*----------------------------------------------------------------------------*/
#if 1   //androidM remove

    #if 0
#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{ .compatible = "mediatek,gsensor", },
	{},
};
#endif

static struct platform_driver stk8313_gsensor_driver = {
	.probe      = stk8313_probe,
	.remove     = stk8313_remove,    
	.driver     = {
		.name  = "gsensor",
	//	.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = gsensor_of_match,
#endif

	}
};
    #endif

#else

static struct platform_driver stk8313_gsensor_driver = {
	.probe      = stk8313_probe,
	.remove     = stk8313_remove,    
	.driver     = {
		.name  = "gsensor",
	//	.owner = THIS_MODULE,
	}
};
#endif
/*----------------------------------------------------------------------------*/

#if 1   //androidM add
static int gsensor_local_init(void)
{

	Printhh("[%s] enter..\n", __FUNCTION__);
	if (i2c_add_driver(&stk8313_i2c_driver)) {
		Printhh("[%s] add driver error\n", __FUNCTION__);
		GSE_ERR("add driver error\n");
		return -1;
	}
	if (-1 == gsensor_init_flag)
		return -1;

	return 0;
}

static int gsensor_remove(void)
{

	Printhh("[%s] enter..\n", __FUNCTION__);
	i2c_del_driver(&stk8313_i2c_driver);
	return 0;
}
#endif  //androidM add

static int __init stk8313_init(void)
{
	//struct acc_hw *hw = get_cust_acc();

	//const char *name = "mediatek,gsensor";
	const char *name = "mediatek,stk8313";

	hw = get_accel_dts_func(name, hw);


	Printhh("[%s] hw->i2c_num = %#x\n", __FUNCTION__, hw->i2c_num);
	Printhh("[%s] hw->i2c_addr[0] = %#x\n", __FUNCTION__, hw->i2c_addr[0]);
	Printhh("[%s] hw->i2c_addr[1] = %#x\n", __FUNCTION__, hw->i2c_addr[1]);
	Printhh("[%s] hw->direction = %#x\n", __FUNCTION__, hw->direction);
	Printhh("[%s] hw->power_id = %#x\n", __FUNCTION__, hw->power_id);
	Printhh("[%s] hw->power_vol = %#x\n", __FUNCTION__, hw->power_vol);
        Printhh("[%s] hw->direction = %d\n", __FUNCTION__, hw->direction);

        //henry:set i2c 400K at Mt6755.dtsi, it's useless
        Printhh("[%s] i2c speend is 400K\n", __FUNCTION__);

	GSE_FUN();
	GSE_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
#if 0   //androidM remove
	i2c_register_board_info(hw->i2c_num, &i2c_stk8313, 1);
#endif

#if 1 //androidM add
	acc_driver_add(&stk8313_init_info);
#endif

#if 0 //androidM remove
	if(platform_driver_register(&stk8313_gsensor_driver))
	{
		GSE_ERR("failed to register driver");
		return -ENODEV;
	}
#endif
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit stk8313_exit(void)
{
	GSE_FUN();
#if 0   //androidM remove
	platform_driver_unregister(&stk8313_gsensor_driver);
#endif
}
/*----------------------------------------------------------------------------*/
module_init(stk8313_init);
//late_initcall(stk8313_init);
module_exit(stk8313_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("STK8313 I2C driver");
MODULE_AUTHOR("Zhilin.Chen@mediatek.com");
