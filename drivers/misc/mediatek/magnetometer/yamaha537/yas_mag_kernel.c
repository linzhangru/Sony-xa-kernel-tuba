/*
 * Copyright(C)2014 MediaTek Inc.
 * Modification based on code covered by the below mentioned copyright
 * and/or permission notice(S).
 */

/* yas_mag_kernel_driver.c - YAS537 compass driver
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

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#if 0
/* FIXME */
#include "stub.h"
#else
//#include <linux/hwmsen_dev.h>//androidM remove
//#include <linux/sensors_io.h>//androidM remove
#include <linux/proc_fs.h>
//#include <linux/hwmsen_helper.h>//androidM remove
//#include <mach/mt_typedefs.h>//androidM remove
//#include <mach/mt_gpio.h>//androidM remove
//#include <mach/mt_pm_ldo.h>//androidM remove
#include <cust_mag.h>
#include "mag.h"
#endif

#include "yas.h"
//#include <mach/i2c.h>		//for mtk i2c //androidM remove
#include <linux/dma-mapping.h>

#if 1
//jonny gp61 s
#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt_gpio_core.h>
//jonny gp61 e
#endif

#define YAS_MTK_NAME			"yas537"
#define YAS_RAW_NAME			"yas537_raw"
#define YAS_CAL_NAME			"yas537_cal"
#define YAS_EULER_NAME			"yas537_euler"

#define YAS_ADDRESS			(0x2e)

#define CONVERT_M_DIV			(1000)	/* 1/1000 = CONVERT_M */
#define CONVERT_O_DIV			(1000)	/* 1/1000 = CONVERT_O */

//#define POWER_NONE_MACRO		(MT65XX_POWER_NONE) //androidM remove
#define YAS537_I2C_USE_DMA			//use dma transfer //androidM remove

#define ABS_STATUS			(ABS_BRAKE)
#define ABS_DUMMY			(ABS_RUDDER)
#define MIN(a, b)			((a) < (b) ? (a) : (b))


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
int g_iDelay = 500;
static int g_iInitFlag = 0;

#if 1   //AndroidM add
/* Maintain  cust info here */
struct mag_hw mag_cust;
static struct mag_hw *g_pHw = &mag_cust;

/* For  driver get cust info */
static struct mag_hw *get_cust_mag(void)
{
	return &mag_cust;
}
#endif


#define YAS_MTK_DEBUG			(1)
#if YAS_MTK_DEBUG
#define MAGN_TAG		"[Msensor] "
#define MAGN_ERR(fmt, args...)	\
	printk(KERN_ERR  MAGN_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define MAGN_LOG(fmt, args...)	printk(KERN_INFO MAGN_TAG fmt, ##args)
#else
#define MAGN_TAG
#define MAGN_ERR(fmt, args...)	do {} while (0)
#define MAGN_LOG(fmt, args...)	do {} while (0)
#endif


#if 1 //henry merge from MTK new yas537
#define DRIVER_DEBUG	1

#define MSE_TAG                  "MSENSOR"

#if DRIVER_DEBUG
#define MSE_FUN(f)               printk(MSE_TAG" %s\r\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)    printk(KERN_ERR MSE_TAG" %s %d : \r\n"fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)    printk(MSE_TAG fmt, ##args)
#define MSE_VER(fmt, args...)   ((void)0)
#else	
#define MSE_FUN(f)             	
#define MSE_ERR(fmt, args...)
#define MSE_LOG(fmt, args...) 
#define MSE_VER(fmt, args...) 
#endif

#endif//henry merge from MTK new yas537


static struct i2c_client *this_client = NULL;
static struct mutex yas537_i2c_mutex;

#if 1   //uncali
static struct mutex sensor_data_mutex;
#endif

struct yas_state {
	struct mag_hw *hw;
	struct mutex lock;
	struct yas_mag_driver mag;
	struct i2c_client *client;
	struct input_dev *euler;
	struct input_dev *cal;
	struct input_dev *raw;
	struct delayed_work work;
	char * dma_va;
	dma_addr_t dma_pa;
	int32_t mag_delay;
	int32_t euler_delay;
	atomic_t mag_enable;
	atomic_t euler_enable;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend sus;
#endif

#if 1//henry merge from MTK new yas537
	int32_t compass_data[3];
	uint8_t accuracy_m; /*!< Measurement data accuracy */
#endif
};


#if 1
static int g_iHwId = 0;
static int __init get_hwid_from_cmdline(char* cmdline)
{
	if(cmdline == NULL || strlen(cmdline) == 0)
	{
		Printhh("[%s] cci hw  id is empty..\n", __FUNCTION__);
	}
	else
	{
		g_iHwId = simple_strtoul(cmdline, NULL, 16);
		Printhh("[%s] g_iHwId=%#x\n", __FUNCTION__, g_iHwId);
	}

	return 0;
}
__setup("hwid=", get_hwid_from_cmdline);
#endif


static int yas_device_open(int32_t type)
{
        //Printhh("[%s] enter..11\n", __FUNCTION__);
//mdelay(g_iDelay);
        Printhh("[%s] enter..empty func, return 0\n", __FUNCTION__);
	return 0;
}

static int yas_device_close(int32_t type)
{
	return 0;
}

static int yas_device_write(int32_t type, uint8_t addr, const uint8_t *buf,
		int len)
{

	uint8_t tmp[2];
	int ret = 0;
	struct yas_state *st = i2c_get_clientdata(this_client);//henry: disable DMA

        //Printhh("[%s] enter..11\n", __FUNCTION__);
	//mdelay(g_iDelay);
        //Printhh("[%s] enter..22\n", __FUNCTION__);
        //Printhh("[%s] enter..ok\n", __FUNCTION__);

	if (sizeof(tmp) - 1 < len)
		return -1;
	tmp[0] = addr;
	memcpy(&tmp[1], buf, len);
	mutex_lock(&yas537_i2c_mutex);

        //Printhh("[%s] this_client->timing = %d\n", __FUNCTION__, this_client->timing);

	if((len+1) <= 8)
	{
		ret = i2c_master_send(this_client, tmp, len+1);
	}
	else
	{
#if 0
		if(unlikely(NULL == st->dma_va))
		{

			this_client->ext_flag &= I2C_MASK_FLAG; //CLEAR DMA FLAG
			for(i=0; i<=(len+1); i=i+8)
			{
				trans_len = ((i+8)<=(len+1)) ? 8 : (len+1-i);
				MSE_LOG("%s   trans_len = %d\n", __FUNCTION__,trans_len);
				ret = i2c_master_send(this_client, &tmp[i], trans_len);
				if(ret < 0)
					break;
			}
		}
		else
#endif
		{
			this_client->ext_flag = this_client->ext_flag | I2C_DMA_FLAG;	//ENABLE DMA FLAG
			memset(st->dma_va, 0, 1024);
			st->dma_va[0] = addr;
			memcpy(&(st->dma_va[1]), buf, len);
			ret = i2c_master_send(this_client, (char *)(st->dma_pa), len+1);
			if(ret < 0)
			{
				MAGN_ERR("%s i2c_master_send failed! ret = %d\n",__FUNCTION__, ret);
			}
		}
	}

	this_client->ext_flag &= I2C_MASK_FLAG; //CLEAR DMA FLAG
	mutex_unlock(&yas537_i2c_mutex);
	if(ret < 0)
		return ret;
    //MSE_LOG("%s   successful\n", __FUNCTION__);
	return 0;
}

static int yas_device_read(int32_t type, uint8_t addr, uint8_t *buf, int len)
{

	//struct mt_i2c_msg msg[2];//androidM remove
	struct i2c_msg msg[2];
	int err = 0;
	struct yas_state *st= i2c_get_clientdata(this_client);//henry: disable DMA

	//Printhh("[%s] enter..ok\n", __FUNCTION__);

	//memset(msg, 0, sizeof(msg));
	msg[0].addr = this_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;
	msg[0].timing = this_client->timing;	//add for mtk i2c
	msg[0].ext_flag = this_client->ext_flag & I2C_MASK_FLAG;//add for mtk i2c
	msg[1].addr = this_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;
	msg[1].timing = this_client->timing;	//add for mtk i2c
	msg[1].ext_flag = this_client->ext_flag & I2C_MASK_FLAG;//add for mtk i2c


        //Printhh("[%s] this_client->timing = %d\n", __FUNCTION__, this_client->timing);

	if((len > 8 ) && (st->dma_va != NULL))
	{
		msg[1].ext_flag = this_client->ext_flag | I2C_DMA_FLAG;//add for mtk i2c
	}
	mutex_lock(&yas537_i2c_mutex);
//#if 0
	if(len <= 8)
	{
		err = i2c_transfer(this_client->adapter, (struct i2c_msg *)msg, 2);
		if (err != 2) {
			dev_err(&this_client->dev,
					"i2c_transfer() read error: "
					"adapter num = %d,slave_addr=%02x, reg_addr=%02x, err=%d\n",
					this_client->adapter->nr, this_client->addr, addr, err);
			mutex_unlock(&yas537_i2c_mutex);
			return err;
		}
	}

//#else
	else
	{
	#if 0
		if(unlikely(NULL == st->dma_va))
		{
			this_client->ext_flag &= I2C_MASK_FLAG; //CLEAR DMA FLAG	
			memset(buf, 0,len);
			buf = &addr;
			err = i2c_master_send(this_client, buf,1);
			if(err < 0)
			{
				MSE_ERR("%s  i2c_master_send failed err = %d\n", __FUNCTION__, err);
				mutex_unlock(&yas537_i2c_mutex);
				return err;
			}
			
			for(i=0; i<=len; i=i+8)
			{
				trans_len = ((i+8)<=len) ? 8 : (len-i);
				MSE_LOG("%s   trans_len = %d\n", __FUNCTION__,trans_len);

				err = i2c_master_recv(this_client, &buf[i], trans_len);

				if(err < 0)
				{
					MSE_ERR("%s  i2c_master_recv failed err = %d\n", __FUNCTION__, err);
					mutex_unlock(&yas537_i2c_mutex);
					return err;
				}
			}
		}
		else
	#endif
		{
			memset(st->dma_va, 0, 1024);
			msg[1].buf = (char *)(st->dma_pa);
			err = i2c_transfer(this_client->adapter, (struct i2c_msg *)msg, 2);
			if (err != 2) {
				dev_err(&this_client->dev,
						"i2c_transfer() read error: "
						"adapter num = %d,slave_addr=%02x, reg_addr=%02x, err=%d\n",
						this_client->adapter->nr, this_client->addr, addr, err);

				mutex_unlock(&yas537_i2c_mutex);
				return err;
			}
			memcpy(buf, st->dma_va, len);
		}
	}
//#endif
//	MSE_LOG("%s   successful\n", __FUNCTION__);

	mutex_unlock(&yas537_i2c_mutex);
	return 0;
}

static void yas_usleep(int us)
{
	usleep_range(us, us + 1000);
}

static uint32_t yas_current_time(void)
{
	return jiffies_to_msecs(jiffies);
}

static void input_get_data(struct input_dev *input, int32_t *x, int32_t *y,
		int32_t *z, int32_t *status)
{
	*x = input_abs_get_val(input, ABS_X);
	*y = input_abs_get_val(input, ABS_Y);
	*z = input_abs_get_val(input, ABS_Z);
	*status = input_abs_get_val(input, ABS_STATUS);
}

static int start_mag(struct yas_state *st)
{
    Printhh("[%s] enter..\n", __FUNCTION__);
//mdelay(g_iDelay);
	mutex_lock(&st->lock);
	st->mag.set_enable(1);  // == yas_set_enable()?
	mutex_unlock(&st->lock);
	schedule_delayed_work(&st->work, 0);    //== yas_work_func()
	return 0;
}

static int stop_mag(struct yas_state *st)
{
    Printhh("[%s] enter..\n", __FUNCTION__);
//mdelay(g_iDelay);
	cancel_delayed_work_sync(&st->work);    //== yas_work_func()
	mutex_lock(&st->lock);
	st->mag.set_enable(0);
	mutex_unlock(&st->lock);
	return 0;
}

static int set_delay(struct yas_state *st, int delay)
{
	int rt;
    Printhh("[%s] enter..\n", __FUNCTION__);
//mdelay(g_iDelay);
	mutex_lock(&st->lock);
	rt = st->mag.set_delay(delay);
	mutex_unlock(&st->lock);
	MAGN_ERR("XINXIN_set_delay=%d\n",delay);
	return rt;
}

static ssize_t yas_mag_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);

    //Printhh("[%s] enter..\n", __FUNCTION__);

	return sprintf(buf, "%d\n", atomic_read(&st->mag_enable));
}

#if 1//androidM add
static ssize_t yas_mag_enable_show_driver(struct device_driver *ddri, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);

    //Printhh("[%s] enter..\n", __FUNCTION__);

	return sprintf(buf, "%d\n", atomic_read(&st->mag_enable));
}

static void set_mag_enable(struct yas_state *st, int enable);

static ssize_t yas_mag_enable_store_driver(struct device_driver *ddri, const char *buf, size_t count)

{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int enable;
	Printhh("[%s] enter..11\n", __FUNCTION__);
	//mdelay(g_iDelay);
	Printhh("[%s] enter..22\n", __FUNCTION__);
	if (kstrtoint(buf, 10, &enable) < 0)
		return -EINVAL;
	set_mag_enable(st, enable);
	return count;
}

static ssize_t yas_mag_data_show_driver(struct device_driver *ddri, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t xyz[3], status;
    
        Printhh("[%s] enter..\n", __FUNCTION__);
//mdelay(g_iDelay);
	mutex_lock(&st->lock);
	input_get_data(st->cal, &xyz[0], &xyz[1], &xyz[2], &status);
	mutex_unlock(&st->lock);
	return sprintf(buf, "%d %d %d\n", xyz[0], xyz[1], xyz[2]);
}

#if 1   //henry add for same interface 2nd compass
static ssize_t yas_chip_info_show_driver(struct device_driver *ddri, char *buf)
{

        Printhh("[%s] enter..\n", __FUNCTION__);
	return sprintf(buf, "yas537 Chip\n");
}

//copy from yas_mag_self_test_show()
static ssize_t yas_shipmenttest_show_driver(struct device_driver *ddri, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	struct yas537_self_test_result r;
	int ret;
	char result[10];


        Printhh("[%s] enter..\n", __FUNCTION__);
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_SELF_TEST, &r);	// == yas_ext()
	mutex_unlock(&st->lock);
        Printhh("[%s] result:%d icId:%d dir:%d sx:%d sy:%d xyz[0]:%d xyz[1]:%d xyz[2]:%d\n", __FUNCTION__, ret, r.id, r.dir,
			r.sx, r.sy, r.xyz[0], r.xyz[1], r.xyz[2]);

        if(ret == 0){
            strcpy(result, "y");
        }
        else{
            strcpy(result, "n");
        }

        return sprintf(buf, "%s\n", result);

}
#endif   //henry add for same interface 2nd compass

#endif


#if 1   //henry add for cei hwid
static ssize_t hw_info_show_driver(struct device_driver *ddri, char *buf)
{
    
	Printhh("[%s] board_type = %#x\n", __FUNCTION__, g_iHwId);
	return sprintf(buf, "%#x\n", g_iHwId);
}
#endif

static void set_mag_enable(struct yas_state *st, int enable)
{
    Printhh("[%s] enter..enable=%d\n", __FUNCTION__, enable);
//mdelay(g_iDelay);

	if (enable) {
		if (!atomic_cmpxchg(&st->mag_enable, 0, 1)
				&& !atomic_read(&st->euler_enable))
			start_mag(st);
	} else {
		if (atomic_cmpxchg(&st->mag_enable, 1, 0)
				&& !atomic_read(&st->euler_enable))
			stop_mag(st);
	}
}

static ssize_t yas_mag_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int enable;
    Printhh("[%s] enter..11\n", __FUNCTION__);
//mdelay(g_iDelay);
    Printhh("[%s] enter..22\n", __FUNCTION__);
	if (kstrtoint(buf, 10, &enable) < 0)
		return -EINVAL;
	set_mag_enable(st, enable);
	return count;
}

static ssize_t yas_mag_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.get_position();
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d\n", ret);
}

static ssize_t yas_mag_position_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret, position;
	sscanf(buf, "%d\n", &position);
	mutex_lock(&st->lock);
	ret = st->mag.set_position(position);	//==yas_set_position()
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}

static ssize_t yas_mag_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t mag_delay;
	mutex_lock(&st->lock);
	mag_delay = st->mag_delay;
	mutex_unlock(&st->lock);
	return sprintf(buf, "%d\n", mag_delay);
}

static int set_mag_delay(struct yas_state *st, int delay)
{
	Printhh("[%s] enter..delay= %d\n", __FUNCTION__, delay);
//mdelay(g_iDelay);
	if (atomic_read(&st->mag_enable) && atomic_read(&st->euler_enable)) {
		if (set_delay(st, MIN(st->euler_delay, delay)) < 0)
			return -EINVAL;
	} else if (atomic_read(&st->mag_enable)) {
		if (set_delay(st, delay) < 0)
			return -EINVAL;
	}
	st->mag_delay = delay;
	return 0;
}

static ssize_t yas_mag_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int delay;
    Printhh("[%s] enter..\n", __FUNCTION__);
//mdelay(g_iDelay);
	if (kstrtoint(buf, 10, &delay) < 0)
		return -EINVAL;
	if (delay <= 0)
		delay = 0;
	if (set_mag_delay(st, delay) < 0)
		return -EINVAL;
	return count;
}

static ssize_t yas_mag_hard_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int8_t hard_offset[3];
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_GET_HW_OFFSET, hard_offset);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d\n", hard_offset[0], hard_offset[1],
			hard_offset[2]);
}

static ssize_t yas_mag_static_matrix_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int16_t m[9];
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_GET_STATIC_MATRIX, m);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d\n", m[0], m[1], m[2],
			m[3], m[4], m[5], m[6], m[7], m[8]);
}

static ssize_t yas_mag_static_matrix_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int16_t m[9];
	int ret;
	sscanf(buf, "%hd %hd %hd %hd %hd %hd %hd %hd %hd\n", &m[0], &m[1],
			&m[2], &m[3], &m[4], &m[5], &m[6], &m[7], &m[8]);
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_SET_STATIC_MATRIX, m);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}

static ssize_t yas_mag_self_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	struct yas537_self_test_result r;
	int ret;
    Printhh("[%s] enter..\n", __FUNCTION__);
mdelay(g_iDelay);
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_SELF_TEST, &r);	// == yas_ext()
	mutex_unlock(&st->lock);
        Printhh("[%s] result:%d icId:%d dir:%d sx:%d sy:%d xyz[0]:%d xyz[1]:%d xyz[2]:%d\n", __FUNCTION__, ret, r.id, r.dir,
			r.sx, r.sy, r.xyz[0], r.xyz[1], r.xyz[2]);

	return sprintf(buf, "%d %d %d %d %d %d %d %d\n", ret, r.id, r.dir,
			r.sx, r.sy, r.xyz[0], r.xyz[1], r.xyz[2]);
}

static ssize_t yas_mag_self_test_noise_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t xyz_raw[3];
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_SELF_TEST_NOISE, xyz_raw);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d\n", xyz_raw[0], xyz_raw[1], xyz_raw[2]);
}

static ssize_t yas_mag_average_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int8_t mag_average_sample;
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_GET_AVERAGE_SAMPLE, &mag_average_sample);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d\n", mag_average_sample);
}

static ssize_t yas_mag_average_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t tmp;
	int8_t mag_average_sample;
	int ret;
	sscanf(buf, "%d\n", &tmp);
	mag_average_sample = (int8_t)tmp;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_SET_AVERAGE_SAMPLE, &mag_average_sample);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}

static ssize_t yas_mag_ouflow_thresh_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int16_t thresh[6];
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_GET_OUFLOW_THRESH, thresh);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d %d %d %d\n", thresh[0], thresh[1],
			thresh[2], thresh[3], thresh[4], thresh[5]);
}

static ssize_t yas_mag_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t xyz[3], status;
    Printhh("[%s] enter..\n", __FUNCTION__);
//mdelay(g_iDelay);
	mutex_lock(&st->lock);
	input_get_data(st->cal, &xyz[0], &xyz[1], &xyz[2], &status);
	mutex_unlock(&st->lock);
	return sprintf(buf, "%d %d %d\n", xyz[0], xyz[1], xyz[2]);
}

static DEVICE_ATTR(mag_delay, S_IRUGO|S_IWUSR|S_IWGRP,
		yas_mag_delay_show,
		yas_mag_delay_store);
static DEVICE_ATTR(mag_enable, S_IRUGO|S_IWUSR|S_IWGRP, yas_mag_enable_show,
		yas_mag_enable_store);
static DEVICE_ATTR(mag_data, S_IRUGO, yas_mag_data_show, NULL);
static DEVICE_ATTR(mag_position, S_IRUSR|S_IWUSR, yas_mag_position_show,
		yas_mag_position_store);
static DEVICE_ATTR(mag_hard_offset, S_IRUSR, yas_mag_hard_offset_show, NULL);
static DEVICE_ATTR(mag_static_matrix, S_IRUSR|S_IWUSR,
		yas_mag_static_matrix_show, yas_mag_static_matrix_store);
static DEVICE_ATTR(mag_self_test, S_IRUSR, yas_mag_self_test_show, NULL);
static DEVICE_ATTR(mag_self_test_noise, S_IRUSR, yas_mag_self_test_noise_show,
		NULL);
static DEVICE_ATTR(mag_average_sample, S_IRUSR|S_IWUSR,
		yas_mag_average_sample_show, yas_mag_average_sample_store);
static DEVICE_ATTR(mag_ouflow_thresh, S_IRUSR, yas_mag_ouflow_thresh_show, NULL);

static struct attribute *yas_mag_attributes[] = {
	&dev_attr_mag_delay.attr,
	&dev_attr_mag_enable.attr,
	&dev_attr_mag_data.attr,
	&dev_attr_mag_position.attr,
	&dev_attr_mag_hard_offset.attr,
	&dev_attr_mag_static_matrix.attr,
	&dev_attr_mag_self_test.attr,
	&dev_attr_mag_self_test_noise.attr,
	&dev_attr_mag_average_sample.attr,
	&dev_attr_mag_ouflow_thresh.attr,
	NULL
};
static struct attribute_group yas_mag_attribute_group = {
	.attrs = yas_mag_attributes
};

static ssize_t daemon_name_show(struct device_driver *ddri, char *buf)
{
	char strbuf[64];
	sprintf(strbuf, "yamaha537");
	return sprintf(buf, "%s", strbuf);		
}

static DRIVER_ATTR(daemon,      S_IRUGO, daemon_name_show, NULL);
//androidM
static DRIVER_ATTR(magenable,      S_IRUGO|S_IWUSR, yas_mag_enable_show_driver, yas_mag_enable_store_driver);
static DRIVER_ATTR(magsensordata,      S_IRUGO, yas_mag_data_show_driver, NULL);
#if 1   //henry add for same interface 2nd compass
static DRIVER_ATTR(chipinfo,      S_IRUGO, yas_chip_info_show_driver, NULL);
static DRIVER_ATTR(shipmenttest,      S_IRUGO, yas_shipmenttest_show_driver, NULL);
#endif

#if 1   //henry add for cei hw id
static DRIVER_ATTR(hwinfo,      S_IRUGO, hw_info_show_driver, NULL);
#endif


static struct driver_attribute *yas537_attr_list[] = 
{
	&driver_attr_daemon,
	&driver_attr_magenable,
	&driver_attr_magsensordata,
	&driver_attr_chipinfo,
	&driver_attr_shipmenttest,
	&driver_attr_hwinfo,
};

static int yas537_create_attr(struct device_driver *driver)
{
	int ret = 0;
	int i = 0;
	int num = sizeof(yas537_attr_list)/sizeof(yas537_attr_list[0]);

	if(NULL == driver)
	{
		return -EINVAL;
	}
	for(i = 0; i < num; i++)
	{
		ret = driver_create_file(driver, yas537_attr_list[i]);
		if(ret < 0)
		{
			MAGN_ERR("driver_create_file(%s) error=%d!\n",yas537_attr_list[i]->attr.name, ret);
			break;
		}
	}
	
	return ret;
}

#if 0 //androidM remove
static int yas537_delete_attr(struct device_driver *driver)
{
	int ret = 0;
	int i = 0;
	int num = sizeof(yas537_attr_list)/sizeof(yas537_attr_list[0]);

	if(NULL == driver)
	{
		return -EINVAL;
	}
	for(i = 0; i < num; i++)
	{
		driver_remove_file(driver, yas537_attr_list[i]);
	}
	
	return ret;

}
#endif

static ssize_t yas_euler_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	return sprintf(buf, "%d\n", atomic_read(&st->euler_enable));
}

static void set_euler_enable(struct yas_state *st, int enable)
{

	Printhh("[%s] enter..enable=%d \n", __FUNCTION__, enable);
	//Printhh("[%s] enter..st->euler_enable=%d \n", __FUNCTION__,  (int)st->euler_enable);
	if (enable) {
		if (!atomic_cmpxchg(&st->euler_enable, 0, 1)
				&& !atomic_read(&st->mag_enable))
			start_mag(st);
	} else {
		if (atomic_cmpxchg(&st->euler_enable, 1, 0)
				&& !atomic_read(&st->mag_enable))
			stop_mag(st);
	}
}

static ssize_t yas_euler_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int enable;
	if (kstrtoint(buf, 10, &enable) < 0)
		return -EINVAL;
	set_euler_enable(st, enable);
	return count;
}

static ssize_t yas_euler_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t euler_delay;
	mutex_lock(&st->lock);
	euler_delay = st->euler_delay;
	mutex_unlock(&st->lock);
	return sprintf(buf, "%d\n", euler_delay);
}

static int set_euler_delay(struct yas_state *st, int delay)
{
	if (atomic_read(&st->mag_enable) && atomic_read(&st->euler_enable)) {
		if (set_delay(st, MIN(st->mag_delay, delay)) < 0)
			return -EINVAL;
	} else if (atomic_read(&st->euler_enable)) {
		if (set_delay(st, delay) < 0)
			return -EINVAL;
	}
	st->euler_delay = delay;
	return 0;
}

static ssize_t yas_euler_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int delay;
	if (kstrtoint(buf, 10, &delay) < 0)
		return -EINVAL;
	if (delay <= 0)
		delay = 0;
	set_euler_delay(st, delay);
	return count;
}

static ssize_t yas_euler_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t xyz[3], status;
	mutex_lock(&st->lock);
	input_get_data(st->euler, &xyz[0], &xyz[1], &xyz[2], &status);
	mutex_unlock(&st->lock);
	return sprintf(buf, "%d %d %d\n", xyz[0], xyz[1], xyz[2]);
}

static DEVICE_ATTR(euler_delay, S_IRUGO|S_IWUSR|S_IWGRP,
		yas_euler_delay_show, yas_euler_delay_store);
static DEVICE_ATTR(euler_enable, S_IRUGO|S_IWUSR|S_IWGRP, yas_euler_enable_show,
		yas_euler_enable_store);
static DEVICE_ATTR(euler_data, S_IRUGO, yas_euler_data_show, NULL);

static struct attribute *yas_euler_attributes[] = {
	&dev_attr_euler_delay.attr,
	&dev_attr_euler_enable.attr,
	&dev_attr_euler_data.attr,
	NULL
};
static struct attribute_group yas_euler_attribute_group = {
	.attrs = yas_euler_attributes
};

static void yas_work_func(struct work_struct *work)
{
	static int32_t dummy_count;
	struct yas_state *st
		= container_of((struct delayed_work *)work,
			struct yas_state, work);
	struct yas_data mag[1];
	int32_t delay;
	uint32_t time_before, time_after;
	int ret;
	int ii;

    //Printhh("[%s] enter..\n", __FUNCTION__);

	time_before = yas_current_time();
	mutex_lock(&st->lock);
	if (atomic_read(&st->mag_enable) && atomic_read(&st->euler_enable))
		delay = MIN(st->euler_delay, st->mag_delay);
	else if (atomic_read(&st->mag_enable))
		delay = st->mag_delay;
	else if (atomic_read(&st->euler_enable))
		delay = st->euler_delay;
	else
		delay = MIN(st->euler_delay, st->mag_delay);
	ret = st->mag.measure(mag, 1);  // == yas_measure_wrap() == yas_device_read()
#if 1   //henry merge from MTK new yas537
	if (ret == 1) {
		for (ii = 0; ii < 3; ii++)
			st->compass_data[ii] = mag[0].xyz.v[ii];
		st->accuracy_m = mag[0].accuracy;
	}
#endif
	mutex_unlock(&st->lock);
	if (ret == 1) {
		/* report magnetic data in [nT] */
		input_report_abs(st->raw, ABS_X, mag[0].xyz.v[0]);
		input_report_abs(st->raw, ABS_Y, mag[0].xyz.v[1]);
		input_report_abs(st->raw, ABS_Z, mag[0].xyz.v[2]);
		input_report_abs(st->raw, ABS_DUMMY, ++dummy_count);
		input_sync(st->raw);
	}
	time_after = yas_current_time();
	delay = delay - (time_after - time_before);
	if (delay <= 0)
		delay = 1;
    	//Printhh("[%s] delay=%d\n", __FUNCTION__, delay);

	schedule_delayed_work(&st->work, msecs_to_jiffies(delay));//== yas_work_func()
}

static int yas_m_enable(int en)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	Printhh("[%s] enter..\n", __FUNCTION__);
	//mdelay(g_iDelay);
	//Printhh("[%s] enter..22\n", __FUNCTION__);
	set_mag_enable(st, en);
	return 0;
}

static int yas_m_set_delay(u64 ns)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	//MAGN_ERR("XINXIN0_set_delay=%d\n",ns);
    Printhh("[%s] enter..\n", __FUNCTION__);
//mdelay(g_iDelay);
	return set_mag_delay(st, (int32_t)ns /1000000);
}


#if 1 //henry merge from MTK new yas537

static int yas537_open(struct inode *inode, struct file *file)
{    
	int ret = 0;
	
	ret = nonseekable_open(inode, file);
	
	return ret;
}
/*----------------------------------------------------------------------------*/
static int yas537_release(struct inode *inode, struct file *file)
{	
	return 0;
}

/*----------------------------------------------------------------------------*/

static long yas537_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)

{
	//int valuebuf[4];
	//int calidata[7];
	//int controlbuf[10];
	char strbuf[64];
	void __user *data;
	long retval=0;
	long len = 0;
	unsigned int enable = 0;
	//int mode=0;
	struct yas_state *st = NULL;
	
	MSE_FUN();

	//Printhh("[%s] enter..\n", __FUNCTION__);

	if(NULL == this_client)
	{
		MSE_ERR("this_client is NULL!\n");
		return -1;
	}
	
	st = i2c_get_clientdata(this_client);
	
	if(NULL == st)
	{
		MSE_ERR("struct yas_state *st is NULL!\n");
		return -1;
	}


	switch (cmd)
	{
		case MSENSOR_IOCTL_INIT:
			Printhh("[%s] MSENSOR_IOCTL_INIT.\n", __FUNCTION__);
			data = (void __user *) arg;
			//retval = ioctl_init(data);         
			break;

		case MSENSOR_IOCTL_SET_POSTURE:			
			break;        

		case MSENSOR_IOCTL_SET_CALIDATA:			
			break;                                

		case MSENSOR_IOCTL_READ_CHIPINFO:
			Printhh("[%s] MSENSOR_IOCTL_READ_CHIPINFO.\n", __FUNCTION__);

			data = (void __user *) arg;
			if(data == NULL)
			{
				MSE_ERR("IO parameter pointer is NULL!\r\n");
				retval = -EFAULT;
				break;
			}		
			
			if(copy_to_user(data, "yas537", sizeof("yas537")))
			{
				retval = -EFAULT;
				goto err_out;
			}                
			break;

		case MSENSOR_IOCTL_READ_SENSORDATA:	//read msensor data
			//Printhh("[%s] MSENSOR_IOCTL_READ_SENSORDATA.\n", __FUNCTION__);
			data = (void __user *) arg;
			if(data == NULL)
			{
				MSE_ERR("IO parameter pointer is NULL!\r\n");
				retval = -EFAULT;
				break;
			}			
			memset(strbuf, 0, sizeof(strbuf));
			
			len = sprintf(strbuf, "%x %x %x", st->compass_data[0]/1000, st->compass_data[1]/1000, st->compass_data[2]/1000);

			retval = copy_to_user(data, strbuf, len);
			if(retval < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_READ_SENSORDATA ERROR: %ld\n", retval);
			}
			break;                

		case MSENSOR_IOCTL_READ_POSTUREDATA:			           
			break;            

		case MSENSOR_IOCTL_READ_CALIDATA:       
			break;

		case MSENSOR_IOCTL_READ_CONTROL:			                           
			break;

		case MSENSOR_IOCTL_SET_CONTROL:			  
			break;

		case MSENSOR_IOCTL_SET_MODE:			                
			break;

		case MSENSOR_IOCTL_SENSOR_ENABLE:
			Printhh("[%s] MSENSOR_IOCTL_SENSOR_ENABLE.\n", __FUNCTION__);
			data = (void __user *) arg;
			if(data == NULL)
			{
				MSE_ERR("IO parameter pointer is NULL!\r\n");
				retval = -EFAULT;
				break;
			}	
			if(copy_from_user(&enable, data, sizeof(enable)))
			{
				MSE_ERR("copy_from_user failed!\n");
				retval = -EFAULT;
				break;	
			}
#if 0  //henry mark           
			retval = yas537_m_enable(enable);
#else
                        //henry: maybe has bug here!
                        retval = yas_m_enable(enable);
#endif
			break;
		case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:	//read oritention sensor data
			break;
		  
		default:
			Printhh("[%s] not supported = 0x%04x\n", __FUNCTION__, cmd);
			MSE_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
		}

	err_out:
	return retval;

}
#ifdef CONFIG_COMPAT
static long yas537_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	void __user *arg64 = compat_ptr(arg);
	
	if((NULL == file->f_op) || (NULL == file->f_op->unlocked_ioctl))
	{
		return -ENOTTY;
	}
	Printhh("[%s] enter..\n", __FUNCTION__);

	switch(cmd)
	{
		case COMPAT_MSENSOR_IOCTL_INIT:
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_INIT, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_INIT failed!\n");
			}
			break;

		case COMPAT_MSENSOR_IOCTL_SET_POSTURE:			
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SET_POSTURE, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_SET_POSTURE failed!\n");
			}
			break;       

		case COMPAT_MSENSOR_IOCTL_SET_CALIDATA:			
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SET_CALIDATA, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_SET_CALIDATA failed!\n");
			}
			break;                               

		case COMPAT_MSENSOR_IOCTL_READ_CHIPINFO:
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CHIPINFO, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_READ_CHIPINFO failed!\n");
			}
			break; 


		case COMPAT_MSENSOR_IOCTL_READ_SENSORDATA:
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_READ_SENSORDATA failed!\n");
			}
			break;                 

		case COMPAT_MSENSOR_IOCTL_READ_POSTUREDATA:			           
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_POSTUREDATA, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_READ_POSTUREDATA failed!\n");
			}
			break;          

		case MSENSOR_IOCTL_READ_CALIDATA:       
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CALIDATA, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_READ_CALIDATA failed!\n");
			}
			break; 


		case COMPAT_MSENSOR_IOCTL_READ_CONTROL:			                           
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CONTROL, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_READ_CONTROL failed!\n");
			}
			break; 


		case COMPAT_MSENSOR_IOCTL_SET_CONTROL:			  
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SET_CONTROL, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_SET_CONTROL failed!\n");
			}
			break;


		case COMPAT_MSENSOR_IOCTL_SET_MODE:			                
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SET_MODE, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_SET_MODE failed!\n");
			}
			break;


		case COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE:
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SENSOR_ENABLE, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_SENSOR_ENABLE failed!\n");
			}
			break;

		case COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
    		if(NULL == arg64)
    		{
				MSE_ERR("invalid argument!\n");
				ret = -EINVAL;
				break;
			}
			ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_FACTORY_SENSORDATA, (unsigned long)arg64);
			if(ret < 0)
			{
				MSE_ERR("MSENSOR_IOCTL_READ_FACTORY_SENSORDATA failed!\n");
			}
			break;
			
		default:
			 MSE_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			 return -ENOIOCTLCMD;
			 break;			
	}

	return ret;
}

#endif

static struct file_operations yas537_fops = 
{
	.owner 			= THIS_MODULE,
	.open			= yas537_open,
	.release 		= yas537_release,
	.unlocked_ioctl = yas537_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl 	= yas537_compat_ioctl, 
#endif

};
static struct miscdevice yas537_misc = 
{
	.minor 			= MISC_DYNAMIC_MINOR,
	.name			= "msensor",
	.fops           = &yas537_fops,

};
#endif //henry merge from MTK new yas537

static int yas_m_open_report_data(int open)
{
	return 0;
}

static int yas_m_get_data(int *x, int *y, int *z, int *status)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
    //Printhh("[%s] enter..\n", __FUNCTION__);
//mdelay(g_iDelay);
	input_get_data(st->cal, x, y, z, status);
	return 0;
}

static int yas_o_enable(int en)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	set_euler_enable(st, en);
	return 0;
}

static int yas_o_set_delay(u64 ns)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	return set_euler_delay(st, (int32_t)ns /1000000);
}

static int yas_o_open_report_data(int open)
{
	return 0;
}

static int yas_o_get_data(int *x, int *y, int *z, int *status)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	input_get_data(st->euler, x, y, z, status);
	return 0;
}

static int yas_local_init(void);
static int yas_local_uninit(void);

static struct mag_init_info yas_init_info = {
	.name = "yas537",
	.init = yas_local_init,
	.uninit = yas_local_uninit,
};

static void yas_power(struct mag_hw *hw, unsigned int on)
{
	static unsigned int power_on;
#if 0    //androidM remove
	MAGN_LOG("[%s]\n", __func__);
	if (hw->power_id != POWER_NONE_MACRO) {
		MAGN_LOG("power %s\n", on ? "on" : "off");
		if (power_on == on) {
			MAGN_LOG("ignore power control: %d\n", on);
		} else if (on) {
			if (!hwPowerOn(hw->power_id, hw->power_vol, "yas537"))
				MAGN_ERR("power on fails!!\n");
		} else {
			if (!hwPowerDown(hw->power_id, "yas537"))
				MAGN_ERR("power off fail!!\n");
		}
	}
#endif
	power_on = on;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void yas_early_suspend(struct early_suspend *h)
{
	struct yas_state *st = container_of(h, struct yas_state,
			sus);
	int err = 0;
	MAGN_LOG("[%s]\n", __func__);
	if (atomic_read(&st->mag_enable) || atomic_read(&st->euler_enable))
		stop_mag(st);
	yas_power(st->hw, 0);
}

static void yas_late_resume(struct early_suspend *h)
{
	struct yas_state *st = container_of(h, struct yas_state, sus);
	int err;
	MAGN_LOG("[%s]\n", __func__);
	yas_power(st->hw, 1);
	if (atomic_read(&st->mag_enable) || atomic_read(&st->euler_enable))
		start_mag(st);
}
#endif


//uncali
int yas537_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
//	hwm_sensor_data* msensor_data;
	struct hwm_sensor_data* msensor_data;

#if 1  //uncali
        int x, y, z, status;
#endif


//#if DEBUG	
	struct i2c_client *client = this_client;  
	//struct akm09911_i2c_data *data = i2c_get_clientdata(client);
    	struct yas_state *data = i2c_get_clientdata(client);
//#endif

	//Printhh("[%s] enter..\n", __FUNCTION__);
	
	switch (command)
	{
		case SENSOR_DELAY:
			Printhh("[%s] SENSOR_DELAY..(no implement)\n", __FUNCTION__);
			#if 0
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 10)
				{
					value = 10;
				}				
				akmd_delay = value;				
			}	
			#endif
			break;

		case SENSOR_ENABLE:
			//Printhh("[%s] SENSOR_ENABLE..(no implement)\n", __FUNCTION__);
			Printhh("[%s] SENSOR_ENABLE..\n", __FUNCTION__);
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;
				Printhh("[%s] SENSOR_ENABLE value = %d..\n", __FUNCTION__, value);
				yas_m_enable(value);
				#if 0
				if(value == 1)
				{
					atomic_set(&m_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&m_flag, 0);
					if(atomic_read(&o_flag) == 0)
					{
					//	atomic_set(&m_flag, 0);  // if gyro, rv,la ,gravity open , then  m flag open 
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);
				#endif
			}
			break;

		case SENSOR_GET_DATA:
			//Printhh("[%s] SENSOR_GET_DATA..\n", __FUNCTION__);
			//if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				//msensor_data = (hwm_sensor_data *)buff_out;
				msensor_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);

                		msensor_data->values[0] = data->compass_data[0];
				msensor_data->values[1] = data->compass_data[1];
				msensor_data->values[2] = data->compass_data[2];

                                yas_m_get_data(&x, &y, &z, &status);    //henry: this is calibration data

				//bias
                		msensor_data->values[3] = data->compass_data[0] - x;
				msensor_data->values[4] = data->compass_data[1] - y;
				msensor_data->values[5] = data->compass_data[2] - z;
                
				msensor_data->status = data->accuracy_m;
				msensor_data->value_divide = CONVERT_M_DIV;
				#if 0
				Printhh("[%s] SENSOR_GET_DATA(%d, %d, %d) (status=%d, divide=%d)..\n", __FUNCTION__, 
                                    msensor_data->values[0], msensor_data->values[1], msensor_data->values[2],
                                    msensor_data->status, msensor_data->value_divide);
				Printhh("[%s] SENSOR_GET_DATA bias(%d, %d, %d)..\n", __FUNCTION__, 
                                    msensor_data->values[3], msensor_data->values[4], msensor_data->values[5]);
				#endif

				mutex_unlock(&sensor_data_mutex);
			}

			break;
		default:
			Printhh("[%s] msensor operate function no this parameter %d!\n", __FUNCTION__, command);
			printk(KERN_ERR "msensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

#if 1 // for 2nd compass
#define YAS_TYPE_M_MAG			(0x00000010) /*!< 3-axis Magnetometer */
#define YAS537_REG_DIDR			(0x80)
#define YAS537_DEVICE_ID		(0x07)	/* YAS537 (MS-3T) */
#endif

#define GPIO_GMC303_GP61 5  //jonny gp61

static int yas_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct yas_state *st;
	struct input_dev *raw = NULL, *cal = NULL, *euler = NULL;
	struct mag_control_path ctl = {0};
	struct mag_data_path mag_data = {0};
	int ret;
#if 1	//henry add for fix crash when bad ic
	int iRawReg = 0, iCalReg = 0, iEulerReg = 0;
#endif
	int ii = 0;

#if 1	//for 2nd compass
	uint8_t u8Data[8];
        static int siRetry = 0;
#endif

#if 1   //henry tt current
	int state;
#endif

#if 1   //mag_uncali
    struct hwmsen_object sobj;
    int err = 0;
    mutex_init(&sensor_data_mutex);
#endif


	MAGN_LOG("[%s]\n", __func__);
	this_client = i2c;
	Printhh("[%s] enter..11\n", __FUNCTION__);
	//mdelay(g_iDelay);
        Printhh("[%s] enter..22\n", __FUNCTION__);

#if 1	//henry add, set 400KHz
	Printhh("[%s] set to 400K\n", __FUNCTION__);
	this_client->timing = 400;
#endif

	st = kzalloc(sizeof(struct yas_state), GFP_KERNEL);
	if (!st) {
		ret = -ENOMEM;
		goto error_ret;
	}
	i2c_set_clientdata(i2c, st);

#if 1   // move forward
	mutex_init(&st->lock);
	mutex_init(&yas537_i2c_mutex);
#endif

#if 1   //for 2nd compass
retryRead:
	if( yas_device_read(YAS_TYPE_MAG, YAS537_REG_DIDR, u8Data, 1) < 0)
	//if( yas_device_read(YAS_TYPE_MAG, YAS537_REG_DIDR, u8Data, 1) >= 0)
	{
	    Printhh("[%s] yas_device_read() error!siRetry = %d \n", __FUNCTION__, siRetry);
            if(siRetry < 5){
                siRetry++;
                mdelay(100);
                goto retryRead;
            }
	    ret = -EFAULT;
	    goto error_ret;
	}
	else{
         //YAS537_DEVICE_ID
	    Printhh("[%s] yas_device_read() ok! \n", __FUNCTION__);
	    Printhh("[%s] u8Data[0] = %#x (should be %#x)\n", __FUNCTION__, u8Data[0], YAS537_DEVICE_ID);
	}
#endif

#if 1   //mag_uncali
	Printhh("[%s] enter..00\n", __FUNCTION__);
	mdelay(g_iDelay);
        Printhh("[%s] enter..0011\n", __FUNCTION__);
        Printhh("[%s] call hwmsen_attach()\n", __FUNCTION__);

    sobj.self = NULL;
    sobj.polling = 1;
    //sobj.sensor_operate = akm09911_operate;
    sobj.sensor_operate = yas537_operate;
    //if( (err = hwmsen_attach(ID_MAGNETIC_FIELD_UNCALIBRATED, &sobj))){
    if( (err = hwmsen_attach(ID_MAGNETIC_UNCALIBRATED, &sobj))){
	Printhh("[%s] attach ucali sensor fail = %d\n", __FUNCTION__, err);
        
    }
#endif

	raw = input_allocate_device();
	if (raw == NULL) {
		ret = -ENOMEM;
		goto error_free_device;
	}
	cal = input_allocate_device();
	if (cal == NULL) {
		ret = -ENOMEM;
		goto error_free_device;
	}
	euler = input_allocate_device();
	if (euler == NULL) {
		ret = -ENOMEM;
		goto error_free_device;
	}

	raw->name = YAS_RAW_NAME;
	raw->dev.parent = &i2c->dev;
	raw->id.bustype = BUS_I2C;
	set_bit(EV_ABS, raw->evbit);
	input_set_abs_params(raw, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(raw, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(raw, ABS_Z, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(raw, ABS_DUMMY, INT_MIN, INT_MAX, 0, 0);
	input_set_drvdata(raw, st);

	cal->name = YAS_CAL_NAME;
	cal->dev.parent = &i2c->dev;
	cal->id.bustype = BUS_I2C;
	set_bit(EV_ABS, cal->evbit);
	input_set_abs_params(cal, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(cal, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(cal, ABS_Z, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(cal, ABS_DUMMY, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(cal, ABS_STATUS, 0, 3, 0, 0);
	input_set_drvdata(cal, st);

	euler->name = YAS_EULER_NAME;
	euler->dev.parent = &i2c->dev;
	euler->id.bustype = BUS_I2C;
	set_bit(EV_ABS, euler->evbit);
	input_set_abs_params(euler, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(euler, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(euler, ABS_Z, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(euler, ABS_DUMMY, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(euler, ABS_STATUS, 0, 3, 0, 0);
	input_set_drvdata(euler, st);

	ret = input_register_device(raw);
	if (ret)
		goto error_free_device;
	iRawReg = 1;

	ret = input_register_device(cal);
	if (ret)
		goto error_free_device;
	iCalReg = 1;
	
	ret = input_register_device(euler);
	if (ret)
		goto error_free_device;
	iEulerReg = 1;

	ret = sysfs_create_group(&cal->dev.kobj, &yas_mag_attribute_group);
	if (ret)
		goto error_free_device;

	ret = sysfs_create_group(&euler->dev.kobj, &yas_euler_attribute_group);
	if (ret)
		goto error_free_device;

	atomic_set(&st->mag_enable, 0);
#if 0    //androidM
	st->hw = get_cust_mag_hw();
#else
	st->hw = get_cust_mag();
#endif

	st->raw = raw;
	st->cal = cal;
	st->euler = euler;

	st->mag_delay = YAS_DEFAULT_SENSOR_DELAY;
	st->euler_delay = YAS_DEFAULT_SENSOR_DELAY;
	st->mag.callback.device_open = yas_device_open;
	st->mag.callback.device_close = yas_device_close;
	st->mag.callback.device_write = yas_device_write;
	st->mag.callback.device_read = yas_device_read;
	st->mag.callback.usleep = yas_usleep;
	st->mag.callback.current_time = yas_current_time;
	INIT_DELAYED_WORK(&st->work, yas_work_func);
#if 0   // move forward
	mutex_init(&st->lock);
	mutex_init(&yas537_i2c_mutex);
#endif

#ifdef YAS537_I2C_USE_DMA
//henry will enter here!
/********try to alloc dma memory 3times************/
	st->dma_va = (char *)dma_alloc_coherent(&(this_client->dev), 1024, &(st->dma_pa), GFP_KERNEL);
	if(unlikely(NULL==st->dma_va))
	{
		st->dma_va = (char *)dma_alloc_coherent(&(this_client->dev), 1024, &(st->dma_pa), GFP_KERNEL);
		if(unlikely(NULL==st->dma_va))
		{
			st->dma_va = (char *)dma_alloc_coherent(&(this_client->dev), 1024, &(st->dma_pa), GFP_KERNEL);
			if(unlikely(NULL==st->dma_va))
			{
				MAGN_ERR("%s  dma_alloc_coherent failed!\n",__FUNCTION__);
			}
		}
	}
#endif

#if 1   //henry merge from MTK new yas537
	for (ii = 0; ii < 3; ii++)
		st->compass_data[ii] = 0;
#endif
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
	st->sus.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	st->sus.suspend = yas_early_suspend;
	st->sus.resume = yas_late_resume;
	register_early_suspend(&st->sus);
#endif*/

	ret = yas_mag_driver_init(&st->mag);
	if (ret < 0) {
		ret = -EFAULT;
		goto error_remove_sysfs;
	}

	Printhh("[%s] call st->mag.init() \n", __FUNCTION__);
	ret = st->mag.init();	// == yas_init()
	if (ret < 0) {
		ret = -EFAULT;
		goto error_remove_sysfs;
	}

#if 1	//henry add
	Printhh("[%s] call set_position(%d) 111 \n", __FUNCTION__, st->hw->direction);
//mdelay(g_iDelay);

	//Printhh("[%s] call set_position(%d) 222 \n", __FUNCTION__, st->hw->direction);

	st->mag.set_position(st->hw->direction);	//== yas_set_position()
#endif

    ret = yas537_create_attr(&(yas_init_info.platform_diver_addr->driver));
	if(ret < 0)
	{
		MAGN_ERR("yas537_create_attr error! \n");
	}

#if 1   //henry merge from MTK new yas537
	ret = misc_register(&yas537_misc);
	if(ret < 0)
	{
		MSE_ERR("misc_register(&yas537_misc) error!\n");
		goto error_misc_register;
	}
#endif

	ctl.is_use_common_factory = false;
	ctl.m_enable = yas_m_enable;
	ctl.m_set_delay  = yas_m_set_delay;
	ctl.m_open_report_data = yas_m_open_report_data;
	ctl.o_enable = yas_o_enable;
	ctl.o_set_delay  = yas_o_set_delay;
	ctl.o_open_report_data = yas_o_open_report_data;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = st->hw->is_batch_supported;

	ret = mag_register_control_path(&ctl);
	if (ret) {
		MAGN_ERR("register mag control path ret\n");
		goto error_remove_sysfs;
	}

	mag_data.div_m = CONVERT_M_DIV;
	mag_data.div_o = CONVERT_O_DIV;
	mag_data.get_data_o = yas_o_get_data;
	mag_data.get_data_m = yas_m_get_data;

	ret = mag_register_data_path(&mag_data);
	if (ret) {
		MAGN_ERR("register st control path ret\n");
		goto error_remove_sysfs;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	st->sus.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		st->sus.suspend  = yas_early_suspend,
		st->sus.resume   = yas_late_resume,
		register_early_suspend(&st->sus);
#endif
	MAGN_ERR("%s: OK\n", __func__);
	Printhh("[%s] probe OK.. \n", __FUNCTION__);
	Printhh("[%s] probe OK.. \n", __FUNCTION__);
	Printhh("[%s] probe OK.. \n", __FUNCTION__);
	g_iInitFlag = 1;
    
#if 1   //henry tt current
	if( (g_iHwId == 0x0) || (g_iHwId == 0x01) || (g_iHwId == 0x02) || (g_iHwId == 0x03) || (g_iHwId == 0x04) || (g_iHwId == 0x05))
	{
		//PDP:0x0, DP1:0x1, DP2:0x2, SP:0x3, AP:0x4, TP:0x5
		Printhh("[%s] set PGIO input_high.. \n", __FUNCTION__);
		mt_set_gpio_dir(GPIO_GMC303_GP61, 0);
		mt_set_gpio_out(GPIO_GMC303_GP61, 1);
		udelay(100);
	}
	else{
		//PQ:0x6 or others
		// do nothing
		Printhh("[%s] hwid=PQ PGIO do nothing.. \n", __FUNCTION__);
	}

	state = mt_get_gpio_dir(GPIO_GMC303_GP61);
	MAGN_LOG("[%s] mt_get_gpio_dir = %#x\n", __func__, state);
    
	state = mt_get_gpio_out(GPIO_GMC303_GP61);
	MAGN_LOG("[%s] mt_get_gpio_out = %#x\n", __func__, state);
    
	state = mt_get_gpio_in(GPIO_GMC303_GP61);
	MAGN_LOG("[%s] mt_get_gpio_in = %#x\n", __func__, state);
    
	state = mt_get_gpio_mode(GPIO_GMC303_GP61);
	MAGN_LOG("[%s] mt_get_gpio_mode = %#x\n", __func__, state);

	state = mt_get_gpio_pull_enable(GPIO_GMC303_GP61);
	MAGN_LOG("[%s] mt_get_gpio_pull_enable = %#x\n", __func__, state);

	state = mt_get_gpio_pull_select(GPIO_GMC303_GP61);
	MAGN_LOG("[%s] mt_get_gpio_pull_select = %#x\n", __func__, state);
#endif

	return 0;

#if 1   //henry merge from MTK new yas537
	misc_deregister(&yas537_misc);
error_misc_register:
#endif

error_remove_sysfs:
	sysfs_remove_group(&st->cal->dev.kobj, &yas_mag_attribute_group);
	sysfs_remove_group(&st->euler->dev.kobj, &yas_euler_attribute_group);
error_free_device:
	if (raw != NULL) {
		if(iRawReg == 1)
		    input_unregister_device(raw);
		else
		    input_free_device(raw);
	}
	if (cal != NULL) {
		if(iCalReg == 1)
		    input_unregister_device(cal);
		else
		    input_free_device(cal);
	}
	if (euler != NULL) {
		if(iEulerReg == 1)
		    input_unregister_device(euler);
		else
		    input_free_device(euler);
	}
	kfree(st);
error_ret:
	i2c_set_clientdata(i2c, NULL);
	this_client = NULL;
	g_iInitFlag = -1;
	return ret;
}

static int yas_remove(struct i2c_client *i2c)
{
	struct yas_state *st = i2c_get_clientdata(i2c);
	MAGN_LOG("[%s]\n", __func__);
	if (st != NULL) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&st->sus);
#endif
		stop_mag(st);
		st->mag.term();
		sysfs_remove_group(&st->cal->dev.kobj, &yas_mag_attribute_group);
		sysfs_remove_group(&st->euler->dev.kobj, &yas_euler_attribute_group);
		input_unregister_device(st->raw);
		input_free_device(st->raw);
		input_unregister_device(st->cal);
		input_free_device(st->cal);
		input_unregister_device(st->euler);
		input_free_device(st->euler);
		kfree(st);
		this_client = NULL;
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int yas_suspend(struct device *dev)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	MAGN_LOG("[%s]\n", __func__);
	if (atomic_read(&st->mag_enable) || atomic_read(&st->euler_enable))
		stop_mag(st);
	yas_power(st->hw, 0);
	return 0;
}

static int yas_resume(struct device *dev)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	MAGN_LOG("[%s]\n", __func__);
	yas_power(st->hw, 1);
	if (atomic_read(&st->mag_enable) || atomic_read(&st->euler_enable))
		start_mag(st);
	return 0;
}

static SIMPLE_DEV_PM_OPS(yas_pm_ops, yas_suspend, yas_resume);
#define YAS_PM_OPS (&yas_pm_ops)
#else
#define YAS_PM_OPS NULL
#endif

static const struct i2c_device_id yas_id[] = {
	{YAS_MTK_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, yas_id);

#if 0 //androidM remove
static struct i2c_board_info __initdata i2c_yas53x = {
	I2C_BOARD_INFO(YAS_MTK_NAME, YAS_ADDRESS)
};
#endif

#if 1   // for 2nd compass
#ifdef CONFIG_OF //androidM add
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,MSENSOR"},
	{},
};
#endif
#endif

static struct i2c_driver yas_driver = {
	.driver = {
		.name	= YAS_MTK_NAME,
		.owner	= THIS_MODULE,
		.pm	= YAS_PM_OPS,
#if 1   // for 2nd compass
#ifdef CONFIG_OF    //androidM add
		.of_match_table = mag_of_match,
#endif
#endif
	},
	.probe		= yas_probe,
	.remove		= yas_remove,
	.id_table	= yas_id,
};

static int yas_local_init(void)
{
	//struct mag_hw *hw = get_cust_mag_hw();
	struct mag_hw *hw = get_cust_mag();
    
        Printhh("[%s] enter..11\n", __FUNCTION__);
//mdelay(g_iDelay);
        Printhh("[%s] enter..22\n", __FUNCTION__);

	MAGN_LOG("[%s]\n", __func__);
	yas_power(hw, 1);
	if (i2c_add_driver(&yas_driver)) {
		MAGN_ERR("i2c_add_driver error\n");
		Printhh("[%s] i2c_add_driver error\n", __FUNCTION__);
		return -1;
	}
	Printhh("[%s] g_iInitFlag = %d\n", __FUNCTION__, g_iInitFlag);
    
	if (-1 == g_iInitFlag)
		return -1;    
	return 0;
}

static int yas_local_uninit(void)
{
	//struct mag_hw *hw = get_cust_mag_hw();
	struct mag_hw *hw = get_cust_mag();
    
	MAGN_LOG("[%s]\n", __func__);
	yas_power(hw, 0);
	i2c_del_driver(&yas_driver);
	return 0;
}


static int __init yas_init(void)
{
	const char *name = "mediatek,yamaha537";
	int state;

	//struct mag_hw *hw = get_cust_mag_hw();    //androidM remove

	g_pHw = get_mag_dts_func(name, g_pHw);
	if (!g_pHw){
		MSE_ERR("get cust_mag dts info fail\n");
		Printhh("[%s] get cust_mag dts info fail...\n", __FUNCTION__);
        }
	Printhh("[%s] board_type = %#x\n", __FUNCTION__, g_iHwId);

	Printhh("[%s] g_pHw->i2c_num = %#x\n", __FUNCTION__, g_pHw->i2c_num);
	Printhh("[%s] g_pHw->i2c_addr[0] = %#x\n", __FUNCTION__, g_pHw->i2c_addr[0]);
	Printhh("[%s] g_pHw->i2c_addr[1] = %#x\n", __FUNCTION__, g_pHw->i2c_addr[1]);
	Printhh("[%s] g_pHw->direction = %#x\n", __FUNCTION__, g_pHw->direction);
	Printhh("[%s] g_pHw->power_id = %#x\n", __FUNCTION__, g_pHw->power_id);
	Printhh("[%s] g_pHw->power_vol = %#x\n", __FUNCTION__, g_pHw->power_vol);

#if 0
	mt_set_gpio_mode(GPIO_GMC303_GP61, 0);
	mt_set_gpio_dir(GPIO_GMC303_GP61, 1);
	mt_set_gpio_out(GPIO_GMC303_GP61, 0);
	udelay(100);
#endif
	state = mt_get_gpio_dir(GPIO_GMC303_GP61);
	MAGN_LOG("[%s] mt_get_gpio_dir = %#x\n", __func__, state);
    
	state = mt_get_gpio_out(GPIO_GMC303_GP61);
	MAGN_LOG("[%s] mt_get_gpio_out = %#x\n", __func__, state);
    
	state = mt_get_gpio_in(GPIO_GMC303_GP61);
	MAGN_LOG("[%s] mt_get_gpio_in = %#x\n", __func__, state);
    
	state = mt_get_gpio_mode(GPIO_GMC303_GP61);
	MAGN_LOG("[%s] mt_get_gpio_mode = %#x\n", __func__, state);


	MAGN_LOG("[%s]: i2c_number=%d\n", __func__, g_pHw->i2c_num);
#if 0    //androidM remove
	i2c_register_board_info(hw->i2c_num, &i2c_yas53x, 1);
#endif

	// for 2nd compass
#if 0
	if (g_pHw->i2c_addr[0] != 0){
		struct i2c_board_info yas537_i2c={ I2C_BOARD_INFO(YAS_MTK_NAME, g_pHw->i2c_addr[0])};

		//yas537_i2c.addr =g_pHw->i2c_addr[0];
		i2c_register_board_info(g_pHw->i2c_num, &yas537_i2c, 1);
		
		MAGN_LOG("[%s] yas537_i2c.type = %s\n", __func__, yas537_i2c.type);
		MAGN_LOG("[%s] yas537_i2c.addr = %x\n", __func__, yas537_i2c.addr);
        	Printhh("[%s] yas537_i2c.type = %s\n", __FUNCTION__, yas537_i2c.type);
        	Printhh("[%s] yas537_i2c.addr = %#x\n", __FUNCTION__, yas537_i2c.addr);

	}
#endif
	mag_driver_add(&yas_init_info);
	return 0;
}

static void __exit yas_exit(void)
{
	MAGN_LOG("[%s]\n", __func__);
}

module_init(yas_init);
module_exit(yas_exit);

MODULE_DESCRIPTION("YAS537 compass driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.9.0.1025");
