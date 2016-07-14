/*
* Copyright (C) 2010 Trusted Logic S.A.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/nfc/pn547.h>
#include <linux/wakelock.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif


//#include <cust_gpio_usage.h>
//#include <cust_eint.h>
//#include <mach/mt_gpio.h>
//#include <mach/eint.h>

#define PN547_USE_KERNEL_STD


#define MAX_BUFFER_SIZE 512

#define NXP_KR_READ_IRQ_MODIFY

#ifdef NXP_KR_READ_IRQ_MODIFY
static bool do_reading;
static bool cancle_read;
#endif

#define NFC_DEBUG 1
#define MAX_TRY_I2C_READ 10
#define I2C_ADDR_READ_L 0x51
#define I2C_ADDR_READ_H 0x57

#define NFC_I2C_BUSNUM  1
#define I2C_ID_NAME "pn547"

struct pn547_dev *pn547_dev_ptr = NULL;
static struct i2c_board_info __initdata nfc_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, 0x28)}; 
//static struct i2c_board_info __initdata nfc_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, 0x2B)};

#define Enable_I2C_DMA 1

#ifdef Enable_I2C_DMA
static char *I2CDMAWriteBuf = NULL;
static unsigned int I2CDMAWriteBuf_pa;
static char *I2CDMAReadBuf = NULL;
static unsigned int I2CDMAReadBuf_pa;
#endif

struct pn547_dev	{
		wait_queue_head_t	read_wq;
		struct mutex	read_mutex;
		struct i2c_client	*client;
		struct miscdevice	pn547_device;
		struct wake_lock	nfc_wake_lock;
		unsigned int	ven_gpio;
		unsigned int	firm_gpio;
		unsigned int	irq_gpio;
		atomic_t	irq_enabled;
};

struct pn547_i2c_platform_data {
		unsigned int irq_gpio;
		unsigned int ven_gpio;
		unsigned int firm_gpio;
};



#if defined(PN547_USE_KERNEL_STD)

static int mt_nfc_probe(struct platform_device *pdev);
static int mt_nfc_remove(struct platform_device *pdev);
static ssize_t pn547_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *offset);
/*static void pn547_dev_irq_handler(void);*/
static irqreturn_t pn547_dev_irq_handler(int irq, void *data);	/*IRQ handler */
static ssize_t pn547_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset);
static int pn547_dev_open(struct inode *inode, struct file *filp);
static int pn547_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int pn547_remove(struct i2c_client *client);
static long pn547_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int mt_nfc_gpio_init(void);
static int mt_nfc_pinctrl_init(struct platform_device *pdev);
static int mt_nfc_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s);
static int mt_nfc_get_gpio_value(int gpio_num);

struct platform_device *nfc_plt_dev = NULL;
struct pinctrl *gpctrl = NULL;
struct pinctrl_state *st_ven_h = NULL;
struct pinctrl_state *st_ven_l = NULL;
struct pinctrl_state *st_rst_h = NULL;
struct pinctrl_state *st_rst_l = NULL;
struct pinctrl_state *st_eint_h = NULL;
struct pinctrl_state *st_eint_l = NULL;
struct pinctrl_state *st_irq_init = NULL;

/*  NFC IRQ */
static u32 nfc_irq;
static int nfc_irq_count;	/*= 0;*//* unnecessary initialise */

static struct pn547_i2c_platform_data pn547_platform_data;
#else
static struct pn547_i2c_platform_data pn547_platform_data = {
		.irq_gpio = GPIO_IRQ_NFC_PIN,
		.ven_gpio = GPIO_NFC_VENB_PIN,
		.firm_gpio = GPIO_NFC_EINT_PIN,
};
#endif//PN547_USE_KERNEL_STD

static const struct file_operations pn547_dev_fops = {
		.owner	= THIS_MODULE,
		.llseek	= no_llseek,
		.read	= pn547_dev_read,
		.write	= pn547_dev_write,
		.open	= pn547_dev_open,
		.unlocked_ioctl = pn547_dev_ioctl,
};

#ifdef CONFIG_OF 
static struct of_device_id pn547_i2c_table[] = {
{ .compatible = "nxp,pn547",}, 
{ },
};
#else
#define pn547_i2c_table NULL
#endif


static const struct i2c_device_id pn547_id[] = {
		{ "pn547", 0 },
		{ }
};

static struct i2c_driver pn547_driver = {
		.id_table	= pn547_id,
		.probe	= pn547_probe,
		.remove	= pn547_remove,
		.driver	= {
		.owner	= THIS_MODULE,
		.name	= "pn547",
		.of_match_table = pn547_i2c_table,
		},
};

#if defined(PN547_USE_KERNEL_STD)
/*  platform driver */
static const struct of_device_id nfc_dev_of_match[] = {
	{.compatible = "mediatek,nfc-gpio-v2",},
	{},
};

static struct platform_driver mtk_nfc_platform_driver = {
	.probe = mt_nfc_probe,
	.remove = mt_nfc_remove,
	.driver = {
		   .name = I2C_ID_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = nfc_dev_of_match,
#endif
		   },
};

static int mt_nfc_get_gpio_value(int gpio_num)
{
	int value = 0;

	value = __gpio_get_value(gpio_num);

	return value;
}

static void pn547_enable_irq(u32 irq_line)
{
	/* pr_debug("%s : irq_line=%d, nfc_irq_count=%d.\n", __func__,
		 irq_line, nfc_irq_count); */

	nfc_irq_count++;	/* It must set before call enable_irq */

	enable_irq(irq_line);

	/* pr_debug("%s : nfc_irq_count = %d.\n", __func__, nfc_irq_count); */
}

static void pn547_disable_irq(u32 irq_line)
{
	/* pr_debug("%s : irq_line=%d, nfc_irq_count=%d.\n", __func__,
		 irq_line, nfc_irq_count); */
	if (nfc_irq_count >= 1) {
		disable_irq_nosync(irq_line);
		nfc_irq_count--;
	} else {
		pr_debug("%s : disable irq fail.\n", __func__);
	}
	/* pr_debug("%s : nfc_irq_count = %d.\n", __func__, nfc_irq_count); */
}

static int mt_nfc_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s)
{
	int ret = 0;
	if (p != NULL && s != NULL) {
		ret = pinctrl_select_state(p, s);
	} else {
		pr_debug("%s : pinctrl_select err\n", __func__);
		ret = -1;
	}
	return ret;
}

static int mt_nfc_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	gpctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gpctrl)) {
		dev_err(&pdev->dev, "Cannot find pinctrl!");
		ret = PTR_ERR(gpctrl);
		goto end;
	}

	st_ven_h = pinctrl_lookup_state(gpctrl, "ven_high");
	if (IS_ERR(st_ven_h)) {
		ret = PTR_ERR(st_ven_h);
		pr_debug("%s : pinctrl err, ven_high\n", __func__);
	}

	st_ven_l = pinctrl_lookup_state(gpctrl, "ven_low");
	if (IS_ERR(st_ven_l)) {
		ret = PTR_ERR(st_ven_l);
		pr_debug("%s : pinctrl err, ven_low\n", __func__);
	}

	st_rst_h = pinctrl_lookup_state(gpctrl, "rst_high");
	if (IS_ERR(st_rst_h)) {
		ret = PTR_ERR(st_rst_h);
		pr_debug("%s : pinctrl err, rst_high\n", __func__);
	}

	st_rst_l = pinctrl_lookup_state(gpctrl, "rst_low");
	if (IS_ERR(st_rst_l)) {
		ret = PTR_ERR(st_rst_l);
		pr_debug("%s : pinctrl err, rst_low\n", __func__);
	}

	st_eint_h = pinctrl_lookup_state(gpctrl, "eint_high");
	if (IS_ERR(st_eint_h)) {
		ret = PTR_ERR(st_eint_h);
		pr_debug("%s : pinctrl err, eint_high\n", __func__);
	}

	st_eint_l = pinctrl_lookup_state(gpctrl, "eint_low");
	if (IS_ERR(st_eint_l)) {
		ret = PTR_ERR(st_eint_l);
		pr_debug("%s : pinctrl err, eint_low\n", __func__);
	}

	st_irq_init = pinctrl_lookup_state(gpctrl, "irq_init");
	if (IS_ERR(st_irq_init)) {
		ret = PTR_ERR(st_irq_init);
		pr_debug("%s : pinctrl err, irq_init\n", __func__);
	}
end:

	return ret;
}

static int mt_nfc_gpio_init(void)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-gpio-v2");

	if (node) {
		of_property_read_u32_array(node, "gpio-ven",
					   &(pn547_platform_data.ven_gpio), 1);
		/*of_property_read_u32_array(node, "gpio-rst",
					   &(pn547_platform_data.sysrstb_gpio),
					   1);*/
		of_property_read_u32_array(node, "gpio-eint",
					   &(pn547_platform_data.firm_gpio),
					   1);
		of_property_read_u32_array(node, "gpio-irq",
					   &(pn547_platform_data.irq_gpio), 1);
	} else {
		pr_debug("%s : get gpio num err.\n", __func__);
		return -1;
	}

	return 0;
}

static int mt_nfc_probe(struct platform_device *pdev)
{
	int ret = 0;

	nfc_plt_dev = pdev;

	pr_debug("%s : &nfc_plt_dev=%p\n", __func__, nfc_plt_dev);

	/* pinctrl init */
	ret = mt_nfc_pinctrl_init(pdev);

	/* gpio init */
	if (mt_nfc_gpio_init() != 0)
		pr_debug("%s : mt_nfc_gpio_init err.\n", __func__);

	return 0;
}

static int mt_nfc_remove(struct platform_device *pdev)
{
	pr_debug("%s : &pdev=%p\n", __func__, pdev);
	return 0;
}

#endif//PN547_USE_KERNEL_STD


//static void pn547_dev_irq_handler(void)
static irqreturn_t pn547_dev_irq_handler(int irq, void *data)	/*IRQ handler */
{
		struct pn547_dev *pn547_dev = pn547_dev_ptr;

		if(NULL == pn547_dev) {
			printk(KERN_DEBUG "pn547_dev NULL\n");
			return IRQ_HANDLED;
		}


#if defined(PN547_USE_KERNEL_STD)
		if(!mt_nfc_get_gpio_value(pn547_dev->irq_gpio)) {
				#if NFC_DEBUG
				pr_err("%s, irq_gpio = %d\n", __func__, mt_nfc_get_gpio_value(pn547_dev->irq_gpio));
				#endif		
				return IRQ_HANDLED;
		}				
#else

		if (!mt_get_gpio_in(pn547_dev->irq_gpio)) {
				#if NFC_DEBUG
				pr_err("%s, irq_gpio = %d\n", __func__, mt_get_gpio_in(pn547_dev->irq_gpio));
				#endif		
				return IRQ_HANDLED;
		}				
#endif//PN547_USE_KERNEL_STD


		#ifdef NXP_KR_READ_IRQ_MODIFY
		do_reading = true;
		#endif
		
		//!!!Temp
		//pn547_disable_irq(nfc_irq);
		//!!!Temp
		
		/* Wake up waiting readers */
		wake_up(&pn547_dev->read_wq);
		
		#if NFC_DEBUG
		pr_info("%s, IRQ_HANDLED\n", __func__);
		#endif
		wake_lock_timeout(&pn547_dev->nfc_wake_lock, 2 * HZ);
		return IRQ_HANDLED;
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
		struct pn547_dev *pn547_dev = filp->private_data;
		#ifndef Enable_I2C_DMA
		char tmp[MAX_BUFFER_SIZE];
		#endif
		int ret = 0;

#if 0
		int readingWatchdog = 0;
#endif

		
		if (count > MAX_BUFFER_SIZE)
			count = MAX_BUFFER_SIZE;
		
		#if NFC_DEBUG
		
#if defined(PN547_USE_KERNEL_STD)
		dev_info(&pn547_dev->client->dev, "%s : reading %zu bytes. irq=%s\n",	__func__, count, 
																	mt_nfc_get_gpio_value(pn547_dev->irq_gpio) ? "1" : "0");

#else		
		dev_info(&pn547_dev->client->dev, "%s : reading %zu bytes. irq=%s\n",	__func__, count, 
																	mt_get_gpio_in(pn547_dev->irq_gpio) ? "1" : "0");
#endif																	
		dev_info(&pn547_dev->client->dev, "pn547 : + r\n");
		#endif
		
		mutex_lock(&pn547_dev->read_mutex);
		
		//!!!Temp
		//pn547_enable_irq(nfc_irq);
		//!!!Temp
		

#if 0		
		wait_irq:
#endif



#if defined(PN547_USE_KERNEL_STD)
		if(!mt_nfc_get_gpio_value(pn547_dev->irq_gpio)){
#else

		if (!mt_get_gpio_in(pn547_dev->irq_gpio)) {
#endif//		PN547_USE_KERNEL_STD
			#ifdef NXP_KR_READ_IRQ_MODIFY
			do_reading = false;
			#endif
			if (filp->f_flags & O_NONBLOCK) {
				dev_info(&pn547_dev->client->dev, "%s : O_NONBLOCK\n", __func__);
				ret = -EAGAIN;
				goto fail;
			}
			#if NFC_DEBUG
			dev_info(&pn547_dev->client->dev, "wait_event_interruptible : in\n");
			#endif
		
			#ifdef NXP_KR_READ_IRQ_MODIFY
			ret = wait_event_interruptible(pn547_dev->read_wq, do_reading);
			#else
			ret = wait_event_interruptible(pn547_dev->read_wq, mt_get_gpio_in(pn547_dev->irq_gpio));
			#endif
		
			#if NFC_DEBUG
			dev_info(&pn547_dev->client->dev,	"wait_event_interruptible : out\n");
			#endif

			#ifdef NXP_KR_READ_IRQ_MODIFY
			if (cancle_read == true) {
				cancle_read = false;
				ret = -1;
				goto fail;
			}
			#endif
		
			if (ret)
				goto fail;
		}
		
		#ifdef Enable_I2C_DMA
		pn547_dev->client->addr = (pn547_dev->client->addr & I2C_MASK_FLAG  );// | I2C_DMA_FLAG;
		pn547_dev->client->ext_flag |= I2C_DMA_FLAG;
		//pn547_dev->client->ext_flag |= I2C_DIRECTION_FLAG;
		pn547_dev->client->ext_flag |= I2C_A_FILTER_MSG;
		pn547_dev->client->timing = 400;

		/* Read data */
   	ret = i2c_master_recv(pn547_dev->client, (unsigned char *)(uintptr_t)I2CDMAReadBuf_pa, count);
		#else
		ret = i2c_master_recv(pn547_dev->client, tmp, count);
		#endif
		
		/* If bad frame(from 0x51 to 0x57) is received from pn547,
		* we need to read again after waiting that IRQ is down.
		* if data is not ready, pn547 will send from 0x51 to 0x57. */

#if 0
		if ((I2C_ADDR_READ_L <= tmp[0] && tmp[0] <= I2C_ADDR_READ_H)
		&& readingWatchdog < MAX_TRY_I2C_READ) {
				pr_warn("%s: data is not ready yet.data = 0x%x, cnt=%d\n",
																					__func__, tmp[0], readingWatchdog);
				usleep_range(2000, 2000); /* sleep 2ms to wait for IRQ */
				readingWatchdog++;
				goto wait_irq;
		}
#endif

		mutex_unlock(&pn547_dev->read_mutex);
		
		if (ret < 0) {
			dev_err(&pn547_dev->client->dev, "%s: i2c_master_recv returned %d\n", __func__, ret);
			return ret;
		}
		if (ret > count) {
			dev_err(&pn547_dev->client->dev, "%s: received too many bytes from i2c (%d)\n", __func__, ret);
			return -EIO;
		}
		
		#ifdef Enable_I2C_DMA
		if (copy_to_user(buf, I2CDMAReadBuf, ret)) {
		#else
		if (copy_to_user(buf, tmp, ret)) {
		#endif
				dev_err(&pn547_dev->client->dev, "%s : failed to copy to user space\n", __func__);
				return -EFAULT;
		}
		return ret;
		
		fail:
		mutex_unlock(&pn547_dev->read_mutex);
		return ret;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
		struct pn547_dev *pn547_dev;
		#ifndef Enable_I2C_DMA
		char tmp[MAX_BUFFER_SIZE];
		#endif
		int ret = 0, retry = 2;
		#if NFC_DEBUG
		int i = 0;
		#endif
		
		pn547_dev = filp->private_data;
		
		#if NFC_DEBUG
		dev_info(&pn547_dev->client->dev, "pn547 : + w\n");
		for (i = 0; i < count; i++)
				dev_info(&pn547_dev->client->dev, "buf[%d] = 0x%x\n",	i, buf[i]);
		#endif
		
		if (count > MAX_BUFFER_SIZE)
				count = MAX_BUFFER_SIZE;

		#ifdef Enable_I2C_DMA
		if (copy_from_user(I2CDMAWriteBuf, buf, count)) {
		#else
		if (copy_from_user(tmp, buf, count)) {
		#endif
				dev_err(&pn547_dev->client->dev, "%s : failed to copy from user space\n", __func__);
				return -EFAULT;
		}
		#if NFC_DEBUG
		dev_info(&pn547_dev->client->dev, "%s : writing %zu bytes.\n", __func__, count);
		#endif

		if (atomic_read(&pn547_dev->irq_enabled) == 1) {

#if defined(PN547_USE_KERNEL_STD)
			pn547_disable_irq(nfc_irq);
#else
			mt_eint_mask(CUST_EINT_IRQ_NFC_NUM);
#endif

			atomic_set(&pn547_dev->irq_enabled, 0);
		}

		/* Write data */
		do {
				retry--;

				#ifdef Enable_I2C_DMA
				pn547_dev->client->addr = (pn547_dev->client->addr & I2C_MASK_FLAG);// | I2C_DMA_FLAG;
   				pn547_dev->client->ext_flag |= I2C_DMA_FLAG;
   				//pn547_dev->client->ext_flag |= I2C_DIRECTION_FLAG;
   				pn547_dev->client->ext_flag |= I2C_A_FILTER_MSG;
				pn547_dev->client->timing = 400;

				ret = i2c_master_send(pn547_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa, count);
				#else
				ret = i2c_master_send(pn547_dev->client, tmp, count);
				#endif
			
				if (ret == count)
				{
						break;
				}
				else
				{
					  dev_err(&pn547_dev->client->dev, "I2C ERROR and skip retry\n");
					  ret = -EIO;
					  break;
				}
			
				usleep_range(6000, 10000); /* Retry, chip was in standby */
				#if NFC_DEBUG
				dev_info(&pn547_dev->client->dev, "retry = %d\n", retry);
				#endif
		} while (retry);
		
		#if NFC_DEBUG
		dev_info(&pn547_dev->client->dev, "pn547 : - w\n");
		#endif
		
		if (ret != count) {
				dev_err(&pn547_dev->client->dev, "%s : i2c_master_send returned %d, %d\n", __func__, ret, retry);
				ret = -EIO;
		}

		if (atomic_read(&pn547_dev->irq_enabled) == 0) {
			atomic_set(&pn547_dev->irq_enabled, 1);

#if defined(PN547_USE_KERNEL_STD)
			pn547_enable_irq(nfc_irq);
#else
			mt_eint_unmask(CUST_EINT_IRQ_NFC_NUM);
#endif

			
		}

		return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
		struct pn547_dev *pn547_dev = container_of(filp->private_data, struct pn547_dev, pn547_device);
		
		filp->private_data = pn547_dev;
		pn547_dev_ptr = pn547_dev;
		
		dev_info(&pn547_dev->client->dev, "%s : %d,%d\n", __func__, imajor(inode), iminor(inode));
		
		return 0;
}

static long pn547_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
		struct pn547_dev *pn547_dev = filp->private_data;
#if defined(PN547_USE_KERNEL_STD)			
		int ret = 0;
#endif//PN547_USE_KERNEL_STD

		switch (cmd) {
				case PN547_SET_PWR:
						if (arg == 2) {
								/* power on with firmware download (requires hw reset) */
								
#if defined(PN547_USE_KERNEL_STD)										
								ret = mt_nfc_pinctrl_select(gpctrl, st_ven_h);
								ret = mt_nfc_pinctrl_select(gpctrl, st_eint_h);		
								usleep_range(10000, 10000);
								ret = mt_nfc_pinctrl_select(gpctrl, st_ven_l);
								usleep_range(10000, 10000);
								ret = mt_nfc_pinctrl_select(gpctrl, st_ven_h);
								usleep_range(10000, 10000);
								if (atomic_read(&pn547_dev->irq_enabled) == 0) {
										atomic_set(&pn547_dev->irq_enabled, 1);
										pn547_enable_irq(nfc_irq);
								}								
#else
								

								mt_set_gpio_out(pn547_dev->ven_gpio, GPIO_OUT_ONE);
								mt_set_gpio_out(pn547_dev->firm_gpio, GPIO_OUT_ONE);
								usleep_range(10000, 10000);
								mt_set_gpio_out(pn547_dev->ven_gpio, GPIO_OUT_ZERO);
								usleep_range(10000, 10000);
								mt_set_gpio_out(pn547_dev->ven_gpio, GPIO_OUT_ONE);
								usleep_range(10000, 10000);
								if (atomic_read(&pn547_dev->irq_enabled) == 0) {
										atomic_set(&pn547_dev->irq_enabled, 1);
										mt_eint_unmask(CUST_EINT_IRQ_NFC_NUM);
								}
#endif//PN547_USE_KERNEL_STD						
								dev_info(&pn547_dev->client->dev,	"%s power on with firmware, irq=%d\n", __func__, atomic_read(&pn547_dev->irq_enabled));
						} else if (arg == 1) {
								/* power on */
								
#if defined(PN547_USE_KERNEL_STD)			
								ret = mt_nfc_pinctrl_select(gpctrl, st_eint_l);		
								ret = mt_nfc_pinctrl_select(gpctrl, st_ven_h);
								usleep_range(10000, 10000);
								if (atomic_read(&pn547_dev->irq_enabled) == 0) {
										atomic_set(&pn547_dev->irq_enabled, 1);
										pn547_enable_irq(nfc_irq);
								}									
#else								
								
								mt_set_gpio_out(pn547_dev->firm_gpio, GPIO_OUT_ZERO);
								mt_set_gpio_out(pn547_dev->ven_gpio, GPIO_OUT_ONE);
								usleep_range(10000, 10000);
								if (atomic_read(&pn547_dev->irq_enabled) == 0) {
										atomic_set(&pn547_dev->irq_enabled, 1);
										mt_eint_unmask(CUST_EINT_IRQ_NFC_NUM);
								}
#endif //PN547_USE_KERNEL_STD
								dev_info(&pn547_dev->client->dev, "%s power on, irq=%d\n", __func__, atomic_read(&pn547_dev->irq_enabled));
						} else if (arg == 0) {
								/* power off */
								
#if defined(PN547_USE_KERNEL_STD)	
								if (atomic_read(&pn547_dev->irq_enabled) == 1) {
										pn547_disable_irq(nfc_irq);
										atomic_set(&pn547_dev->irq_enabled, 0);
								}
								dev_info(&pn547_dev->client->dev, "%s power off, irq=%d\n", __func__, atomic_read(&pn547_dev->irq_enabled));
								ret = mt_nfc_pinctrl_select(gpctrl, st_eint_l);		
								ret = mt_nfc_pinctrl_select(gpctrl, st_ven_l);								
#else
			 					
								if (atomic_read(&pn547_dev->irq_enabled) == 1) {
										mt_eint_mask(CUST_EINT_IRQ_NFC_NUM);
										atomic_set(&pn547_dev->irq_enabled, 0);
								}
								dev_info(&pn547_dev->client->dev, "%s power off, irq=%d\n", __func__, atomic_read(&pn547_dev->irq_enabled));
								mt_set_gpio_out(pn547_dev->firm_gpio, GPIO_OUT_ZERO);
								mt_set_gpio_out(pn547_dev->ven_gpio, GPIO_OUT_ZERO);
								usleep_range(10000, 10000);
#endif
						#ifdef NXP_KR_READ_IRQ_MODIFY
						} else if (arg == 3) {
								pr_info("%s Read Cancle\n", __func__);
								cancle_read = true;
								do_reading = true;
								wake_up(&pn547_dev->read_wq);
						#endif
						} else {
								dev_err(&pn547_dev->client->dev, "%s bad arg %lu\n", __func__, arg);
								return -EINVAL;
						}
						
						return ret;
						
						break;
				default:
						dev_err(&pn547_dev->client->dev, "%s bad ioctl %u\n", __func__,	cmd);
						return -EINVAL;
		}
		
		return 0;
}

static int pn547_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
		int ret;		
		struct pn547_dev *pn547_dev;
		struct pn547_i2c_platform_data *platform_data;
		struct device_node *node;
		
		printk("NFC -> pn547_probe++++\n");
		
		platform_data = &pn547_platform_data;
		if (platform_data == NULL) {
			pr_err("%s : nfc probe fail\n", __func__);
			return  -ENODEV;
		}

		if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
				dev_err(&client->dev, "%s : need I2C_FUNC_I2C\n", __func__);
				return -ENODEV;
		}
		
		
#if defined(PN547_USE_KERNEL_STD)			
	
	ret = mt_nfc_pinctrl_select(gpctrl, st_irq_init);
	usleep_range(900, 1000);

	ret = mt_nfc_pinctrl_select(gpctrl, st_ven_h);
	usleep_range(900, 1000);

	ret = mt_nfc_pinctrl_select(gpctrl, st_eint_l);
#else
		mt_set_gpio_mode(platform_data->irq_gpio, GPIO_IRQ_NFC_PIN_M_EINT);
		mt_set_gpio_dir(platform_data->irq_gpio, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(platform_data->irq_gpio, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(platform_data->irq_gpio, GPIO_PULL_DOWN);

		/* ven_gpio */
		mt_set_gpio_mode(platform_data->ven_gpio, GPIO_NFC_VENB_PIN_M_GPIO);
		mt_set_gpio_dir(platform_data->ven_gpio, GPIO_DIR_OUT);
		mt_set_gpio_out(platform_data->ven_gpio, GPIO_OUT_ONE);	

		/* firm_gpio */
		mt_set_gpio_mode(platform_data->firm_gpio, GPIO_NFC_EINT_PIN_M_GPIO);
		mt_set_gpio_dir(platform_data->firm_gpio, GPIO_DIR_OUT);	
		mt_set_gpio_out(platform_data->firm_gpio, GPIO_OUT_ZERO);	
#endif
	
		pn547_dev = kzalloc(sizeof(*pn547_dev), GFP_KERNEL);
		if (pn547_dev == NULL) {
				dev_err(&client->dev, "failed to allocate memory for module data\n");
				ret = -ENOMEM;
				goto err_exit;
		}
		
		dev_info(&client->dev, "%s : IRQ num %d\n", __func__, client->irq);
		
		pn547_dev->irq_gpio = platform_data->irq_gpio;
		pn547_dev->ven_gpio = platform_data->ven_gpio;
		pn547_dev->firm_gpio = platform_data->firm_gpio;
		pn547_dev->client = client;
		
		/* init mutex and queues */
		init_waitqueue_head(&pn547_dev->read_wq);
		mutex_init(&pn547_dev->read_mutex);
		
		pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
		pn547_dev->pn547_device.name = "pn544";
		pn547_dev->pn547_device.fops = &pn547_dev_fops;
		
		ret = misc_register(&pn547_dev->pn547_device);
		if (ret) {
				dev_err(&client->dev, "%s : misc_register failed. ret = %d\n", __FILE__, ret);
				goto err_misc_register;
		}
		
		i2c_set_clientdata(client, pn547_dev);
		
		wake_lock_init(&pn547_dev->nfc_wake_lock, WAKE_LOCK_SUSPEND, "nfc_wake_lock");

		#ifdef Enable_I2C_DMA

#ifdef CONFIG_64BIT  
		I2CDMAWriteBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa, GFP_KERNEL);
#else
		I2CDMAWriteBuf = (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa, GFP_KERNEL);
#endif
		if (I2CDMAWriteBuf == NULL) {
			dev_err(&client->dev, "%s : failed to allocate dma buffer\n", __func__);
			goto err_request_irq_failed;
		}

#ifdef CONFIG_64BIT 
    I2CDMAReadBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL);
#else
		I2CDMAReadBuf = (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL);
#endif
		if (I2CDMAReadBuf == NULL) {
			dev_err(&client->dev, "%s : failed to allocate dma buffer\n", __func__);
			goto err_request_irq_failed;
		}
        printk(KERN_DEBUG "%s :I2CDMAWriteBuf_pa %d, I2CDMAReadBuf_pa,%d\n", __func__, I2CDMAWriteBuf_pa, I2CDMAReadBuf_pa);
		#endif

		
#if defined(PN547_USE_KERNEL_STD)
		/*  NFC IRQ settings     */
		node = of_find_compatible_node(NULL, NULL, "mediatek, IRQ_NFC-eint");
	
		if (node) {
	
			nfc_irq = irq_of_parse_and_map(node, 0);
	
			client->irq = nfc_irq;
	
			ret =
			    request_irq(nfc_irq, pn547_dev_irq_handler,
					IRQF_TRIGGER_RISING, "irq_nfc-eint", NULL);
	
			if (ret) {
	
				pr_err("%s : EINT IRQ LINE NOT AVAILABLE\n", __func__);
	
			} else {
	
				pr_debug("%s : set EINT finished, nfc_irq=%d", __func__,
					 nfc_irq);
				atomic_set(&pn547_dev->irq_enabled, 0);
				pn547_disable_irq(nfc_irq);
			}
		}	
#else		
		
		/* request irq. the irq is set whenever the chip has data available
		* for reading. it is cleared when all data has been read.
		*/
		client->irq = CUST_EINT_IRQ_NFC_NUM;
		dev_info(&pn547_dev->client->dev, "%s : requesting IRQ %d\n", __func__,	client->irq);
		
		mt_eint_set_hw_debounce(CUST_EINT_IRQ_NFC_NUM, CUST_EINT_IRQ_NFC_DEBOUNCE_CN);
		mt_eint_registration(CUST_EINT_IRQ_NFC_NUM, CUST_EINT_IRQ_NFC_TYPE, pn547_dev_irq_handler, 1);
		mt_eint_mask(CUST_EINT_IRQ_NFC_NUM);
		atomic_set(&pn547_dev->irq_enabled, 0);
#endif
		
		printk("NFC -> pn547_probe----\n");
		
		return 0;
		
		err_request_irq_failed:
				wake_lock_destroy(&pn547_dev->nfc_wake_lock);
				misc_deregister(&pn547_dev->pn547_device);
		err_misc_register:
				mutex_destroy(&pn547_dev->read_mutex);
				kfree(pn547_dev);
		err_exit:
				gpio_free(platform_data->firm_gpio);
		//err_firm:
				gpio_free(platform_data->ven_gpio);
		//err_ven:
				gpio_free(platform_data->irq_gpio);
		return ret;
}

static int pn547_remove(struct i2c_client *client)
{
		struct pn547_dev *pn547_dev;

		#ifdef Enable_I2C_DMA
		if (I2CDMAWriteBuf) {
#ifdef CONFIG_64BIT
			dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAWriteBuf, I2CDMAWriteBuf_pa);
#else
			dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf, I2CDMAWriteBuf_pa);
#endif

			I2CDMAWriteBuf = NULL;
			I2CDMAWriteBuf_pa = 0;
		}

		if (I2CDMAReadBuf) {
#ifdef CONFIG_64BIT
            dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf, I2CDMAReadBuf_pa);
#else
			dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf, I2CDMAReadBuf_pa);
#endif
			I2CDMAReadBuf = NULL;
			I2CDMAReadBuf_pa = 0;
		}
		#endif

		pn547_dev = i2c_get_clientdata(client);
		wake_lock_destroy(&pn547_dev->nfc_wake_lock);
		free_irq(client->irq, pn547_dev);
		misc_deregister(&pn547_dev->pn547_device);
		mutex_destroy(&pn547_dev->read_mutex);
		gpio_free(pn547_dev->irq_gpio);
		gpio_free(pn547_dev->ven_gpio);
		gpio_free(pn547_dev->firm_gpio);
		kfree(pn547_dev);
		
		return 0;
}

/*
* module load/unload record keeping
*/

static int __init pn547_dev_init(void)
{
		pr_info("Loading pn547 driver\n");
   	i2c_register_board_info(NFC_I2C_BUSNUM, &nfc_board_info, 1);
   	
   	platform_driver_register(&mtk_nfc_platform_driver);
   	
		return i2c_add_driver(&pn547_driver);
}
module_init(pn547_dev_init);

static void __exit pn547_dev_exit(void)
{
		pr_info("Unloading pn547 driver\n");
		i2c_del_driver(&pn547_driver);
}
module_exit(pn547_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC pn547 driver");
MODULE_LICENSE("GPL");
