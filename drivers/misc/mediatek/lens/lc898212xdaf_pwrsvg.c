#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include	"lc898212xdaf_AfDef.h"

#define LC898212XDAF_I2C_SLAVE_ADDR        0xE4
#define DRVNAME "LC898212XDAF_PWRSVG"

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static struct i2c_client *g_pstAF_I2Cclient;

static int LC898212XD_WriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData)
{
	int i4RetValue = 0;
	int retry = 3;

	do {
		i4RetValue =
		    i2c_master_send(g_pstAF_I2Cclient, a_pSendData, a_sizeSendData);
		if (i4RetValue != a_sizeSendData) {
			LOG_INF(" I2C send failed!!, Addr = 0x%x, Data = 0x%x\n",
				     a_pSendData[0], a_pSendData[1]);
		} else {
			break;
		}
		udelay(50);
	} while ((retry--) > 0);

	return 0;
}

static int LC898212XD_ReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData,
			  u16 a_sizeRecvData)
{
	int i4RetValue = 0;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, a_pSendData, a_sizeSendData);
	if (i4RetValue != a_sizeSendData) {
		LOG_INF(" I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, (u8 *) a_pRecvData, a_sizeRecvData);
	if (i4RetValue != a_sizeRecvData) {
		LOG_INF(" I2C read failed!!\n");
		return -1;
	}

	return 0;
}

static int RegWriteA(unsigned short addr, unsigned char data) {
	int i4RetValue = 0;
	u8 puSendCmd[2] = { (u8) (addr & 0xFF), (u8) (data & 0xFF) };

	i4RetValue = LC898212XD_WriteRegI2C(puSendCmd, sizeof(puSendCmd));
	if (i4RetValue == 0)
		return 0;
	else
		return -1;
}

static int RegReadA(unsigned short addr, unsigned char *data) {
	int i4RetValue = 0;
	u8 puSendCmd[1] = { (u8) (addr & 0xFF) };

	i4RetValue = LC898212XD_ReadRegI2C(puSendCmd, sizeof(puSendCmd), data, 1);
	if (i4RetValue == 0)
		return 0;
	else
		return -1;
}

void LC898212XDAF_powerSaving(struct i2c_client *client) {
	unsigned char regValue = 0;

	g_pstAF_I2Cclient = client;
	g_pstAF_I2Cclient->addr = LC898212XDAF_I2C_SLAVE_ADDR >> 1;
	g_pstAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;

	msleep(1);

	RegWriteA(CLKSEL_212,	0x34);
	RegWriteA(SFTRST_212,	0x80);
	RegWriteA(CLKSEL_212,	0x38);
	RegWriteA(CLKSEL_212,	0x39);

	RegReadA(STBY_212, &regValue);
	printk("LC898212XDAF_PWRSVG STBY_212 regValue = 0x%x\n", regValue);
	RegReadA(ENBL_212, &regValue);
	printk("LC898212XDAF_PWRSVG ENBL_212 regValue = 0x%x\n", regValue);
	RegReadA(SFTRST_212, &regValue);
	printk("LC898212XDAF_PWRSVG SFTRST_212 regValue = 0x%x\n", regValue);
	RegReadA(CLKSEL_212, &regValue);
	printk("LC898212XDAF_PWRSVG CLKSEL_212 regValue = 0x%x\n", regValue);
}
