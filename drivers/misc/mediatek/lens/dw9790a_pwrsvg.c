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

#define DW9790A_I2C_SLAVE_ADDR        0x18
#define DRVNAME "DW9790A_PWRSVG"

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static struct i2c_client *g_pstAF_I2Cclient;

static int DW9790A_ReadReg(u16 a_u2Addr, unsigned short *a_pu2Result)
{
	char pBuff, puSendCmd[1] = { (char)a_u2Addr };

	if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 1) != 1) {
		LOG_INF(" DW9790A_PWRSVG I2C read failed(in send)!!\n");
		return -1;
	}

	if (i2c_master_recv(g_pstAF_I2Cclient, &pBuff, 1) != 1) {
		LOG_INF(" DW9790A_PWRSVG I2C read failed!!\n");
		return -1;
	}
	*a_pu2Result = pBuff;
	return 0;
}

static int DW9790A_WriteReg(u16 a_u2Addr, u16 a_u2Data)
{
	char puSendCmd[2] = { (char)a_u2Addr, (char)a_u2Data };

	if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2) < 0) {
		LOG_INF(" DW9790A_PWRSVG I2C send failed!!\n");
		return -1;
	}

	return 0;
}

void DW9790A_powerSaving(struct i2c_client *client) {
	int i4RetValue = 0, retry = 0;
	unsigned short regValue = 0;

	g_pstAF_I2Cclient = client;
	g_pstAF_I2Cclient->addr = DW9790A_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
	g_pstAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;

	while(1) {
		i4RetValue = DW9790A_WriteReg(0x02, 0x20);	//switch to sleep mode
		LOG_INF(" i4RetValue = %d\n", i4RetValue);
		retry++;
		if(i4RetValue == 0)
			break;
		if(retry == 3)
			break;
	}
	DW9790A_ReadReg(0x02, &regValue);
	printk("DW9790A_PWRSVG regValue = 0x%x\n", regValue);
}
