/*
 * LC898212XDAF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"

#include "AfDef.h"
#include "AfInit.h"
#include "AfSTMV.h"

#define AF_DRVNAME "LC898212XDAF_DRV"
#define AF_I2C_SLAVE_ADDR                0xE4
#define E2PROM_WRITE_ID 0xA0

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened, startFlag = 0;
static spinlock_t *g_pAF_SpinLock;
struct STCALDAT StCalDat;

static unsigned long g_u4AF_INF = 0;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static long g_i4MotorStatus;

static int Min_Pos;
static int Max_Pos;

static unsigned short Hall_Max = 0x0000;	/* Please read INF position from EEPROM or OTP */
static unsigned short Hall_Min = 0x0000;	/* Please read MACRO position from EEPROM or OTP */
static unsigned char Hall_Off = 0x00;	/* Please Read Offset from EEPROM or OTP */
static unsigned char Hall_Bias = 0x00;	/* Please Read Bias from EEPROM or OTP */

unsigned char version;
unsigned char offset_212H;
unsigned char offset_212L;

/*******************************************************************************
* WriteRegI2C
********************************************************************************/
int LC898212XD_WriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId)
{
	int i4RetValue = 0;
	int retry = 3;

	g_pstAF_I2Cclient->addr = (i2cId >> 1);
	//LOG_INF(" Write id = 0x%x\n", g_pstAF_I2Cclient->addr);
	g_pstAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;

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

/*******************************************************************************
* ReadRegI2C
********************************************************************************/
int LC898212XD_ReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData,
			  u16 a_sizeRecvData, u16 i2cId)
{
	int i4RetValue = 0;

	g_pstAF_I2Cclient->addr = (i2cId >> 1);
	//LOG_INF(" Read id = 0x%x\n", g_pstAF_I2Cclient->addr);

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

static int E2PROMReadA(unsigned short addr, u8 *data)
{
	int ret = 0;

	u8 puSendCmd[2] = { (u8) (addr >> 8), (u8) (addr & 0xFF) };

	ret = LC898212XD_ReadRegI2C(puSendCmd, sizeof(puSendCmd), data, 1, E2PROM_WRITE_ID);
	if (ret < 0) {
		LOG_INF(" I2C read e2prom failed!!\n");
		return ret;
	}
	else {
		return ret;
	}
}

static unsigned long convertAF_DAC(short ReadData)
{
	int trans, pos;

	if(version == 0x04) {
		if((ReadData > 47536) && (ReadData < 65536))
			trans = ReadData - 65536;
		else
			trans = ReadData;
	}
	else {
		if((ReadData > 59391) && (ReadData < 65536))
			trans = ReadData - 65536;
		else
			trans = ReadData;
	}

	pos = (trans-(Min_Pos))*(1023-0)/(Max_Pos-Min_Pos)+0;

	return pos;
}

static void getMaxMin_pos(void)
{
	E2PROMReadA(0x0E06, (unsigned char *)&StCalDat.UcVerDat);
	WaitTime(5);
	version = StCalDat.UcVerDat;
	LOG_INF("version = %d (0x%x)\n", version, StCalDat.UcVerDat);
	if(version == 0x04) {
		Max_Pos = 18000;
		Min_Pos = -18000;
	}
	else {
		Max_Pos = 6144;
		Min_Pos = -6144;
	}
}
/*
static void getMacInf_pos(void)
{
	unsigned char val1 = 0x00, val2 = 0x00;
	int ret1 = 0, ret2 = 0;

	ret1 = E2PROMReadA(0x12, (unsigned char *)&val1);
	udelay(1000);
	ret2 = E2PROMReadA(0x11, (unsigned char *)&val2);
	udelay(1000);
	if((ret1+ret2) == 0) {
		g_u4AF_INF = convertAF_DAC(((val1 << 8) | (val2 & 0x00FF)) & 0xFFFF);
		LOG_INF("val1: 0x%x, val2: 0x%x, g_u4AF_INF = %ld", val1, val2, g_u4AF_INF);
	}
	else {
		g_u4AF_INF = 0;
	}

	ret1 = ret2 = 0;
	ret1 = E2PROMReadA(0x14, (unsigned char *)&val1);
	udelay(1000);
	ret2 = E2PROMReadA(0x13, (unsigned char *)&val2);
	udelay(1000);
	if((ret1+ret2) == 0) {
		g_u4AF_MACRO = convertAF_DAC(((val1 << 8) | (val2 & 0x00FF)) & 0xFFFF);
		LOG_INF("val1: 0x%x, val2: 0x%x, g_u4AF_MACRO = %ld", val1, val2, g_u4AF_MACRO);
	}
	else {
		g_u4AF_MACRO = 1023; 
	}
}
*/
static void LC898212XD_init(void)
{
//	unsigned char val1, val2;

	LOG_INF("Init - Start\n");
/*
	E2PROMReadA(0x0E01, (unsigned char *)&val1);
	WaitTime(5);
	E2PROMReadA(0x0E00, (unsigned char *)&val2);
	WaitTime(5);
	Hall_Min = ((val1 << 8) | (val2 & 0x00FF)) & 0xFFFF;

	E2PROMReadA(0x0E03, (unsigned char *)&val1);
	WaitTime(5);
	E2PROMReadA(0x0E02, (unsigned char *)&val2);
	WaitTime(5);
	Hall_Max = ((val1 << 8) | (val2 & 0x00FF)) & 0xFFFF;
*/
	E2PROMReadA(0x0E04, (unsigned char *)&StCalDat.StHalAdj.UcHalOff);
	WaitTime(5);
	E2PROMReadA(0x0E05, (unsigned char *)&StCalDat.StHalAdj.UcHalGan);
	WaitTime(5);
	Hall_Off = StCalDat.StHalAdj.UcHalOff;
	Hall_Bias = StCalDat.StHalAdj.UcHalGan;

	if(version == 0x04) {
		E2PROMReadA(0x0E07,(unsigned char *)&StCalDat.UcDofLsb);
		WaitTime(5);
		E2PROMReadA(0x0E08,(unsigned char *)&StCalDat.UcDofMsb);
		WaitTime(5);
		offset_212H = StCalDat.UcDofMsb;
		offset_212L = StCalDat.UcDofLsb;
		LOG_INF("OFFSET_212H = 0x%x, OFFSET_212L = 0x%x\n", offset_212H, offset_212L);
	}

	LOG_INF("=====LC898212XD:=init=hall_max:0x%x==hall_min:0x%x====hall_off:0x%x, hall_bias:0x%x===\n",
	     Hall_Max, Hall_Min, Hall_Off, Hall_Bias);

	ServoOn();		/* Close loop ON */

	spin_lock(g_pAF_SpinLock);
	*g_pAF_Opened = 1;
	spin_unlock(g_pAF_SpinLock);

	LOG_INF("Init - End\n");
}

static unsigned short AF_convert(int position)
{
	int targetCode, result;
	targetCode = ((((position - 0) * (Max_Pos - Min_Pos) / (1023 - 0)) + Min_Pos));

	if (targetCode < 0) {
		result = 65536 + targetCode;
		LOG_INF("targetCode < 0\n");
	}
	else {
		result = targetCode;
		LOG_INF("targetCode >= 0\n");
	}

	LOG_INF("result = %d, 0x%04x\n", result, result);
	return result;
}

static inline int getAFInfo(__user stAF_MotorInfo *pstMotorInfo)
{
	stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;

	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	if (g_i4MotorStatus == 1)
		stMotorInfo.bIsMotorMoving = 1;
	else
		stMotorInfo.bIsMotorMoving = 0;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		u16 InitDAC, InitPos;
		/* Driver Init */
		LC898212XD_init();

		IniDrv(Hall_Off, Hall_Bias);	/* Initialize driver IC */
		IniSvo();
		WaitTime(1);
		ret = RamReadA(RZ_212H, &InitDAC);
		WaitTime(5);
		if (ret == 0) {
			if(version == 0x04) {
				if ((InitDAC > 47536) && (InitDAC < 65536)) {
					InitPos = convertAF_DAC(InitDAC);
					LOG_INF("Init DAC %d (0x%04x)\n", (InitDAC-65536), InitDAC);
					LOG_INF("Init POS %d\n", InitPos);
				}
				else if (InitDAC > 65536)
					LOG_INF("Init DAC incorrect...\n");
				else {
					InitPos = convertAF_DAC(InitDAC);
					LOG_INF("Init DAC %d (0x%04x)\n", InitDAC, InitDAC);
					LOG_INF("Init POS %d\n", InitPos);
				}
			}
			else {
				if ((InitDAC > 59391) && (InitDAC < 65536)) {
					InitPos = convertAF_DAC(InitDAC);
					LOG_INF("Init DAC %d (0x%04x)\n", (InitDAC-65536), InitDAC);
					LOG_INF("Init POS %d\n", InitPos);
				}
				else if (InitDAC > 65536)
					LOG_INF("Init DAC incorrect...\n");
				else {
					InitPos = convertAF_DAC(InitDAC);
					LOG_INF("Init DAC %d (0x%04x)\n", InitDAC, InitDAC);
					LOG_INF("Init POS %d\n", InitPos);
				}
			}
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);
		}
		else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		startFlag = 1;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	LOG_INF("move [curr] %ld [target] %ld\n", g_u4CurrPosition, g_u4TargetPosition);

	spin_lock(g_pAF_SpinLock);
	g_i4MotorStatus = 0;
	spin_unlock(g_pAF_SpinLock);

	if (Stmv212(AF_convert(a_u4Position)) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");

		spin_lock(g_pAF_SpinLock);
		g_i4MotorStatus = -1;
		spin_unlock(g_pAF_SpinLock);
	}

	return 0;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static void setStandby(void)
{
	RegWriteA(CLKSEL_212,	0x34);
	RegWriteA(SFTRST_212,	0x80);
	RegWriteA(CLKSEL_212,	0x38);
	RegWriteA(CLKSEL_212,	0x39);
}

/* ////////////////////////////////////////////////////////////// */
long LC898212XDAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		if(startFlag == 0) {
			getMaxMin_pos();
//			getMacInf_pos();
		}
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int LC898212XDAF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	int count = 0;

	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		while (count != 3) {
			moveAF((100+g_u4AF_INF));
			WaitTime(5);
			moveAF((50+g_u4AF_INF));
			WaitTime(5);
			if(g_i4MotorStatus == -1)
				count++;
			else
				break;
		}
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		startFlag = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	setStandby();

	LOG_INF("End\n");

	return 0;
}

void LC898212XDAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
}
