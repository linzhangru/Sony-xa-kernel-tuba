/* ******************************************************************************** */
/*  */
/* LC89821x Initialize Module */
/*  */
/* Program Name        : AfInit.h */
/* Design                  : Rex.Tang */
/* History                 : First edition                                         2013.07.20 Rex.Tang */
/*  */
/* Description             : AF Initialize Functions and Definations */
/* ******************************************************************************** */
/* ************************** */
/* Include Header File */
/* ************************** */
#include	"AfInit.h"
#include	"AfDef.h"
#include <linux/delay.h>

#define AF_DRVNAME "LC898212XDAF_DRV"

#define AF_DEBUG 1
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

struct STHALREG CsHalReg_v01[] = {
	{0x0076, 0x0C},		/*0C,0076,0dB*/
	{0x0077, 0x50},		/*50,0077,30dB*/
	{0x0078, 0x20},		/*20,0078,6dB*/
	{0x0086, 0x40},		/*40,0086*/
	{0x00F0, 0x00},		/*00,00F0,Through,0dB,fs/1,invert=0*/
	{0x00F1, 0x00},		/*00,00F1,LPF,1800Hz,0dB,fs/1,invert=0*/
	{0xFFFF, 0xFF}
};
struct STHALFIL  CsHalFil_v01[] = {
	{0x0030, 0x0000},	/*0000,0030,LPF,1800Hz,0dB,fs/1,invert=0*/
	{0x0040, 0x8010},	/*8010,0040,0dB,invert=1*/
	{0x0042, 0x7790},	/*7790,0042,HBF,25Hz,550Hz,0dB,fs/1,invert=0*/
	{0x0044, 0x8930},	/*8930,0044,HBF,25Hz,550Hz,0dB,fs/1,invert=0*/
	{0x0046, 0x6E70},	/*6E70,0046,HBF,25Hz,550Hz,0dB,fs/1,invert=0*/
	{0x0048, 0x47F0},	/*47F0,0048,-5dB,invert=0*/
	{0x004A, 0x1990},	/*1990,004A,-14dB,invert=0*/
	{0x004C, 0x4030},	/*4030,004C,-6dB,invert=0*/
	{0x004E, 0x7FF0},	/*7FF0,004E,0dB,invert=0*/
	{0x0050, 0x04F0},	/*04F0,0050,LPF,300Hz,0dB,fs/1,invert=0*/
	{0x0052, 0x7610},	/*7610,0052,LPF,300Hz,0dB,fs/1,invert=0*/
	{0x0054, 0x1010},	/*1010,0054,DI,-18dB,fs/16,invert=0*/
	{0x0056, 0x0000},	/*0000,0056,DI,-16dB,fs/16,invert=0*/
	{0x0058, 0x7FF0},	/*7FF0,0058,DI,-16dB,fs/16,invert=0*/
	{0x005A, 0x0680},	/*0680,005A,LPF,400Hz,0dB,fs/1,invert=0*/
	{0x005C, 0x72F0},	/*72F0,005C,LPF,400Hz,0dB,fs/1,invert=0*/
	{0x005E, 0x7F70},	/*7F70,005E,HPF,35Hz,0dB,fs/1,invert=0*/
	{0x0060, 0x7ED0},	/*7ED0,0060,HPF,35Hz,0dB,fs/1,invert=0*/
	{0x0062, 0x7FF0},	/*7FF0,0062,Through,0dB,fs/1,invert=0*/
	{0x0064, 0x0000},	/*0000,0064,Through,0dB,fs/1,invert=0*/
	{0x0066, 0x0000},	/*0000,0066,Through,0dB,fs/1,invert=0*/
	{0x0068, 0x5130},	/*5130,0068,HPF,400Hz,-3.5dB,fs/1,invert=0*/
	{0x006A, 0x72F0},	/*72F0,006A,HPF,400Hz,-3.5dB,fs/1,invert=0*/
	{0x006C, 0x8010},	/*8010,006C,0dB,invert=1*/
	{0x006E, 0x0000},	/*0000,006E,Cutoff,invert=0*/
	{0x0070, 0x0000},	/*0000,0070,Cutoff,invert=0*/
	{0x0072, 0x18E0},	/*18E0,0072,LPF,1800Hz,0dB,fs/1,invert=0*/
	{0x0074, 0x4E30},	/*4E30,0074,LPF,1800Hz,0dB,fs/1,invert=0*/
	{0xFFFF, 0xFFFF}
};

struct STHALREG CsHalReg_v04[] = {
	{0x0076, 0x0C},		/*0C,0076,0dB*/
	{0x0077, 0x50},		/*50,0077,30dB*/
	{0x0078, 0x20},		/*20,0078,6dB*/
	{0x0086, 0x40},		/*40,0086*/
	{0x00F0, 0x00},		/*00,00F0,Through,0dB,fs/1,invert=0*/
	{0x00F1, 0x00},		/*00,00F1,LPF,1800Hz,0dB,fs/1,invert=0*/
	{0xFFFF, 0xFF}
};
struct STHALFIL CsHalFil_v04[] = {
	{0x0030, 0x0000},	/*0000,0030,LPF,1800Hz,0dB,fs/1,invert=0*/
	{0x0040, 0x8010},	/*8010,0040,0dB,invert=1*/
	{0x0042, 0x78E0},	/*78E0,0042,HBF,10Hz,450Hz,0dB,fs/1,invert=0*/
	{0x0044, 0x8770},	/*8770,0044,HBF,10Hz,450Hz,0dB,fs/1,invert=0*/
	{0x0046, 0x7170},	/*7170,0046,HBF,10Hz,450Hz,0dB,fs/1,invert=0*/
	{0x0048, 0x1450},	/*1450,0048,-16dB,invert=0*/
	{0x004A, 0x0E50},	/*0E50,004A,-19dB,invert=0*/
	{0x004C, 0x4030},	/*4030,004C,-6dB,invert=0*/
	{0x004E, 0x7FF0},	/*7FF0,004E,0dB,invert=0*/
	{0x0050, 0x04F0},	/*04F0,0050,LPF,300Hz,0dB,fs/1,invert=0*/
	{0x0052, 0x7610},	/*7610,0052,LPF,300Hz,0dB,fs/1,invert=0*/
	{0x0054, 0x0E50},	/*0E50,0054,DI,-19dB,fs/16,invert=0*/
	{0x0056, 0x0000},	/*0000,0056,DI,-16dB,fs/16,invert=0*/
	{0x0058, 0x7FF0},	/*7FF0,0058,DI,-16dB,fs/16,invert=0*/
	{0x005A, 0x0680},	/*0680,005A,LPF,400Hz,0dB,fs/1,invert=0*/
	{0x005C, 0x72F0},	/*72F0,005C,LPF,400Hz,0dB,fs/1,invert=0*/
	{0x005E, 0x7F70},	/*7F70,005E,HPF,35Hz,0dB,fs/1,invert=0*/
	{0x0060, 0x7ED0},	/*7ED0,0060,HPF,35Hz,0dB,fs/1,invert=0*/
	{0x0062, 0x7FF0},	/*7FF0,0062,Through,0dB,fs/1,invert=0*/
	{0x0064, 0x0000},	/*0000,0064,Through,0dB,fs/1,invert=0*/
	{0x0066, 0x0000},	/*0000,0066,Through,0dB,fs/1,invert=0*/
	{0x0068, 0x5130},	/*5130,0068,HPF,400Hz,-3.5dB,fs/1,invert=0*/
	{0x006A, 0x72F0},	/*72F0,006A,HPF,400Hz,-3.5dB,fs/1,invert=0*/
	{0x006C, 0x8010},	/*8010,006C,0dB,invert=1*/
	{0x006E, 0x0000},	/*0000,006E,Cutoff,invert=0*/
	{0x0070, 0x0000},	/*0000,0070,Cutoff,invert=0*/
	{0x0072, 0x18E0},	/*18E0,0072,LPF,1800Hz,0dB,fs/1,invert=0*/
	{0x0074, 0x4E30},	/*4E30,0074,LPF,1800Hz,0dB,fs/1,invert=0*/
    {0xFFFF, 0xFFFF}
};

void IniFil(void)
{
	unsigned char	UcAryId ;

	struct STHALREG *pHalReg;
	struct STHALFIL *pHalFil;

	unsigned short	UsDatX12;
	unsigned short	UsDatX03;
	unsigned char		UcDatDss;
	unsigned short	UsDatAoe;

	if(version == 0x04){
		pHalReg = (struct STHALREG *)CsHalReg_v04;
		pHalFil = (struct STHALFIL *)CsHalFil_v04;
	}else{
		pHalReg = (struct STHALREG *)CsHalReg_v01;
		pHalFil = (struct STHALFIL *)CsHalFil_v01;
	}

	UsDatX12	= (unsigned short)((unsigned short)(pHalReg[0].UcRegDat) << 8 | (unsigned short)(pHalReg[1].UcRegDat) | (unsigned short)(pHalReg[4].UcRegDat) << 7);
	UsDatX03	= (unsigned short)pHalReg[2].UcRegDat << 8 ;
	RegReadA(DSSEL_212, & UcDatDss) ;				// 0x86
	UcDatDss	= ( UcDatDss & 0x30 ) | pHalReg[3].UcRegDat ;
	UsDatAoe	= (unsigned short)(pHalFil[0].UsRamDat) | ((unsigned short)(pHalReg[5].UcRegDat) << 12 ) ;

	RamWriteA(pHalReg[0].UsRegAdd , UsDatX12);		// 0x76
	RamWriteA(pHalReg[2].UsRegAdd , UsDatX03);		// 0x78
	RegWriteA(pHalReg[3].UsRegAdd , UcDatDss);		// 0x86
	RamWriteA(pHalFil[0].UsRamAdd , UsDatAoe);		// 0x30

	UcAryId	= 1 ;
	while(pHalFil[ UcAryId ].UsRamAdd != 0xFFFF) {
		RamWriteA(pHalFil[ UcAryId ].UsRamAdd,pHalFil[ UcAryId ].UsRamDat) ;		// 0x40 ~ 0x74
		UcAryId++ ;
	}
}

void IniSvo(void)
{
	unsigned short UsRedAof;

	RamReadA(ADOFFSET_212H,	&UsRedAof);		// 0x3C
	RamWriteA(MS1Z22_212H,	UsRedAof);		// 0x18
	RamWriteA(RZ_212H,	UsRedAof);			// 0x04

	WaitTime(1);

	RegWriteA(SWTCH_212, 0x80);				// 0x83
	RegWriteA(ENBL_212, 0x84);				// 0x87
}

void IniDrv(unsigned char hall_off, unsigned char hall_bias)
{
	unsigned char new1=0, new2=0;

	LOG_INF("hall_off = 0x%x, hall_bias = 0x%x\n", hall_off, hall_bias);

	RegWriteA(CLKSEL_212, 0x34);				// 0x80		CLKSEL 1/1, CLKON
	RegWriteA(ADSET_212	, 0xA0);				// 0x81		AD 12bit 4Time

	RegWriteA(STBY_212, 0xE0);			// 0x84		STBY   AD ON,DA ON,OP ON
	RegWriteA(ENBL_212, 0x05);			// 0x87		PIDSW OFF,AF ON,MS2 ON
	RegWriteA(ANA2_212, 0x24);			// 0xA4		Internal OSC Setup (No01=24.18MHz)

	RamWriteA(RZ_212H, 0x0000);				// 0x04		RZ Clear(Target Value)
	RamWriteA(PIDZO_212H, 0x0000);			// 0x02		EQ output Clear
	RamWriteA(MS1Z22_212H, 0x0000);			// 0x18		Set Start Positon

	IniFil();									// 0x76, 0x78, 0x86, 0x30, 0x40~0x74
	RamWriteA(ms11a_212H, 0x0800);				// 0x5A		Set Coefficient Value For StepMove
	RegWriteA(DSSEL_212, 0x60);					// 0x86		DSSEL 1/16 INTON
	RegWriteA(ANA1_212, 0x70);									// 0x88		ANA1   Hall Bias:2mA Amp Gain: x100 for M
	RegWriteA(DAHLXO_212H, hall_off);			// 0x28		Hall Offset/Bias
	RegWriteA(DAHLXO_212L, hall_bias);			// 0x29		Hall Offset/Bias

	if(version == 0x04) {
		RegWriteA(OFFSET_212H, offset_212H);					// 0x3A		Hall Digital Offset MSB
		RegWriteA(OFFSET_212L, offset_212L);					// 0x3B		Hall Digital Offset LSB
		RegReadA(OFFSET_212H, &new1);
		RegReadA(OFFSET_212L, &new2);
		LOG_INF("OFFSET_212H = 0x%x, OFFSET_212L = 0x%x\n", new1, new2);
	}
	else {
		RamWriteA(OFFSET_212H, 0x0000);			// 0x3A		OFFSET Clear
	}

	RamWriteA(gain1_212H, 0x4030);							// 0x4C		Loop Gain

	RegWriteA(SWTCH_212, 0x2C);					// 0x83		RZ OFF,STSW=ms2x2,MS1IN OFF,MS2IN=RZ,FFIN AFTER,DSW ag
	RegWriteA(CLR_212, 0xC0);					// 0x85		AF filter, MS1 Clr
	WaitTime(1);
	RamWriteA(CHTGX_212H, 0x0200);				// 0x7C		Settling Threshold Value
	RegWriteA(PINSEL_212, 0x80);				// 0x93		Settling Time (64 Sampling Time 1.365msec( EQCLK=12MHz ))
	RegWriteA(MSSET_212, 0x00);					// 0x8F		ms12b = -ms12a, ms22b = -ms22a

	RegWriteA(STBY_212, 0xE3);					// 0x84		STBY   AD ON,DA ON,OP ON,DRMODE H,LNSTBB H
	RegWriteA(PWMZONE2_212, 0x00);				// 0x97		DRVSEL
	RegWriteA(PWMZONE1_212, 0x42);				// 0x98		LNFC   1.5MHz 12bit
	RegWriteA(PWMZONE0_212, 0x00);				// 0x99		LNSMTH Smoothing Set
	RegWriteA(ZONE3_212, 0x00);					// 0x9A		LNSAMP

	// StepMove
	RamWriteA(MS1Z12_212H, 0x0060);				// 0x16		Set StepSize
	RegWriteA(STMINT_212, 0x00);				// 0xA0		Set StepInterval
}

/* ******************************************************************************** */
/* Function Name        : ServoON */
/* Retun Value          : NON */
/* Argment Value        : None */
/* Explanation          : Servo On function */
/* History                      : First edition                                         2013.07.20 Rex.Tang */
/* ******************************************************************************** */
void ServoOn(void)
{
	RegWriteA(CLKSEL_212,	0x38);
	RegWriteA(CLKSEL_212,	0x34);
	RegWriteA(SFTRST_212,	0x00);
}
