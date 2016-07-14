/* ******************************************************************************** */
/*  */
/* LC89821x Initialize header */
/*  */
/* Program Name        : AfInit.h */
/* Design                  : Rex.Tang */
/* History                 : First edition                                         2013.07.20 Rex.Tang */
/*  */
/* Description             : Interface Functions and Definations */
/* ******************************************************************************** */
#ifndef	__AFINIT__
#define __AFINIT__

extern unsigned char version;
extern unsigned char offset_212H;
extern unsigned char offset_212L;

struct STCALDAT{
	struct {
		unsigned char	UcHalOff ;				// Hall Offset Value
		unsigned char	UcHalGan ;				// Hall Gain Value
	} StHalAdj ;

	struct {
		unsigned short	UsLpgVal ;				// Loop Gain X
	} StLopGan ;

	unsigned char		UcVerDat ;				// Version Information

	unsigned short		UcDofMsb ;				// Hall Digital Offset MSB 8bit
	unsigned short		UcDofLsb ;				// Hall Digital Offset LSB 8bit

};

struct STHALREG {
	unsigned short UsRegAdd;
	unsigned char	UcRegDat;
};										// Hall Register Data Table

struct STHALFIL {
	unsigned short UsRamAdd;
	unsigned short UsRamDat;
};										// Hall Filter Coefficient Table

extern void IniDrv(unsigned char hall_off, unsigned char hall_bias);
extern void IniSvo(void);
extern void ServoOn(void);

/*====================================================================
	Interface functions (import)
=====================================================================*/
extern void RamWriteA(unsigned short addr, unsigned short data);

extern int RamReadA(unsigned short addr, unsigned short *data);

extern void RegWriteA(unsigned short addr, unsigned char data);

extern void RegReadA(unsigned short addr, unsigned char *data);

extern void WaitTime(unsigned short msec);

#endif				/* __AFINIT__ */
