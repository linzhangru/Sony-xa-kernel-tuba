/* ******************************************************************************** */
/*  */
/* LC898212 Step Move header */
/*  */
/* Program Name        : AfSTMV.h */
/* Design                  : Rex.Tang */
/* History                 : First edition                                         2013.07.19 Rex.Tang */
/*  */
/* Description             : Interface Functions and Definations */
/* ******************************************************************************** */
#ifndef __AFSTMV__
#define	__AFSTMV__

/* ************************** */
/* Definations */
/* ************************** */
/* Convergence Judgement */
#define INI_MSSET_212		(unsigned char)0x00	/* Initialize Value For [8Fh] */
#define CHTGX_THRESHOLD		(unsigned short)0x0200	/* Convergence Judge Threshold */
#define CHTGOKN_TIME		(unsigned char)0x80	/* 64 Sampling Time 1.365msec( EQCLK=12MHz ) */
#define CHTGOKN_WAIT		3	/* CHTGOKN_WAIT(3ms) > CHTGOKN_TIME(2.732msec) ( CHTGOKN_WAIT has to be longer than CHTGOKN_TIME) */

/* StepMove */
#define STMV_INTERVAL		(unsigned char)0x00	/* Step Interval(STMVINT) */

#define	STMCHTG_ON			(unsigned char)0x08	/* STMVEN Register Set */
#define STMSV_ON			(unsigned char)0x04
#define STMLFF_ON			(unsigned char)0x02
#define STMVEN_ON			(unsigned char)0x01
#define	STMCHTG_OFF			(unsigned char)0x00
#define STMSV_OFF			(unsigned char)0x00
#define STMLFF_OFF			(unsigned char)0x00
#define STMVEN_OFF			(unsigned char)0x00

#define	STMCHTG_SET			STMCHTG_OFF	/* Convergence Judgement On */
#define STMSV_SET			STMSV_ON	/* Setting Target Position = End Position */
#define STMLFF_SET			STMLFF_OFF

/*====================================================================
	Interface functions (export)
=====================================================================*/
/* Step Move to Target Positon Function */
extern unsigned char Stmv212(short);

/*====================================================================
	Interface functions (import)
=====================================================================*/
extern void RamWriteA(unsigned short addr, unsigned short data);

extern int RamReadA(unsigned short addr, unsigned short *data);

extern void RegWriteA(unsigned short addr, unsigned char data);

extern void RegReadA(unsigned short addr, unsigned char *data);

extern void WaitTime(unsigned short msec);

#endif				/* __AFSTMV__ */
