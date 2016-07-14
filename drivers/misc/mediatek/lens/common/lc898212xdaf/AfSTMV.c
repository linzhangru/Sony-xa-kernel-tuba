/* ******************************************************************************** */
/*  */
/* << LC89821x Step Move module >> */
/* Program Name        : AfSTMV.c */
/* Design                  : Y.Yamada */
/* History                 : First edition                                         2009.07.31 Y.Tashita */
/* History                 : LC898211 changes                                      2012.06.11 YS.Kim */
/* History                 : LC898212 changes                                      2013.07.19 Rex.Tang */
/* ******************************************************************************** */
/* ************************** */
/* Include Header File */
/* ************************** */
#include <linux/delay.h>

#include	"AfInit.h"
#include	"AfSTMV.h"
#include	"AfDef.h"

/* ************************** */
/* Definations */
/* ************************** */
#define	LC898212_fs	234375
#define AF_DRVNAME "LC898212XDAF_DRV"

#define AF_DEBUG 1
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

/*--------------------------
    Local defination
---------------------------*/

//********************************************************************************
// Step Move
//********************************************************************************
//********************************************************************************
// Function Name 	: Stmv212
// Return Value		: Stepmove result(1:Need Retry , 0:OK)
// Argument Value	: Target Position
// Explanation		: Stepmove Function
// History			: First edition 						2015.02.10 YS.Kim
//********************************************************************************
unsigned char Stmv212(short SsSmvEnd)
{
	unsigned char	UcStmOpe;
	//unsigned short var;
	unsigned short stmv_size;
	short	SsParStt;

	RegReadA(STMVEN_212, &UcStmOpe);								// 0x8A
	if( UcStmOpe & (unsigned char)STMVEN_ON ){					// Step Move Operating Check
		LOG_INF(" Lens Operating now...\n");
		return 1;													// Step Move Operating Now
	}

	if(version == 0x04) {
		stmv_size=0x0028;
	}
	else {
		stmv_size=0x0060;
	}

	RamReadA(RZ_212H, (unsigned short *)&SsParStt);			// 0x04	Get Start Position
	if( SsParStt < SsSmvEnd ){									// Check StepMove Direction
		RamWriteA(MS1Z12_212H, stmv_size);						// 0x16 Set Direction positive
		//LOG_INF(" Stmv212 write pos_dir\n");
	} else if( SsParStt > SsSmvEnd ){
		RamWriteA(MS1Z12_212H, -stmv_size);						// 0x16 Set Direction negative
		//LOG_INF(" Stmv212 write neg_dir\n");
	}

	RamWriteA(STMVENDH_212, SsSmvEnd );							// 0xA1 Set StepMove Target Position
	RegWriteA(STMVEN_212, 0x05 );								// 0x8A Start StepMove

	//RamReadA(STMVENDH_212, &var);
	//LOG_INF(" Stmv212 set Position to 0x%04x\n", var);

	return 0;
}
