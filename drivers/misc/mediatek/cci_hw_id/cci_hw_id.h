#ifndef __CCI_HW_ID_H
#define __CCI_HW_ID_H

/* Implement CCI HW ID/PROJECT ID */

#define SET_MODEL_TYPE_1    0x01
#define SET_MODEL_TYPE_2    0x02
#define SET_MODEL_TYPE_3    0x04

#define SET_MODEL_TYPE_4    0x08
#define SET_MODEL_TYPE_5    0x10
#define SET_MODEL_TYPE_6    0x20


extern int get_cci_hw_id(void);
extern char* get_cci_phase_name(void);

#endif /* __CCI_HW_ID_H */
