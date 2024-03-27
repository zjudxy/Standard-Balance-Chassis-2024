#ifndef _HW_CAN_H_
#define _HW_CAN_H_
/* ------------------------------ Include ------------------------------ */
#include "system.h"
/* ------------------------------ Macro Definition ------------------------------ */



/* ------------------------------ Type Definition ------------------------------ */



/* ------------------------------ Extern Global Variable ------------------------------ */



/* ------------------------------ Function Declaration (used in other .c files) ------------------------------ */

void CanInit(void);
void FDCAN_Send_Msg(FDCAN_HandleTypeDef* hfdcan, uint8_t* msg, uint32_t id, uint8_t len);
#endif
