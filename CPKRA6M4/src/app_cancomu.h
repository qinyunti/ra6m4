
#ifndef APP_CAN_H
#define APP_CAN_H

#include "bsp_can.h"
void app_cancomu_init(void);
int can_tx(can_id_e id,can_msg_st* msg,int32_t ms);

#endif
