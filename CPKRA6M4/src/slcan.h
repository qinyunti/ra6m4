#ifndef SLCAN_H
#define SLCAN_H

#define SLCAN_MTU 30 // (sizeof("T1111222281122334455667788EA5F\r")+1)

int8_t slcan_parse_frame(can_msg_st *msg, uint8_t* buf);
int8_t slcan_parse_str(uint8_t *buf, uint8_t len);

#endif
