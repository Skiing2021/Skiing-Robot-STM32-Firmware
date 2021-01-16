#ifndef RC_H
#define RC_H
#include "main.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
#endif
