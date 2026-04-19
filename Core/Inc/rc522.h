#ifndef __RC522_H__
#define __RC522_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void RC522_Init(void);
uint8_t RC522_ReadUid(uint8_t uid[4]);

#ifdef __cplusplus
}
#endif

#endif /* __RC522_H__ */
