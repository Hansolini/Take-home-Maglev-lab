

#ifndef COBS_H
#define COBS_H

#include <stdint.h>

void cobsEncode(const uint8_t *ptr, uint8_t length, uint8_t *dst);
void cobsDecode(const uint8_t *ptr, uint8_t length, uint8_t *dst);

#endif