/**
  * @file cobs.h
  * @brief Header file for cobs.c
  *
  * @author
  *   Riccardo Antonello (riccardo.antonello@unipd.it)
  *   Dept. of Information Engineering, University of Padova
  *
  * @date 15 December 2017
  */

/*  Function prototypes  */

#ifndef COBS_H
#define COBS_H

#include <stdint.h>

void cobsEncode(const uint8_t *ptr, uint8_t length, uint8_t *dst);
void cobsDecode(const uint8_t *ptr, uint8_t length, uint8_t *dst);

#endif