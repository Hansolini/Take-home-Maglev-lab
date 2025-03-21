
#include "cobs.h"


/*	COBS encoding routine  */
#define FinishBlock(X) (*code_ptr = (X), code_ptr = dst++, code = 0x01)

void cobsEncode(const uint8_t *ptr, uint8_t length, uint8_t *dst)
{
    const uint8_t *end = ptr + length;
    uint8_t *code_ptr = dst++;
    uint8_t code = 0x01;
    
    while (ptr < end)
    {
        if (*ptr == 0)
            FinishBlock(code);
        else
        {
            *dst++ = *ptr;
            if (++code == 0xFF)
                FinishBlock(code);
        }
        ptr++;
    }
    
    *dst = 0x00; // if put after the next instruction, it doesn't always work (it would modify the memory cell right after the last one)
    FinishBlock(code);
}


/*	COBS decoding routine */
void cobsDecode(const uint8_t *ptr, uint8_t length, uint8_t *dst)
{
  const uint8_t *end = ptr + length;
  int i, code;

  while (ptr < end)
  {
    code = *ptr++;

    for (i = 1; i < code; i++)
      *dst++ = *ptr++;

    if (code < 0xFF)
      *dst++ = 0;
  }
}
