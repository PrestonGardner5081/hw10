/*
 * bsc.h
 *
 *  Created on: Apr 24, 2021
 *      Author: steveb
 */

#ifndef BSC_H_
#define BSC_H_

union BSC_C_register
{
  struct BSC_C_register_field
  {
    uint32_t  READ:1;
    uint32_t  reserved0:3;
    uint32_t  CLEAR:2;
    uint32_t  reserved1:1;
    uint32_t  ST:1;
    uint32_t  INTD:1;
    uint32_t  INTT:1;
    uint32_t  INTR:1;
    uint32_t  reserved2:4;
    uint32_t  I2CEN:1;
    uint32_t  reserved3:16;
  }         field;
  uint32_t  value;
};

union BSC_S_register
{
  struct BSC_S_register_field
  {
    uint32_t  TA:1;
    uint32_t  DONE:1;
    uint32_t  TXW:1;
    uint32_t  RXR:1;
    uint32_t  TXD:1;
    uint32_t  RXD:1;
    uint32_t  TXE:1;
    uint32_t  RXF:1;
    uint32_t  ERR:1;
    uint32_t  CLKT:1;
    uint32_t  reserved:22;
  }         field;
  uint32_t  value;
};

union BSC_DLEN_register
{
  struct BSC_DLEN_register_field
  {
    uint32_t  DLEN:16;
    uint32_t  reserved:16;
  }         field;
  uint32_t  value;
};

union BSC_A_register
{
  struct BSC_A_register_field
  {
    uint32_t  ADDR:7;
    uint32_t  reserved:25;
  }         field;
  uint32_t  value;
};

union BSC_FIFO_register
{
  struct BSC_FIFO_register_field
  {
    uint32_t  DATA:8;
    uint32_t  reserved:24;
  }         field;
  uint32_t  value;
};

union BSC_DIV_register
{
  struct BSC_DIV_register_field
  {
    uint32_t  CDIV:16;
    uint32_t  reserved:16;
  }         field;
  uint32_t  value;
};

union BSC_DEL_register
{
  struct BSC_DEL_register_field
  {
    uint32_t  REDL:16;
    uint32_t  FEDL:16;
  }         field;
  uint32_t  value;
};

union BSC_CLKT_register
{
  struct BSC_CLKT_register_field
  {
    uint32_t  TOUT:16;
    uint32_t  reserved:16;
  }         field;
  uint32_t  value;
};

struct bsc_register
{
  union BSC_C_register    C;
  union BSC_S_register    S;
  union BSC_DLEN_register DLEN;
  union BSC_A_register    A;
  union BSC_FIFO_register FIFO;
  union BSC_DIV_register  DIV;
  union BSC_DEL_register  DEL;
  union BSC_CLKT_register CLKT;
};

#endif /* BSC_H_ */
