/*
 * uart.h
 *
 *  Created on: Feb 15, 2020
 *      Author: steveb
 */

#ifndef UART_H_
#define UART_H_

union DR_register
{
  struct DR_register_field
  {
    uint32_t  DATA:8;
    uint32_t  FE:1;
    uint32_t  PI:1;
    uint32_t  BE:1;
    uint32_t  OE:1;
    uint32_t  reserved:20;
  }         field;
  uint32_t  value;
};

union RSRECR_register
{
  struct RSRECR_register_field
  {
    uint32_t  FE:1;
    uint32_t  PE:1;
    uint32_t  BE:1;
    uint32_t  OE:1;
    uint32_t  reserved:28;
  }         field;
  uint32_t  value;
};

union FR_register
{
  struct FR_register_field
  {
    uint32_t  CTS:1;
    uint32_t  DSR:1;
    uint32_t  DCD:1;
    uint32_t  BUSY:1;
    uint32_t  RXFE:1;
    uint32_t  TXFF:1;
    uint32_t  RXFF:1;
    uint32_t  TXFE:1;
    uint32_t  RI:1;
    uint32_t  reserved:23;
  }         field;
  uint32_t  value;
};

union ILPR_register
{
  struct ILPR_register_field
  {
    uint32_t  ILPR:32;
  }         field;
  uint32_t  value;
};

union IBRD_register
{
  struct IBRD_register_field
  {
    uint32_t  IBRD:16;    /* BAUDDIV=(FUARTCLK/(16 Baud rate)) */
    uint32_t  reserved:16;
  }         field;
  uint32_t  value;
};

union FBRD_register
{
  struct FBRD_register_field
  {
    uint32_t  FBRD:6;
    uint32_t  reserved:26;
  }         field;
  uint32_t  value;
};

union LCRH_register
{
  struct LCRH_register_field
  {
    uint32_t  BRK:1;
    uint32_t  PEN:1;
    uint32_t  EPS:1;
    uint32_t  STP2:1;
    uint32_t  FEN:1;
    uint32_t  WLEN:2; /* 0=5bit, 1=6bit, 2=7bit, 3=8bit */
    uint32_t  SPS:1;
    uint32_t  reserved:24;
  }         field;
  uint32_t  value;
};

union CR_register
{
  struct CR_register_field
  {
    uint32_t  UARTEN:1;
    uint32_t  SIREN:1;
    uint32_t  SIRLP:1;
    uint32_t  reserved0:4;
    uint32_t  LBE:1;
    uint32_t  TXE:1;
    uint32_t  RXE:1;
    uint32_t  DTR:1;
    uint32_t  RTS:1;
    uint32_t  OUT1:1;
    uint32_t  OUT2:1;
    uint32_t  RSEN:1;
    uint32_t  CTSEN:1;
    uint32_t  reserved1:16;
  }         field;
  uint32_t  value;
};

union IFLS_register
{
  struct IFLS_register_field
  {
    uint32_t  TXIFLSEL:3;
    uint32_t  RXIFLSEL:3;
    uint32_t  TXIFPSEL:3;
    uint32_t  RXIFPSEL:3;
    uint32_t  reserved:20;
  }         field;
  uint32_t  value;
};

union IMSC_register
{
  struct IMSC_register_field
  {
    uint32_t  RIMIM:1;
    uint32_t  CTSMIM:1;
    uint32_t  DCDMIM:1;
    uint32_t  DSRMIM:1;
    uint32_t  RXIM:1;
    uint32_t  TXIM:1;
    uint32_t  RTIM:1;
    uint32_t  FEIM:1;
    uint32_t  PEIM:1;
    uint32_t  BEIM:1;
    uint32_t  OEIM:1;
    uint32_t  reserved:21;
  }         field;
  uint32_t  value;
};

union RIS_register
{
  struct RIS_register_field
  {
    uint32_t  RIRMIS:1;
    uint32_t  CTRMIS:1;
    uint32_t  DCDRMIS:1;
    uint32_t  DSRRMIS:1;
    uint32_t  RXRIS:1;
    uint32_t  TXRIS:1;
    uint32_t  RTRIS:1;
    uint32_t  FERIS:1;
    uint32_t  PERIS:1;
    uint32_t  BERIS:1;
    uint32_t  OERIS:1;
    uint32_t  reserved:21;
  }         field;
  uint32_t  value;
};

union MIS_register
{
  struct MIS_register_field
  {
    uint32_t  RIMMIS:1;
    uint32_t  CTSMMIS:1;
    uint32_t  DCDMMIS:1;
    uint32_t  DSRMMIS:1;
    uint32_t  RXMIS:1;
    uint32_t  TXMIS:1;
    uint32_t  RTMIS:1;
    uint32_t  FEMIS:1;
    uint32_t  PEMIS:1;
    uint32_t  BEMIS:1;
    uint32_t  OEMIS:1;
    uint32_t  reserved:21;
  }         field;
  uint32_t  value;
};

union ICR_register
{
  struct ICR_register_field
  {
    uint32_t  RIMIC:1;
    uint32_t  CTSMIC:1;
    uint32_t  DCDMIC:1;
    uint32_t  DSRMIC:1;
    uint32_t  RXIC:1;
    uint32_t  TXIC:1;
    uint32_t  RTIC:1;
    uint32_t  FEIC:1;
    uint32_t  PEIC:1;
    uint32_t  BEIC:1;
    uint32_t  OEIC:1;
    uint32_t  reserved:21;
  }         field;
  uint32_t  value;
};

union DMACR_register
{
  struct DMACR_register_field
  {
    uint32_t  RXDMAE:1;
    uint32_t  TXDMAE:1;
    uint32_t  DMAONERR:1;
    uint32_t  reserved:29;
  }         field;
  uint32_t  value;
};

union ITCR_register
{
  struct ITCR_register_field
  {
    uint32_t  ITCR0:1;
    uint32_t  ITCR1:1;
    uint32_t  reserved:30;
  }         field;
  uint32_t  value;
};

union ITIP_register
{
  struct ITIP_register_field
  {
    uint32_t  ITIP0:1;
    uint32_t  reserved0:2;
    uint32_t  ITIP3:1;
    uint32_t  reserved1:28;
  }         field;
  uint32_t  value;
};

union ITOP_register
{
  struct ITOP_register_field
  {
    uint32_t  ITIP0:1;
    uint32_t  reserved0:2;
    uint32_t  ITIP3:1;
    uint32_t  reserved1:2;
    uint32_t  ITIP6:1;
    uint32_t  ITOP7:1;
    uint32_t  ITOP8:1;
    uint32_t  ITOP9:1;
    uint32_t  ITOP10:1;
    uint32_t  ITOP11:1;
    uint32_t  reserved2:20;
  }         field;
  uint32_t  value;
};

union TDR_register
{
  struct TDR_register_field
  {
    uint32_t  TDR10_0:11;
    uint32_t  reserved:21;
  }         field;
  uint32_t  value;
};

struct uart_register
{
  union DR_register     DR;
  union RSRECR_register RSRECR;
  uint8_t               unused0[0x18-(0x4+sizeof(union RSRECR_register))];
  union FR_register     FR;
  uint8_t               unused1[0x20-(0x18+sizeof(union FR_register))];
  union ILPR_register   ILPR;
  union IBRD_register   IBRD;
  union FBRD_register   FBRD;
  union LCRH_register   LCRH;
  union CR_register     CR;
  union IFLS_register   IFLS;
  union IMSC_register   IMSC;
  union RIS_register    RIS;
  union MIS_register    MIS;
  union ICR_register    ICR;
  union DMACR_register  DMACR;
  uint8_t               unused2[0x80-(0x48+sizeof(union DMACR_register))];
  union ITCR_register   ITCR;
  union ITIP_register   ITIP;
  union ITOP_register   ITOP;
  union TDR_register    TDR;
};

#endif /* UART_H_ */
