/*
 * io_peripherals.h
 *
 *  Created on:      Feb.   3, 2018
 *  Last updated on: March  4, 2022
 *      Author: steveb
 */

#ifndef IO_PERIPHERALS_H_
#define IO_PERIPHERALS_H_

#if 0
#define PHYSICAL_ADDRESS 0x7E000000 /* base for BCM2835, PI1 */
#define PERIPHERAL_CLOCK 250000000
#elif 0
#define PHYSICAL_ADDRESS 0x20000000 /* base for BCM2708, PI2 */
#define PERIPHERAL_CLOCK 333333333
#elif 1
#define PHYSICAL_ADDRESS 0x3F000000 /* base for BCM2709, PI3 */
#define PERIPHERAL_CLOCK 50000000
#define FUART_CLOCK 48000000
#else
#define PHYSICAL_ADDRESS 0xFE000000 /* base for BCM2711, PI4B */
#define PERIPHERAL_CLOCK 50000000   /* peripheral clock frequency = AXI clock frequency */
#define FUART_CLOCK 48000000
#endif
#define IO_REGISTERS ((volatile struct io_peripherals *)PHYSICAL_ADDRESS)

struct pcm_register
{
  uint8_t unused; /* empty structure */
};

struct io_peripherals
{
  volatile struct cm_register *cm;     /* offset = 0x101000, width = 0xA8 */
  volatile struct gpio_register *gpio; /* offset = 0x200000, width = 0x84 */
  volatile struct uart_register *uart; /* offset - 0x201000, width = 0x90 */
  volatile struct pcm_register *pcm;   /* offset = 0x203000, width = 0x24 */
  volatile struct spi_register *spi;   /* offset = 0x204000, width = 0x18 */
  volatile struct pwm_register *pwm;   /* offset = 0x20c000, width = 0x28 */
  volatile struct bsc_register *bsc;   /* offset = 0x804000, width = 0x20 */
};

#endif /* IO_PERIPHERALS_H_ */
