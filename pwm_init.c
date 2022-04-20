#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "io_peripherals.h"
#include "pwm_init.h"

void pwm_init(struct io_peripherals *io, int range)
{
    io->pwm->RNG1 = range;        /* the range value, 100 level steps */
    io->pwm->RNG2 = range;        /* the range value, 100 level steps */
    io->pwm->DAT1 = 1;            /* initial beginning level=1/100=1% */
    io->pwm->DAT2 = 1;            /* initial beginning level=1/100=1% */
    io->pwm->CTL.field.MODE1 = 0; /* PWM mode */
    io->pwm->CTL.field.MODE2 = 0; /* PWM mode */
    io->pwm->CTL.field.RPTL1 = 1; /* not using FIFO, but repeat the last byte anyway */
    io->pwm->CTL.field.RPTL2 = 1; /* not using FIFO, but repeat the last byte anyway */
    io->pwm->CTL.field.SBIT1 = 0; /* idle low */
    io->pwm->CTL.field.SBIT2 = 0; /* idle low */
    io->pwm->CTL.field.POLA1 = 0; /* non-inverted polarity */
    io->pwm->CTL.field.POLA2 = 0; /* non-inverted polarity */
    io->pwm->CTL.field.USEF1 = 0; /* do not use FIFO */
    io->pwm->CTL.field.USEF2 = 0; /* do not use FIFO */
    io->pwm->CTL.field.MSEN1 = 1; /* use M/S algorithm, level=pwm.DAT1/PWM_RANGE */
    io->pwm->CTL.field.MSEN2 = 1; /* use M/S algorithm, level=pwm.DAT2/PWM_RANGE */
    io->pwm->CTL.field.CLRF1 = 1; /* clear the FIFO, even though it is not used */
    io->pwm->CTL.field.PWEN1 = 1; /* enable the PWM channel */
    io->pwm->CTL.field.PWEN2 = 1; /* enable the PWM channel */
}