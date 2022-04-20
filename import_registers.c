#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "cm.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "pwm.h"
#include "bsc.h"
#include "io_peripherals.h"
#include "import_registers.h"

struct io_peripherals * import_registers( void )
{
  static struct io_peripherals  io;           /* the structure whose pointers are being initialized */
  struct io_peripherals *       return_value; /* the return value of this function */
  int                           mmap_file;    /* the file descriptor used to map the memory */
  off_t                         page_size;    /* specify how large a window is mapped per register */

  mmap_file = open( "/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC );
  if (mmap_file != -1)
  {
    page_size = 4096;

    /* try to put the physical I/O space at the same address range in the virtual address space */
    io.cm = mmap( 0, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, mmap_file, PHYSICAL_ADDRESS+0x101000 );
    if (io.cm != MAP_FAILED)
    {
      io.gpio = mmap( 0, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, mmap_file, PHYSICAL_ADDRESS+0x200000 );
      if (io.gpio != MAP_FAILED)
      {
        io.uart = mmap( 0, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, mmap_file, PHYSICAL_ADDRESS+0x201000 );
        if (io.uart != MAP_FAILED)
        {
          io.pcm = mmap( 0, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, mmap_file, PHYSICAL_ADDRESS+0x203000 );
          if (io.pcm != MAP_FAILED)
          {
            io.spi = mmap( 0, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, mmap_file, PHYSICAL_ADDRESS+0x204000 );
            if (io.spi != MAP_FAILED)
            {
              io.pwm = mmap( 0, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, mmap_file, PHYSICAL_ADDRESS+0x20c000 );
              if (io.pwm != MAP_FAILED)
              {
                io.bsc = mmap( 0, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, mmap_file, PHYSICAL_ADDRESS+0x804000 );
                if (io.bsc != MAP_FAILED)
                {
                  return_value = &io; /* mapped memory */
                }
                else
                {
                  printf( "unable to map register space7\n" );

                  close( mmap_file );

                  return_value = NULL;
                }
              }
              else
              {
                printf( "unable to map register space6\n" );

                close( mmap_file );

                return_value = NULL;
              }
            }
            else
            {
              printf( "unable to map register space5\n" );

              close( mmap_file );

              return_value = NULL;
            }
          }
          else
          {
            printf( "unable to map register space4\n" );

            close( mmap_file );

            return_value = NULL;
          }
        }
        else
        {
          printf( "unable to map register space3\n" );

          close( mmap_file );

          return_value = NULL;
        }
      }
      else
      {
        printf( "unable to map register space2\n" );

        close( mmap_file );

        return_value = NULL;
      }
    }
    else
    {
      printf( "unable to map register space1\n" );

      close( mmap_file );

      return_value = NULL;
    }
  }
  else
  {
    printf( "unable to open /dev/mem\n" );

    return_value = NULL;
  }

  return return_value;
}

