/*----------------------------------------------------------------------------*/
/* oled.c OLED Display                                                        */
/*----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "nrf.h"
#include "nrf_gpio.h"

#include "oled.h"
#include "ssd130x.h"
#include "boards.h"

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*----------------------------------------------------------------------------*/

#define MIN(a,b) (((a)<(b))?(a):(b))

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*----------------------------------------------------------------------------*/
void oled_puts(char * stringz)
{
    char buffer [max_line_width + 1];

    int length = strlen(stringz);
    int seglen;

    ssd130x_clear();

    for (int line = min_line; line <= max_line; line++) {

        if (length <= 0) 
            break;

        seglen = MIN(length, max_line_width);

        memset(buffer, 0, sizeof(buffer));
        strncpy(buffer, stringz, seglen);

        ssd130x_set_cursor(0, line);
        ssd130x_write_stringz(buffer); 

        stringz += max_line_width;
        length  -= max_line_width;        
    }
}

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*----------------------------------------------------------------------------*/
void oled_test(void)
{
    oled_puts("This is a long test message to drive the OLED display.");
}

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*----------------------------------------------------------------------------*/
void oled_init(void)
{
    ssd130x_init();

    oled_test();
}
