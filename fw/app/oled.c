/*----------------------------------------------------------------------------*/
/* oled.c OLED Display                                                        */
/*----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "nrf.h"
#include "nrf_gpio.h"

#include "oled.h"
#include "ssd130x.h"
#include "boards.h"

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*----------------------------------------------------------------------------*/
void oled_puts(char * stringz)
{
    if (strlen(stringz) <= MAX_OLED_STRING_LENGTH) {
        ssd130x_write_stringz(stringz);       
    }
}

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*----------------------------------------------------------------------------*/
void oled_test(void)
{
    ssd130x_clear();

    ssd130x_set_cursor(0, 1);
    ssd130x_write_stringz("== OLED*Blue ==");

    ssd130x_set_cursor(0, 2);
    ssd130x_write_stringz("Robin Callender");

    ssd130x_set_cursor(0, 3);
    ssd130x_write_stringz("line 3");

    ssd130x_set_cursor(0, 4);
    ssd130x_write_stringz("line 4");
}

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*----------------------------------------------------------------------------*/
void oled_init(void)
{
    ssd130x_init();

    oled_test();
}
