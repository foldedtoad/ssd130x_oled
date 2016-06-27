/*----------------------------------------------------------------------------*/
/*  oled.h   OLED display                                                     */
/*----------------------------------------------------------------------------*/
#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include <ssd130x.h>

/*
 *  Maximum display length:  BLE packet size and OLED line width common.
 */
#define MAX_DISPLAY_LENGTH  15

void oled_init(void);
void oled_puts(char * stringz);

#endif  /* OLED_H */
