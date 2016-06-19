/*----------------------------------------------------------------------------*/
/*  oled.h   OLED display                                                     */
/*----------------------------------------------------------------------------*/
#ifndef OLED_H
#define OLED_H

#include <stdint.h>

#define MAX_OLED_STRING_LENGTH  15

void oled_init(void);
void oled_puts(char * stringz);

#endif  /* OLED_H */
