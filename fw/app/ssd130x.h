/*
 *   ssd130x.h
 */
#ifndef _SSD130X_H_
#define _SSD130X_H_

#include <stdint.h>
#include <stdbool.h>

extern const int min_line;
extern const int max_line;
extern const int max_line_width;

void ssd130x_init(void);
void ssd130x_clear(void);
void ssd130x_clear_line(uint16_t line);
void ssd130x_write_stringz(char * str);
void ssd130x_set_cursor(uint16_t column, uint16_t line);
void ssd130x_scroll_right(uint8_t start, uint8_t stop);
void ssd130x_scroll_left(uint8_t start, uint8_t stop);
void ssd130x_scroll_diag_right(uint8_t start, uint8_t stop);
void ssd130x_scroll_diag_left(uint8_t start, uint8_t stop);
void ssd130x_scroll_stop(void);

#endif /* _SSD130X_H_ */
