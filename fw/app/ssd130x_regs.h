/*---------------------------------------------------------------------------*/
/* ssd130x_regs.h                                                            */
/*---------------------------------------------------------------------------*/
#ifndef _SSD130X_REGS_H_
#define _SSD130X_REGS_H_

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/

#define SSD130x_LCDWIDTH    96
#define SSD130x_LCDHEIGHT   16

#define MIN_COLUMN          0
#define MAX_COLUMN          (SSD130x_LCDWIDTH - 1)

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/

#define SSD130x_SETCONTRAST                             ((uint8_t)0x81)
#define SSD130x_DISPLAYALLON_RESUME                     ((uint8_t)0xA4)
#define SSD130x_DISPLAYALLON                            ((uint8_t)0xA5)
#define SSD130x_NORMALDISPLAY                           ((uint8_t)0xA6)
#define SSD130x_INVERTDISPLAY                           ((uint8_t)0xA7)
#define SSD130x_DISPLAYOFF                              ((uint8_t)0xAE)
#define SSD130x_DISPLAYON                               ((uint8_t)0xAF)

#define SSD130x_SETDISPLAYOFFSET                        ((uint8_t)0xD3)
#define SSD130x_SETCOMPINS                              ((uint8_t)0xDA)

#define SSD130x_SETVCOMDESELECT                         ((uint8_t)0xDB)

#define SSD130x_SETDISPLAYCLOCKDIV                      ((uint8_t)0xD5)
#define SSD130x_SETPRECHARGE                            ((uint8_t)0xD9)

#define SSD130x_SETMULTIPLEX                            ((uint8_t)0xA8)

#define SSD130x_SETLOWCOLUMN                            0x00
#define SSD130x_SETHIGHCOLUMN                           0x10

#define SSD130x_SETSTARTLINE                            ((uint8_t)0x40)

#define SSD130x_MEMORYMODE                              ((uint8_t)0x20)
#define SSD130x_SETCOLUMNADDRESS                        ((uint8_t)0x21)
#define SSD130x_SETPAGEADDRESS                          ((uint8_t)0x22)

#define SSD130x_COMSCANINC                              ((uint8_t)0xC0)
#define SSD130x_COMSCANDEC                              ((uint8_t)0xC8)

#define SSD130x_SEGREMAP                                ((uint8_t)0xA0)

#define SSD130x_CHARGEPUMP                              ((uint8_t)0x8D)

#define SSD130x_GDDRAM_START_ADDR                       ((uint8_t)0xB0)

#define SSD130x_EXTERNALVCC                             0x1
#define SSD130x_SWITCHCAPVCC                            0x2

// Scrolling #defines
#define SSD130x_ACTIVATE_SCROLL                         ((uint8_t)0x2F)
#define SSD130x_DEACTIVATE_SCROLL                       ((uint8_t)0x2E)
#define SSD130x_SET_VERTICAL_SCROLL_AREA                ((uint8_t)0xA3)
#define SSD130x_RIGHT_HORIZONTAL_SCROLL                 ((uint8_t)0x26)
#define SSD130x_LEFT_HORIZONTAL_SCROLL                  ((uint8_t)0x27)
#define SSD130x_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL    ((uint8_t)0x29)
#define SSD130x_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL     ((uint8_t)0x2A)

#define SSD130x_CONTROL_CMD      0x00
#define SSD130x_CONTROL_DATA     0x40
#define SSD130x_CONTROL_CO       0x80

#endif /* _SSD130X_REGS_H_ */
