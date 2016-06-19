/*
 *   ssd130x.c
 *   Copyright (c) 2016 Robin Callender. All Rights Reserved.
 */
#include <string.h>
#include <stdio.h>

#include "nrf_error.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_gpiote.h"
#include "twi_master.h"
#include "softdevice_handler.h"
#include "app_scheduler.h"

#include "config.h"
#include "boards.h"
#include "ssd130x.h"
#include "ssd130x_regs.h"
#include "ble_oled.h"
#include "dbglog.h"

//#include "font5x7.h"
#include "font8x8.h"

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/

extern ble_oled_t  g_oled_service;  // see main.c

/*---------------------------------------------------------------------------*/
/* Configuration section                                                     */
/*---------------------------------------------------------------------------*/

#define SSD130X_I2C_ADDRESS      0x78

//static uint8_t  m_vccstate = SSD130x_EXTERNALVCC;
static uint8_t  m_vccstate = SSD130x_SWITCHCAPVCC;

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static void ssd130x_write_cmd(uint8_t cmd)
{
    bool     success = false;
    uint8_t  bytes [2];

    bytes[0] = SSD130x_CONTROL_CMD;
    bytes[1] = cmd;

    success = twi_master_transfer(SSD130X_I2C_ADDRESS,
                                  (uint8_t*) &bytes, sizeof(bytes),
                                  TWI_ISSUE_STOP);
    if (success == true) {
        return;
    }

    PRINTF("ssd130x: write_cmd: cmd(%02Xh) FAILED\n", (unsigned) cmd);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static bool ssd130x_write_data(uint8_t data)
{
    bool     success = false;
    uint8_t  bytes [2];

    bytes[0] = SSD130x_CONTROL_DATA;
    bytes[1] = data;

    success = twi_master_transfer(SSD130X_I2C_ADDRESS,
                                  (uint8_t*) &bytes, sizeof(bytes),
                                  TWI_ISSUE_STOP);
    if (success == true) {
        return true;
    }

    PRINTF("ssd130x: write_data: data(%02Xh) FAILED\n", (unsigned) data);

    return false;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void ssd130x_write_stringz(char * stringz)
{
    unsigned char * ch;
    uint8_t         buffer [sizeof(SSD130x_chardef_t) + 1];

    buffer[0] = SSD130x_CONTROL_DATA;

    for (ch = (unsigned char*)stringz; *ch; ++ch) {

        if (*ch < sizeof(SSD130x_font)/sizeof(SSD130x_chardef_t)) {
            memcpy(&buffer[1], &SSD130x_font[*ch], sizeof(SSD130x_chardef_t));
        }
        else {
            memcpy(&buffer[1], &SSD130x_font['.'], sizeof(SSD130x_chardef_t));
        }

        twi_master_transfer(SSD130X_I2C_ADDRESS,
                            (uint8_t*) &buffer, sizeof(buffer),
                            TWI_ISSUE_STOP);
    }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
bool ssd130x_init_cmds(void)
{
    // Display Off
    ssd130x_write_cmd(SSD130x_DISPLAYOFF);

    // Frame Rate Setting
    ssd130x_write_cmd(SSD130x_SETDISPLAYCLOCKDIV);
    ssd130x_write_cmd(0x80);

    // MUX Ratio
    ssd130x_write_cmd(SSD130x_SETMULTIPLEX);
    ssd130x_write_cmd(0x1F);

    // Set Display Offset
    ssd130x_write_cmd(SSD130x_SETDISPLAYOFFSET);
    ssd130x_write_cmd(0x00);

    // Display Start Line
    ssd130x_write_cmd(SSD130x_SETSTARTLINE | 0x00);

    ssd130x_write_cmd(SSD130x_CHARGEPUMP);
    if (m_vccstate == SSD130x_EXTERNALVCC)
        ssd130x_write_cmd(0x10);  // Disable Charge Pump
    else
        ssd130x_write_cmd(0x14);  // Enable Charge Pump

    // Set Memory Addressing Mode
    ssd130x_write_cmd(SSD130x_MEMORYMODE);
    ssd130x_write_cmd(0x00);

    // Set Segment Remap:  Addr 127 --> SEG0
    ssd130x_write_cmd(SSD130x_SEGREMAP | 0x01);

    // Set COM Output Scan Direction
    ssd130x_write_cmd(SSD130x_COMSCANDEC);

    // Set SEG Pins Hardware Cfg
    ssd130x_write_cmd(SSD130x_SETCOMPINS);
    ssd130x_write_cmd(0x02);

    // Contrast Setting
    ssd130x_write_cmd(SSD130x_SETCONTRAST);
    ssd130x_write_cmd(0x8F);

    // Set Pre-charge Period
    ssd130x_write_cmd(SSD130x_SETPRECHARGE);
    if (m_vccstate == SSD130x_EXTERNALVCC)
        ssd130x_write_cmd(0x22);
    else
        ssd130x_write_cmd(0xF1);

    // Set VCOMH Deselect Level
    ssd130x_write_cmd(SSD130x_SETVCOMDESELECT);
    ssd130x_write_cmd(0x40);

    // Entire Display Mode Off
    ssd130x_write_cmd(SSD130x_DISPLAYALLON_RESUME);

    // Set Normal Display
    ssd130x_write_cmd(SSD130x_NORMALDISPLAY);

    // Display On
    ssd130x_write_cmd(SSD130x_DISPLAYON);

    return true;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static bool ssd130x_clean_DDR(void)
{
    int i, j;

    for (i=0; i < 5; i++) {

        // Set Page
        ssd130x_write_cmd(SSD130x_GDDRAM_START_ADDR + i);
        ssd130x_write_cmd(0);     // Lower Column Address
        ssd130x_write_cmd(16);    // Higher Column Address

        // NOTE: 128 is width of GDDRAM in bytes
        for (j=0; j < 128; j++) {
            if (!ssd130x_write_data(0x00))
                return false;
        }
    }
    return true;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void ssd130x_set_cursor(uint16_t column, uint16_t line)
{
    if (column > MAX_COLUMN) return;
    if (line < MIN_LINE && line > MAX_LINE) return;

    column *= FONT_WIDTH;

    ssd130x_write_cmd(SSD130x_GDDRAM_START_ADDR | (line - 1));

    ssd130x_write_cmd(SSD130x_SETLOWCOLUMN  | ((column >> 0) & 0x0F));
    ssd130x_write_cmd(SSD130x_SETHIGHCOLUMN | ((column >> 4) & 0x07));
}

/*---------------------------------------------------------------------------*/
/* To scroll whole display: start=0, stop=15                                 */
/*---------------------------------------------------------------------------*/
void ssd130x_scroll_right(uint8_t start, uint8_t stop)
{
    ssd130x_write_cmd(SSD130x_RIGHT_HORIZONTAL_SCROLL);
    ssd130x_write_cmd(0x00);
    ssd130x_write_cmd(start);
    ssd130x_write_cmd(0x00);
    ssd130x_write_cmd(stop);
    ssd130x_write_cmd(0x01);
    ssd130x_write_cmd(0xFF);
    ssd130x_write_cmd(SSD130x_ACTIVATE_SCROLL);
}

/*---------------------------------------------------------------------------*/
/* To scroll whole display: start=0, stop=15                                 */
/*---------------------------------------------------------------------------*/
void ssd130x_scroll_left(uint8_t start, uint8_t stop)
{
    ssd130x_write_cmd(SSD130x_LEFT_HORIZONTAL_SCROLL);
    ssd130x_write_cmd(0x00);   // dummy byte
    ssd130x_write_cmd(start);  // start page (PAGE[start])
    ssd130x_write_cmd(0x00);   // step interval (5-frames)
    ssd130x_write_cmd(stop);   // stop page (PAGE[stop])
    ssd130x_write_cmd(0x01);   // scrolling offset (PAGE-1)
    ssd130x_write_cmd(0xFF);
    ssd130x_write_cmd(SSD130x_ACTIVATE_SCROLL);
}

/*---------------------------------------------------------------------------*/
/* To scroll whole display: start=0, stop=15                                 */
/*---------------------------------------------------------------------------*/
void ssd130x_scroll_diag_right(uint8_t start, uint8_t stop)
{
    ssd130x_write_cmd(SSD130x_SET_VERTICAL_SCROLL_AREA);
    ssd130x_write_cmd(0x00);
    ssd130x_write_cmd(SSD130x_LCDHEIGHT);
    ssd130x_write_cmd(SSD130x_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
    ssd130x_write_cmd(0x00);
    ssd130x_write_cmd(start);
    ssd130x_write_cmd(0x00);
    ssd130x_write_cmd(stop);
    ssd130x_write_cmd(0x01);
    ssd130x_write_cmd(SSD130x_ACTIVATE_SCROLL);
}

/*---------------------------------------------------------------------------*/
/* To scroll whole display: start=0, stop=15                                 */
/*---------------------------------------------------------------------------*/
void ssd130x_scroll_diag_left(uint8_t start, uint8_t stop)
{
    ssd130x_write_cmd(SSD130x_SET_VERTICAL_SCROLL_AREA);
    ssd130x_write_cmd(0x00);
    ssd130x_write_cmd(SSD130x_LCDHEIGHT);
    ssd130x_write_cmd(SSD130x_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
    ssd130x_write_cmd(0x00);
    ssd130x_write_cmd(start);
    ssd130x_write_cmd(0x00);
    ssd130x_write_cmd(stop);
    ssd130x_write_cmd(0x01);
    ssd130x_write_cmd(SSD130x_ACTIVATE_SCROLL);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void ssd130x_scroll_stop(void)
{
    ssd130x_write_cmd(SSD130x_DEACTIVATE_SCROLL);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void ssd130x_clear(void)
{
    int i;

    for (i=0; i < (SSD130x_LCDHEIGHT * SSD130x_LCDWIDTH / 8); i++) {
        if (!ssd130x_write_data(0x00))
            return;
    }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void ssd130x_clear_line(uint16_t line) // line is one-based, not zero-based
{
    int i;

    if (line < MIN_LINE && line > MAX_LINE) return;

    ssd130x_write_cmd(SSD130x_GDDRAM_START_ADDR | (line - 1));

    ssd130x_write_cmd(SSD130x_SETLOWCOLUMN  | 0);
    ssd130x_write_cmd(SSD130x_SETHIGHCOLUMN | 0);

    for (i=0; i < SSD130x_LCDWIDTH; i++) {
        ssd130x_write_data(0x00);
    }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static void ssd130x_gpio_config(void)
{
    /* SSD130X_SCL - Input */
    nrf_gpio_pin_dir_set(SSD130X_SCL, NRF_GPIO_PIN_DIR_INPUT);

    /* SSD130X_SDA - Output high */
    nrf_gpio_pin_dir_set(SSD130X_SDA, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_set(SSD130X_SDA);

    /* SSD130X_RST - Reset */
    nrf_gpio_pin_dir_set(SSD130X_RST, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_set(SSD130X_SDA);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void ssd130x_init(void)
{
    PUTS(__FUNCTION__);

    /* Configure GPIO lines */
    ssd130x_gpio_config();

    if (twi_master_init() == false) {
        PUTS("twi_master_init failed");
        return;
    }

    /* Drive reset high, then low, then high to reinitialize. */
    nrf_gpio_pin_set(SSD130X_RST);
    nrf_delay_ms(1);

    nrf_gpio_pin_clear(SSD130X_RST);
    nrf_delay_ms(1);

    nrf_gpio_pin_set(SSD130X_RST);
    nrf_delay_ms(1);

    /* Initialize the OLED controller. */
    if (!ssd130x_init_cmds()) {
        /* Probably OLED is not connected */
        return;
    }
    nrf_delay_ms(1);

    ssd130x_clean_DDR();
}
