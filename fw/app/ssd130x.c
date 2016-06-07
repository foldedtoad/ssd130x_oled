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

#include "math.h"
#include "limits.h"

#include "font5x7.h"

extern ble_oled_t  g_oled_service;  // see main.c

/*---------------------------------------------------------------------------*/
/* Configuration section                                                     */
/*---------------------------------------------------------------------------*/

#define SSD130X_I2C_ADDRESS      0x78

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/

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
void ssd130x_write_stringz(unsigned char * str)
{
    bool     success = false;
    uint8_t  reg;
    int      length;

    length = strlen((char*)str);

    reg = SSD130x_CONTROL_DATA;

    success = twi_master_transfer(SSD130X_I2C_ADDRESS,
                                  &reg, sizeof(reg),
                                  TWI_DONT_ISSUE_STOP);
    if (success == true) {

        success = twi_master_transfer(SSD130X_I2C_ADDRESS,
                                     (uint8_t*)str, length,
                                     TWI_ISSUE_STOP);
        if (success == true) {
            return;
        }
    }

    PRINTF("ssd130x write string: reg(%02Xh) FAILED\n", (unsigned) reg);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
bool ssd130x_init_cmds(void)
{
    // Display Off
    ssd130x_write_cmd(SSD130x_DISPLAYOFF);

    // MUX Ratio
    ssd130x_write_cmd(SSD130x_SETMULTIPLEX);
    ssd130x_write_cmd(0x0F);

    // Set Display Offset
    ssd130x_write_cmd(SSD130x_SETDISPLAYOFFSET);
    ssd130x_write_cmd(0x1F);

    // Set Segment Remap
    ssd130x_write_cmd(SSD130x_SEGREMAP | 0x01);

    // Set COM Output Scan Direction
    ssd130x_write_cmd(SSD130x_COMSCANINC);

    // Set Normal Display
    ssd130x_write_cmd(SSD130x_NORMALDISPLAY);

    // Display Start Line
    ssd130x_write_cmd(SSD130x_SETSTARTLINE | 0);

    // Entire Display Mode Off
    ssd130x_write_cmd(SSD130x_DISPLAYALLON_RESUME);

    // Contrast Setting
    ssd130x_write_cmd(SSD130x_SETCONTRAST);
    ssd130x_write_cmd(0x39);

    // Set Pre-charge Period
    ssd130x_write_cmd(SSD130x_SETPRECHARGE);
    ssd130x_write_cmd(0x11);

    // Frame Rate Setting
    ssd130x_write_cmd(SSD130x_SETDISPLAYCLOCKDIV);
    ssd130x_write_cmd(0xC4);

    // Set SEG Pins Hardware Cfg
    ssd130x_write_cmd(SSD130x_SETCOMPINS);
    ssd130x_write_cmd(0x12);

    // Set VCOMH Deselect Level
    ssd130x_write_cmd(SSD130x_SETVCOMDESELECT);
    ssd130x_write_cmd(0x20);

    // Set Memory Addressing Mode : Page Address mode
    ssd130x_write_cmd(SSD130x_MEMORYMODE);
    ssd130x_write_cmd(2);

    // Set Column Address
    ssd130x_write_cmd(SSD130x_SETCOLUMNADDRESS);
    ssd130x_write_cmd(MIN_COLUMN);
    ssd130x_write_cmd(MAX_COLUMN);

    // Set Page Address
    ssd130x_write_cmd(SSD130x_SETPAGEADDRESS);
    ssd130x_write_cmd(MIN_LINE - 1);
    ssd130x_write_cmd(MAX_LINE - 1);

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

    nrf_delay_ms(1);

    /* Clear display */
    ssd130x_clear();
    
    nrf_delay_ms(1);

    ssd130x_set_cursor(0, 1);
    ssd130x_set_cursor(0, 2);

    nrf_delay_ms(1);
    
    ssd130x_write_stringz((unsigned char*) "== OLED Blue ==");
}
