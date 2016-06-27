/* 
 *  gather.c
 *  Copyright (c) 2016, Robin Callender. All Rights Reserved.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "app_timer.h"
#include "app_scheduler.h"

#include "gather.h"
#include "oled.h"
#include "dbglog.h"

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/

static char      m_segments [sizeof(segment_t) * MAX_SEGMENTS];
static uint16_t  m_index = 0;

/*---------------------------------------------------------------------------*/
/*  Process input segment                                                    */
/*---------------------------------------------------------------------------*/
void gather_segments(char * data)
{
    uint8_t    seg_flag;
    uint16_t   seg_len;
    char     * seg_data;

    seg_flag = data[0];
    seg_len  = data[1] + (data[2] << 8);
    seg_data = &data[3];

    PRINTF("%s: seg_flag: 0x%02x, seg_len: %d\n", __func__, 
               (unsigned) seg_flag, (int)seg_len);

    if ((m_index + seg_len) >= sizeof(m_segments)) {
        PRINTF("%s: overflow!!\n", __func__);
        return;
    }

    if (seg_flag & segment_first) {
        memset(m_segments, 0, sizeof(m_segments));
        m_index = 0;
        memcpy((char*) &m_segments[m_index], seg_data, seg_len);
    }

    if (seg_flag & segment_middle) {
        memcpy((char*) &m_segments[m_index], seg_data, seg_len);
    }

    if (seg_flag & segment_last) {

        if (!(seg_flag & segment_first)) {
            memcpy((char*) &m_segments[m_index], seg_data, seg_len);
        }

        oled_puts(m_segments);
    }

    m_index += seg_len;
}
