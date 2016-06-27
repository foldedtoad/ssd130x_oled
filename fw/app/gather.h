/* 
 *  gather.h
 *  Copyright (c) 2016, Robin Callender. All Rights Reserved.
 */
#ifndef __GATHER_H__
#define __GATHER_H__

#define MAX_SEGMENTS             4
#define MAX_SEGMENT_DATA_LENGTH  15

typedef enum {
    segment_first  = 0x01,
    segment_middle = 0x02,
    segment_last   = 0x04,
} segment_enum_t;

typedef struct {
    segment_enum_t  seg_flag;
    uint16_t        seg_length;
    uint8_t         seg_data [MAX_SEGMENT_DATA_LENGTH];
} __attribute__((packed)) segment_t;

void gather_segments(char * segment);

#endif /* __GATHER_H__ */
