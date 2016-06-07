/* 
 *  Copyright (c) 2015 Robin Callender. All Rights Reserved.
 */
#ifndef BOARDS_H
#define BOARDS_H

#include "nrf_gpio.h"

/*
 *  Only include boards which have Arduino Shield support.
 */

#if defined(BOARD_PCA10028)
  #include "pca10028.h"
#else
#error "Board is not defined or supported"
#endif

#endif
