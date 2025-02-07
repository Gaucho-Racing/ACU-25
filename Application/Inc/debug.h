#ifndef DEBUG_H
#define DEBUG_H

#include "config.h"
#include "bcc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "stm32g474xx.h"
#include <stdio.h>
#include <math.h>

#ifndef __UINT32_MAX__
    #include <inttypes.h>
#else
    typedef unsigned long uint32_t;
    typedef unsigned long long uint64_t; // maybe not needed?
#endif

// self-defined
void print(char* arr);
void print_float(float value);
void print_decimal(int value);
uint8_t spi_send_string(const uint8_t *data, uint16_t length);
uint8_t spi_read_string(uint8_t *buffer, uint16_t length);

#endif