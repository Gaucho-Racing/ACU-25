/*
  ADC1283.h - Library for reading voltages from MCP3x0x ADCs
  Created by Yandong Liu, 20230930
*/
#ifndef ADC_H
#define ADC_H

#include "stm32g4xx_ll_spi.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "config.h"

// #define USE_SPI0 1

#define USE_SPI1 1
// #define spi_settings SPISettings

#ifdef USE_SPI0
#define spi SPI
#elif USE_SPI1
#define spi SPI1
#endif

typedef struct {
    uint8_t _csPin;
    float _vref;
    uint32_t _fsck; // 3200000
    // spi_settings _mySetting = spi_settings(_fsck, MSBFIRST, SPI_MODE3);
    // void updateSpiSettings();
} ADC;

void begin(ADC * adc);

void init(ADC * adc);
void init_2(ADC * adc, uint8_t pin, float voltage, uint32_t freq);

void set_fsck(ADC * adc, uint32_t newFreq);
void set_vref(ADC * adc, float newVref);
void set_cs_pin(ADC * adc, uint8_t newPin);

float read_adc_voltage(ADC * adc, uint8_t mux);
float* read_adc_voltage_2(ADC * adc, uint8_t mux, uint8_t nextMux, float results[2]);
float read_voltage_last(ADC * adc, uint8_t nextMux);
float read_voltage_tot(ADC * adc, uint8_t mux, uint16_t nSample);

uint16_t read_raw(ADC * adc, uint8_t mux);
uint16_t* read_raw_2(ADC * adc, uint8_t mux, uint8_t nextMux, uint16_t results[2]);
uint16_t read_raw_last(ADC * adc, uint8_t nextMux);
uint16_t read_raw_tot(ADC * adc, uint8_t mux, uint8_t nSample);
uint32_t read_raw_tot_long(ADC * adc, uint8_t mux, uint16_t nSample);

#endif
