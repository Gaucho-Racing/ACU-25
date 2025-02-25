/*
  ADC.h - Library for reading voltages from ADC ADCs
  Created by Yandong Liu, 20230930
*/
#include "adc.h"

// void ADC::updateSpiSettings() {
//   _mySetting = spi_settings(_fsck, MSBFIRST, SPI_MODE3);
// }

void init(ADC* adc) {
  adc->_csPin = 4;
  adc->_vref = 5.0;
  adc->_fsck = 1000000;
//   updateSpiSettings();
}

void init_2(ADC* adc, uint8_t pin, float voltage, uint32_t freq) {
  adc->_csPin = pin;
  adc->_vref = voltage;
  adc->_fsck = freq;
//   updateSpiSettings();
}


void set_fsck(ADC* adc, uint32_t newFreq) {
    adc->_fsck = newFreq;
    // updateSpiSettings();
}

void set_cs_pin(ADC* adc, uint8_t newPin) {
  adc->_csPin = newPin;
}

void set_vref(ADC* adc, float newVref) {
  adc->_vref = newVref;
}


void begin(ADC * adc) {
//   pinMode(_csPin, OUTPUT);
//   digitalWrite(_csPin, HIGH);
//   spi.begin();
}

float read_adc_voltage(ADC * adc, uint8_t mux){
    return read_raw(adc, mux) / 4096.0 * adc->_vref;
}

float* read_adc_voltage_2(ADC * adc, uint8_t mux, uint8_t nextMux, float results[2]){
    uint16_t resultsRaw[2];
    // readRaw(mux, nextMux, resultsRaw);
    // results[0] = resultsRaw[0] / 4096.0 * _vref;
    // results[1] = resultsRaw[1] / 4096.0 * _vref;
    return results;
}

float read_voltage_last(ADC * adc, uint8_t nextMux){
    // return readRawLast(nextMux) / 4096.0 * _vref;
    return 0.0;
}

float read_voltage_tot(ADC * adc, uint8_t mux, uint16_t nSample){
    // if (nSample > 16) {
    //     return readRawTotLong(mux, nSample) / 4096.0 / nSample * _vref;
    // }
    // else {
    //     return readRawTot(mux, nSample) / 4096.0 / nSample * _vref;
    // }
    return 0.0;
}

uint16_t read_raw(ADC * adc, uint8_t mux){
    return 0;
}

uint16_t* read_raw_2(ADC * adc, uint8_t mux, uint8_t nextMux, uint16_t results[2]){
    return 0;
}

uint16_t read_raw_last(ADC * adc, uint8_t nextMux){
    return 0;
}

uint16_t read_raw_tot(ADC * adc, uint8_t mux, uint8_t nSample){
    return 0;
}

uint32_t read_raw_tot_long(ADC * adc, uint8_t mux, uint16_t nSample){
    return 0;
}
