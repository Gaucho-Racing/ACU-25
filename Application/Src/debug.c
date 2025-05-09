#include "debug.h"
#include "state.h"

extern ACU acu;
extern State state;

extern void print_adc_data(ACU *acu);
extern void print_lpuart(char* arr);
extern void print_voltage(Battery *bty);
extern void print_temperature(Battery * bty);

void debug(){
  print_lpuart("------------------- Debug Start -------------------\n");
  #if DUMP_TARGETS == 1
  // print target current, voltage, and temperature
  #endif

  #if DUMP_VOLTS == 1
  print_voltage(acu.bty);
  #endif

  #if DUMP_TEMPS == 1
  print_temperature(acu.bty);
  #endif

  #if DUMP_ADC_DATA == 1
  print_adc_data(&acu);
  #endif

  #if DUMP_ERR_WARN == 1
  // print acu.acu_err_warns
  #endif

  #if CHARG_CTL == 1
  // print charger control, maybe status
  #endif

  #if DUMP_IMD_DATA == 1
  // print IMD data
  
  #endif
  #if DUMP_ENERGY_MEASURE_DATA == 1
  // print EM data
  
  #endif
  #if DUMP_CHARGER_DATA == 1
  // print Charger data
  
  #endif
  print_lpuart("-------------------- End Debug --------------------\n");
}

