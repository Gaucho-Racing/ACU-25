#include "acu.h"

void acu_init(ACU * acu){

    acu->max_output_current = MAX_HV_CURRENT;
    acu->max_temp = MAX_DCDC_TEMP;
    acu->temps[0] = acu->temps[2] = 25;

    acu->cell_OT_Threshold = CELL_MAX_TEMP;
    acu->cell_UT_Threshold = CELL_MIN_TEMP;

    // fans.begin();
    // fans.writeRegister(FAN_MODE_addr, 0b00000010); // ACU fan (fan 4) rpm mode
    // this->ACU_ADC.begin();
    // cur_ref = ACU_ADC.readVoltageTot(ADC_MUX_HV_CURRENT,256); //Zero current sensor offset
    // dcdc_ref = ACU_ADC.readVoltageTot(ADC_MUX_DCDC_CURRENT,256); //Zero current sensor offset
    // uint8_t count = 0;
    // while (abs(cur_ref - 1.235) > ERRMG_ISNS_VREF) {
    //     if (count > 10) {
    //     cur_ref = 1.235;
    //     break;
    //     }
    //     count++;
    //     Serial.printf("Current sensor ref: %f ", cur_ref);
    //     D_L1("HV current too far from zero. Check hardware. ");
    //     delay(500);
    //     cur_ref = ACU_ADC.readVoltageTot(ADC_MUX_HV_CURRENT,256);
    // }
    return;
}
// check if the values are correct
void check_acu(ACU * acu){
    update_all(acu);
    bool hasErrors = false;
    return;
}
void can_read(ACU * acu){
    return;
}
void can_send(ACU * acu){
    return;
}

/*
uint8_t relay_state;
    float glv_voltage; 
    float ts_voltage; 
    float ts_current;
    float shutdown_volt; 
    float dcdc_current; 
    float temps[2]; // thermistor readings
    float fan_Ref;

    float max_output_current;
    float max_temp;
     */

// modifiers
void set_max_charge_voltage(ACU * acu, float current){
    return;
}
void set_max_charge_current(ACU * acu, float current){
    return;
}
void set_max_output_current(ACU * acu, float current){
    acu->bty->max_output_current = current;
}
void set_max_temp(ACU * acu, float temp){
    return;
}

void update_shitdown_voltage(ACU * acu){
    return;
}
void update_glv_voltage(ACU * acu){
    return;
}
void update_ts_voltage(ACU * acu){
    return;
}
void update_ts_current(ACU * acu){
    return;
}
void update_dc_dc_current(ACU * acu){
    return;
}

void update_temp_one(ACU * acu){
    return;
}
void update_temp_two(ACU * acu){
    return;
}
void update_temp_three(ACU * acu){
    return;
}

void update_fan_ref(ACU * acu){
    return;
}
void update_relay_state(ACU * acu){
    return;
}

float V2T_f(float voltage, float B){ // default = 4390
    B = B < 0 ? 4390: B;
    float R = voltage / ((5.0 - voltage) / 47e3) / 100e3;
    float T = 1.0 / ((log(R) / B) + (1.0 / 298.15));
    return T - 273.15;
}
float V2T_i(int16_t voltage, float B){ // default = 4390
    B = B < 0 ? 4390: B;
    float actualVoltage = (voltage + 10000) * 0.000150;
    float R = actualVoltage / ((5.0 - actualVoltage) / 47e3) / 100e3;
    float T = 1.0 / ((log(R) / B) + (1.0 / 298.15));
    return T - 273.15;
}
float V2T_complex(float vdd, float voltage, float B, float Rsns, float Rref){
    float R = voltage / ((vdd - voltage) / Rref) / Rsns;
    float T = 1.0 / ((log(R) / B) + (1.0 / 298.15));
    return T - 273.15;
}

void update_all(ACU * acu){
    update_glv_voltage(acu);
    update_ts_voltage(acu);
    update_ts_current(acu);
    update_shitdown_voltage(acu);
    update_dc_dc_current(acu);
    update_temp_one(acu);
    update_temp_two(acu);
    update_temp_three(acu);
    update_relay_state(acu);
}