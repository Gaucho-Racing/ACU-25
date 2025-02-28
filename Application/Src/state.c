#include "state.h"

extern uint8_t cycle;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern void print_lpuart(char* arr);
extern uint16_t adc_data[3];

float constrain(float value, float lowerBound, float upperBound) {
    if (value < lowerBound) {
        return lowerBound;
    } else if (value > upperBound) {
        return upperBound;
    } else {
        return value;
    }
}

void shitdown(){

}
/// @brief do nothing, in initial state wait for VDM to send start command, maybe poll CAN
void standby(){
    // get ts_current
    state_system_check(false, false);
    cycle++;
    cycle = cycle % 8;
}
void precharge(){
    print_lpuart("In Precharge...\n");
    acu.acu_err_warns &= ~(ACU_CLEAR_WARN);
    acu.acu_err_warns &= ~(ACU_PRECHARGE);

    acu.relay_state = 0b000;
    // do error checking here

    print_lpuart("Precharge Start\n");
    reset_latch(&acu); // currently this function does nothing
    LL_mDelay(100);

    update_adc_array_data(&acu);

    if (fabs(acu.glv_voltage - acu.shutdown_volt) > ERRMG_GLV_SDC) {
        print_lpuart("Latch not closed, skill issue\n");
        state = SHITDOWN;
        return;
    }

    print_lpuart("systemCheck\n");
    if (!state_system_check(true, false)) {
        state = SHITDOWN;
        return;
    }
    print_lpuart("Close AIR-\n");
    acu.relay_state = 0b100; // close AIR-
    can_send(&acu, ACU_Status_2); // I'm totally guessing it's this one LOL

    print_lpuart("Close precharge relay\n");
    acu.relay_state = 0b101; // close precharge relay
    can_send(&acu, ACU_Status_2); // I'm totally guessing it's this one LOL;

    // check voltage, if difference > threshold after 2 seconds throw error
    uint32_t startTime = HAL_GetTick();
    update_adc_array_data(&acu);
    while (acu.ts_voltage < get_total_voltage(&acu) * PRECHARGE_THRESHOLD) {
        print_lpuart("Precharging... "); 
        if (state_system_check(false, false)) {
            print_lpuart("PreCharge (TsVoltage) => Shutdown\n");
            state = SHITDOWN;
            return;
        }

        update_adc_array_data(&acu);
        if(fabs(acu.glv_voltage - acu.shutdown_volt) > ERRMG_GLV_SDC){
            print_lpuart("SDC voltage dropped while precharging!! Check connections\n");
            acu.acu_err_warns |= ACU_PRECHARGE;
            can_send(&acu, ACU_Status_2);
            state = SHITDOWN;
            return;
        }
        if (HAL_GetTick() - startTime > 5000) { // timeout, throw error
            print_lpuart("Precharge timeout, error\n");
            acu.acu_err_warns |= ACU_PRECHARGE;
            can_send(&acu, ACU_Status_2);
            state = SHITDOWN;
            return;
        }

        can_read_all(&acu); // do all
        can_dump(&acu); // dump everything
        if(state != PRECHARGE){
            print_lpuart("NOT IN PRECHARGE??????\n");
        }
    }

    // delay 3 seconds, for safety
    startTime = HAL_GetTick();
    uint8_t goToCharge = false; // change this to false on final build
    while (HAL_GetTick() - startTime < 3000) {
        acu_check(&acu, (uint8_t)state, false);
        if(can_polling(&acu)){
            if(RxHeader.Identifier == Charger_Data_ACU){
                print_lpuart("Charger_Data_ACU ping received!\n");
                goToCharge = 1;
            }
        }
        update_adc_array_data(&acu);
        if(acu.ts_voltage < get_total_voltage(&acu) * PRECHARGE_THRESHOLD){
            state = SHITDOWN;
            acu.acu_err_warns |= ACU_PRECHARGE;
            print_lpuart("acu.ts_voltage < THRESHOLD\n");
            return;
        }
        print_lpuart("waiting...\n");
        can_dump(&acu); // dump everything
        LL_mDelay(50);
    }

    acu.relay_state = 0b111; // close all relays
    can_send(&acu, ACU_Status_3); // IDK JUST SENDING RELAY STATUS

    if(goToCharge){
        acu.chg_ctrl = 1;
        state = CHARGE;
    }
    else{
        acu.chg_ctrl = 0;
        state = NORMAL;
        print_lpuart("Precharge Done. Ready to drive. State Normal\n");
    }
    // acu.cur_ref = acu.ACU_ADC.readVoltageTot(ADC_MUX_HV_CURRENT, 1024);
    return;
}

uint32_t last_charge_time = 0;
uint32_t last_discharge_time = 0;
uint32_t last_send_time = 0;
uint32_t last_call_time = 0;

void charge(){
    acu.acu_err_warns &= ~(ACU_CLEAR_WARN);
    acu_check(&acu, (uint8_t)state, false);
    if(HAL_GetTick() - last_charge_time >= 2000){
        reset_discharge(&battery); // currently does nothing
        last_charge_time = HAL_GetTick();
        if(!state_system_check(true, false)){
            state = SHITDOWN;
            print_lpuart("Failed system check inside of charge\n");
            return;
        }
        //voltage checks done in system check, kick off cell balancing
        do_cell_balancing(&battery, true); 
    }
    if(HAL_GetTick() - last_send_time > 990){
        last_send_time = HAL_GetTick();
        if(battery.max_cell_volt > 4.15){
            battery.max_chg_current = constrain(battery.max_chg_current, 0.0, acu.target_current);
        } else {
            battery.max_chg_current = acu.target_current;
        }
        battery.max_chg_current = acu.target_current; // this part doens't really make sense bc we aren't really sending the val we just updated
        can_send(&acu, ACU_Charger_Control);
    }

    //if no CAN data for 5 seconds, shut down
    if(HAL_GetTick() - acu.lastChrgRecieveTime > 5000){
        print_lpuart("CHARGE: Charger CAN timeout, shutting down");
        state = SHITDOWN;
        return;
    }

    // re-measure current sensor ref every 5 minutes
    if (HAL_GetTick() - last_call_time > 300000) {
        last_call_time = HAL_GetTick();
        state = NORMAL; // turn off charger
        acu.chg_ctrl = 0;
        can_send(&acu, ACU_Charger_Control);
        LL_mDelay(1000);

        // ??????
        // acu.cur_ref = acu.ACU_ADC.readVoltageTot(ADC_MUX_HV_CURRENT, 1024);

        state = CHARGE; // turn charger back on
        acu.chg_ctrl = 1;
        can_send(&acu, ACU_Charger_Control);
    }
    
}
void normal(){

}

// returns false if failed, else return true
bool state_system_check(bool full_check, bool startup){
    acu_check(&acu, (uint8_t)state,startup);
    battery_check(&battery, full_check);
    return (acu.acu_err_warns & ACU_ERR_OVER_TEMP) == 0;
}

void check_charge(){
    
}