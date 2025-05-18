#include "state.h"

extern uint8_t cycle;
extern uint16_t adc_data[6];
extern FDCAN_RxHeaderTypeDef RxHeader_Charger;
extern void print_lpuart(char* arr);

/// @brief constrains float vlaues
/// @param value 
/// @param lowerBound 
/// @param upperBound 
/// @return another float
float constrain(float value, float lowerBound, float upperBound) {
    if (value < lowerBound) {
        return lowerBound;
    } else if (value > upperBound) {
        return upperBound;
    } else {
        return value;
    }
}

/// @brief returns the current state
/// @return uint8_t of state
uint8_t get_state(){
    return (uint8_t)(state);
}

/// @brief self explanatory
void shitdown(){
    print_lpuart("State: 🪦");
    // Open all
    acu.relay_state = 0;
    acu.chg_ctrl = (uint8_t)NO_CHARGE;
    enqueue(ACU_Charger_Control, FDCAN3);

    //indicates to battery to stop charging
    reset_discharge(&battery); // TODO: double check this
    
    acu.acu_err_warns &= ~(ACU_CLEAR_WARN);
    uint16_t precharge_error = acu.acu_err_warns & ACU_PRECHARGE;

    acu.acu_err_warns &= ~(ACU_CLEAR_ERRR);
    if (precharge_error) acu.acu_err_warns |= ACU_PRECHARGE;

    uint8_t pass = state_system_check(true, false);
    update_adc_data(&acu);
    
    if (acu.ts_voltage < SAFE_V_TO_TURN_OFF && pass) { // safe to turn off if TS voltage < 60V
        print_lpuart("Shutdown (Safe) => Standby");
        state = STANDBY;
    }
}

/// @brief do nothing, in initial state wait for VDM to send start command, maybe poll CAN
void standby(){
    print_lpuart("State: 🏠");
    // get ts_current
    update_adc_data(&acu);
    if(state_system_check(false, false) == false){
        print_lpuart("𝓕𝓾𝓬𝓴\n");
        state = SHITDOWN;
    }

    // update cycle
    cycle++;
    cycle = cycle % 8;
}

/// @brief State: PRECHARGE
void precharge(){
    print_lpuart("State: 🙏");
    acu.acu_err_warns &= ~(ACU_CLEAR_WARN);
    acu.acu_err_warns &= ~(ACU_PRECHARGE);

    // all to zero
    acu.relay_state = 0;

    // if (not successful) {
    //     state = SHUTDOWN;
    // }

    LL_mDelay(100);
    update_adc_data(&acu);

    if ((fabsf(acu.sdc_volt_w - acu.sdc_volt_v) < GLV_SDC_LOW) && acu.sdc_volt_v > SDC_HIGH) {
        print_lpuart("¯\\_(ツ)_/¯ Latch not closed, skill issue\n");
        acu.acu_latch = 0;
        state = SHITDOWN;
        return;
    }
    else {acu.acu_latch = 1;}
    
    // system check
    if (!state_system_check(true, false)) {
        print_lpuart("¯\\_(ツ)_/¯ failed state_system_check\n");
        state = SHITDOWN;
        return;
    }

    // close AIR-
    acu.relay_state |= AIR_MINUS;
    // hopefully data will send itself

    // Close precharge relay
    acu.relay_state |= RELAY_PRE;
    // hopefully data will send itself

    uint32_t start_time = HAL_GetTick();
    update_adc_data(&acu);

    // keep looping until ts_voltage reaches 0.95 of total cell voltage
    while (acu.ts_voltage < get_total_voltage(&acu) * PRECHARGE_THRESHOLD) {

        if (!state_system_check(false, false)) {
            print_lpuart("¯\\_(ツ)_/¯ PreCharge (ts_voltage) => Shutdown\n");
            state = SHITDOWN;
            return;
        }

        if(fabsf(acu.voltage_12v - acu.sdc_volt_w) > GLV_SDC_LOW){
            print_lpuart("¯\\_(ツ)_/¯ SDC voltage dropped while precharging!! Check connections\n");
            acu.acu_err_warns |= ACU_PRECHARGE;
            enqueue(ACU_Status_2, FDCAN1);
            state = SHITDOWN;
            return;
        }
        if (HAL_GetTick() - start_time > 5000) { // timeout, throw error
            print_lpuart("¯\\_(ツ)_/¯ Precharge timeout, error\n");
            acu.acu_err_warns |= ACU_PRECHARGE;
            enqueue(ACU_Status_2,FDCAN1);
            state = SHITDOWN;
            return;
        }

        if(state != PRECHARGE){
            print_lpuart("🦍💨 NOT IN PRECHARGE??????\n");
        }
        update_adc_data(&acu);
    }

    start_time = HAL_GetTick();
    uint8_t goToCharge = 0; // change this to false on final build
    update_adc_data(&acu);

    // 3 seconds to check if we go to charge
    while (HAL_GetTick() - start_time < 3000) {
        if(!acu_check(&acu, false)){
            state = SHITDOWN;
            return;
        }
        // signal to go to charge!
        if(acu.chgr->chgr_status ^ CHARGER_COOMMMM){ // (if X ^ 1) => true
            print_lpuart("💩 Charger_Data_ACU ping received!\n");
            goToCharge = 1;
        }
        
        // actively check total_voltage vs ts_voltage just in case
        if(acu.ts_voltage < get_total_voltage(&acu) * PRECHARGE_THRESHOLD){
            print_lpuart("( ˶°ㅁ°) !! TS Voltage went down in Precharge\n");
            acu.acu_err_warns |= ACU_PRECHARGE;
            state = SHITDOWN;
            return;
        }
        update_adc_data(&acu);
        LL_mDelay(50);
    }

    acu.relay_state |= AIR_PLUS;
    enqueue(ACU_Status_3,FDCAN3);

    if(goToCharge){
        acu.chg_ctrl = PLS_CHARGE;
        state = CHARGE;
    }
    else{
        acu.chg_ctrl = NO_CHARGE;
        state = NORMAL;
    }
    return;
}

uint32_t last_charge_time = 0;
uint32_t last_discharge_time = 0;
uint32_t last_send_time = 0;
uint32_t last_call_time = 0;

void charge(){
    print_lpuart("State: 🛌");
    acu.acu_err_warns &= ~(ACU_CLEAR_WARN);

    if(!acu_check(&acu, false)){
        state = SHITDOWN;
        return;
    }
    
    if(HAL_GetTick() - last_charge_time >= 2000){
        reset_discharge(&battery); // TODO: config cell balancing

        last_charge_time = HAL_GetTick();
        if(!state_system_check(true, false)){
            state = SHITDOWN;
            print_lpuart("( ˶°ㅁ°) !! Failed system check inside of charge\n");
            return;
        }
        // do cell balancing
        do_cell_balancing(&battery); 
    }

    if(HAL_GetTick() - last_send_time > 990){
        last_send_time = HAL_GetTick();
        if(battery.max_cell_volt > battery.max_volt_thresh){
            battery.max_chg_current = constrain(battery.max_chg_current, 0.0f, acu.target_current);
        } else {
            battery.max_chg_current = acu.target_current;
        }
        // TODO: figure this line out
        battery.max_chg_current = acu.target_current; 
        //every 0.99 seconds send charger "ping"
        enqueue(ACU_Charger_Control,FDCAN3);
    }

    //if no CAN data for 5 seconds, shut down
    if(HAL_GetTick() - acu.lastChrgRecieveTime > 5000){
        print_lpuart("( ˶°ㅁ°) !! CHARGE: Charger CAN timeout, shutting down");
        state = SHITDOWN;
        return;
    }

    // re-measure current sensor ref every 5 minutes
    if (HAL_GetTick() - last_call_time > 300000) {

        last_call_time = HAL_GetTick();
        state = NORMAL; // turn charger off

        acu.chg_ctrl = NO_CHARGE;
        enqueue(ACU_Charger_Control,FDCAN3);

        LL_mDelay(1000);

        state = CHARGE; // turn charger back on
        acu.chg_ctrl = PLS_CHARGE;
        enqueue(ACU_Charger_Control,FDCAN3);
    }
    
    return;
}

uint8_t tsVoltErrCount = 0;
void normal(){
    print_lpuart("State: 💃");
    if(!state_system_check(false, false)){
        print_lpuart("( ˶°ㅁ°) !! SystemCheck failed in NORMAL state\n");
        state = SHITDOWN;
        return;
    }

    update_adc_data(&acu);
    float totalV = get_total_voltage(&acu);
    if (fabsf(acu.ts_voltage - totalV) > 80) {
        print_lpuart("∘ ∘ ∘ ( °ヮ° ) ? TS voltage mismatch");
        tsVoltErrCount++;
        if (tsVoltErrCount >= ERRMG_ACU_ERR) {
            tsVoltErrCount = ERRMG_ACU_ERR;
            state = SHITDOWN;
            if (acu.ts_voltage < totalV) {
                acu.acu_err_warns |= ACU_ERR_UNDER_VOLT;
            }
            else {
                acu.acu_err_warns |= ACU_ERR_OVER_VOLT;
            }
        }
    }
    else {
        tsVoltErrCount = 0;
    }

    // update cycle
    cycle++;
    cycle = cycle % 8;

    update_adc_data(&acu);
    if (acu.ts_current > 0.5) acu.cur_LastHighTime = HAL_GetTick();
    if (HAL_GetTick() - acu.cur_LastHighTime > 10000) {
        update_adc_data(&acu);
    }
}

/// @brief System Check for everything
/// @param full_check 
/// @param startup 
/// @return returns True if passes, False otherwise
bool state_system_check(bool full_check, bool startup){

    bool a_check = acu_check(&acu, startup);

    if(a_check == false){
        print_lpuart("(¬_¬\") Failed acu_check\n");
    }


    bool b_check = battery_check(&battery, full_check);

    if(b_check == false){
        print_lpuart("(¬_¬\") Failed battery_check\n");
    }

    // update errors
    if(battery.faults & BATTERY_FAULT_CELL_OV){
        acu.acu_err_warns |= ACU_ERR_OVER_VOLT;
    }
    if(battery.faults & BATTERY_FAULT_CELL_UV){
        acu.acu_err_warns |= ACU_ERR_UNDER_VOLT;
    }
    if(battery.faults & BATTERY_FAULT_CELL_OT){
        acu.acu_err_warns |= ACU_ERR_OVER_TEMP;
    }

    if(a_check == false){
        // print all current errors
    }
    if(b_check == false){
        // print all current errors
    }
    
    return !a_check & !b_check;
}