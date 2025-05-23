#include "acu.h"
#include "state.h"

extern ACU acu;
extern Battery battery;
extern State state;

extern uint8_t cycle;
extern bool first_init;
extern uint16_t adc_data[6];
extern bcc_status_t bcc_error; 
extern uint8_t bcc_cooked_count;
extern FDCAN_RxHeaderTypeDef RxHeader_Charger;

extern void print_lpuart(char* arr);
extern void print_bcc_status(bcc_status_t stat);

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

void set_state(uint8_t value){
    state = value;
}

/// @ brief: just prints the state
void print_state(){
    switch(state){
        case INIT:
            print_lpuart("State: INIT\n");
            break;
        case STANDBY:
            print_lpuart("State: STANDBY\n");
            break;
        case PRECHARGE:
            print_lpuart("State: PRECHARGE\n");
            break;
        case CHARGE:
            print_lpuart("State: CHARGE\n");
            break;
        case NORMAL:
            print_lpuart("State: NORMAL\n");
            break;
        case SHITDOWN:
            print_lpuart("State: SHITDOWN\n");
            break;
        default:
            print_lpuart("State: Error\n");
            break;
    }
}

/// @brief self explanatory
void shitdown(){
    // Open all
    acu.relay_state = 0;
    acu.chg_ctrl = (uint8_t)NO_CHARGE;
    write_prechg(state);
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

/// @brief either mega-cooked or in init
void init(){
    if(first_init == true){
        // setup battery configuring
        battery.drvConfig.commMode = BCC_MODE_TPL;
        battery.drvConfig.devicesCnt = NUM_TOTAL_IC;
        battery.drvConfig.drvInstance = 0U;
        battery.drvConfig.loopBack = false;
        for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
            battery.drvConfig.device[i] = BCC_DEVICE_MC33771C;
            battery.drvConfig.cellCnt[i] = NUM_CELL_IC;
        }
    }

    // initialize BCC first
    bcc_error = BCC_Init(&(battery.drvConfig));

    // start the cooked counter
    uint8_t counter = TRIES;
    while (bcc_error != BCC_STATUS_SUCCESS && counter > 0){ 
        print_bcc_status(bcc_error);
        bcc_error = BCC_Init(&(battery.drvConfig));
        print_lpuart("trying agin BCC_Init\n");
        counter--;
    }
    if (counter == 0){
        state = SHITDOWN;
        print_lpuart("(¬_¬\") [BCC_Init = MEGA-COOKED]\n");
        return;
    }
    
    print_lpuart("PASSED from BCC_Init\n");
    battery.min_temp_thresh = CELL_MIN_TEMP;
    battery.max_temp_thresh = CELL_MAX_TEMP;
    battery.min_volt_thresh = CELL_MIN_VOLT;
    battery.max_volt_thresh = CELL_MAX_VOLT;

    // initialize registers
    bool succ = init_registers(&battery);
    if (!succ) {
        print_lpuart("(*_*) [Failed init_registers...]\n");
        return;
    }

    clear_faults(&(battery.drvConfig));  

    // cb & first battery check
    state = init_cell_balancing(&battery) && battery_check(&battery, true) == 1 ? STANDBY : SHITDOWN;
    if (state == SHITDOWN) {
        print_lpuart("error occured in cell_balancing init, and battery_check\n");
        return;
    }

    if(first_init == true){
        update_adc_data(&acu);
        acu_init(&acu);
        acu.bty = &battery;
        print_lpuart("🤖 completed acu_init()\n");
    }

    reset_discharge(&battery);
    if(!state_system_check(true, true)){
        #if DEBUGG == 0
        state = SHITDOWN;
        #endif
        print_lpuart("Failed 1st state_system_check. SHIT\n");
    }
    else {
        state = STANDBY;
    }
    if(first_init == true){
        first_init = false;
    }
    bcc_cooked_count = 0;
}

/// @brief do nothing, in initial state wait for VDM to send start command, maybe poll CAN
void standby(){
    
    update_adc_data(&acu); // get ts_currents
    if(state_system_check(false, false) == false){
        print_lpuart("𝓕𝓾𝓬𝓴\n");
        #if DEBUGG == 0
        state = state == INIT ? INIT : SHITDOWN;
        #endif
    }
    // update cycle
    cycle++;
    cycle = cycle % 8;
}

/// @brief State: PRECHARGE
void precharge(){
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
        #if DEBUGG == 0
        state = SHITDOWN;
        return;
        #endif
    }
    else {acu.acu_latch = 1;}
    
    // system check
    if (!state_system_check(true, false)) {
        print_lpuart("𝓕𝓾𝓬𝓴\n");
        print_lpuart("¯\\_(ツ)_/¯ precharge: bad state_sys_check\n");
        #if DEBUGG == 0
        state = state == INIT ? INIT : SHITDOWN;
        return;
        #endif
    }

    // close AIR-
    acu.relay_state |= AIR_MINUS;
    // write_IRneg(state);

    // Close precharge relay
    acu.relay_state |= RELAY_PRE;

    uint32_t start_time = HAL_GetTick();
    update_adc_data(&acu);

    // keep looping until ts_voltage reaches 0.95 of total cell voltage
    while (acu.ts_voltage < get_total_voltage(&acu) * PRECHARGE_THRESHOLD) {

        if (!state_system_check(false, false)) {
            print_lpuart("¯\\_(ツ)_/¯ PreCharge (235) => Shutdown\n");
            #if DEBUGG == 0
            state = state == INIT ? INIT : SHITDOWN;
            return;
            #endif
        }

        if(fabsf(acu.voltage_12v - acu.sdc_volt_w) > GLV_SDC_LOW){
            print_lpuart("¯\\_(ツ)_/¯ PreCharge (243) SDC volt dropped\n");
            acu.acu_err_warns |= ACU_PRECHARGE;
            enqueue(ACU_Status_2, FDCAN1);
            #if DEBUGG == 0
            state = SHITDOWN;
            return;
            #endif
        }
        if (HAL_GetTick() - start_time > 5000) { // timeout, throw error
            print_lpuart("¯\\_(ツ)_/¯ (252) Precharge timeoutn");
            acu.acu_err_warns |= ACU_PRECHARGE;
            enqueue(ACU_Status_2,FDCAN1);
            #if DEBUGG == 0
            state = SHITDOWN;
            return;
            #endif
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
            print_lpuart("PRECHARGE (274) => SHITDOWN");
            #if DEBUGG == 0
            state = SHITDOWN;
            return;
            #endif
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
            #if DEBUGG == 0
            state = SHITDOWN;
            return;
            #endif
        }
        update_adc_data(&acu);
        LL_mDelay(50);
    }

    acu.relay_state |= AIR_PLUS;
    // write_IRpos(state);

    enqueue(ACU_Status_3,FDCAN3);

    if(goToCharge){
        acu.chg_ctrl = PLS_CHARGE;
        write_prechg(true);
        state = CHARGE;
    }
    else{
        acu.chg_ctrl = NO_CHARGE;
        write_prechg(false);
        state = NORMAL;
    }
    return;
}

uint32_t last_charge_time = 0;
uint32_t last_discharge_time = 0;
uint32_t last_send_time = 0;
uint32_t last_call_time = 0;

/// charge state
void charge(){
    acu.acu_err_warns &= ~(ACU_CLEAR_WARN);

    if(!acu_check(&acu, false)){
        print_lpuart("CHARGE (328) => SHITDOWN");
        #if DEBUGG == 0
        state = SHITDOWN;
        return;
        #endif
    }
    
    if(HAL_GetTick() - last_charge_time >= 2000){
        reset_discharge(&battery); 

        last_charge_time = HAL_GetTick();
        if(!state_system_check(true, false)){
            print_lpuart("( ˶°ㅁ°) !! Failed system check inside of charge\n");
            #if DEBUGG == 0
            state = state == INIT ? INIT : SHITDOWN;
            return;
            #endif
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
        enqueue(ACU_Charger_Control, FDCAN3);
    }

    //if no CAN data for 5 seconds, shut down
    if(HAL_GetTick() - acu.lastChrgRecieveTime > 5000U){
        print_lpuart("( ˶°ㅁ°) !! CHARGE: Charger CAN timeout, shutting down\n");
        #if DEBUGG == 0
        state = SHITDOWN;
        return;
        #endif
    }

    // re-measure current sensor ref every 5 minutes
    if (HAL_GetTick() - last_call_time > 300000U) {

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
    if(!state_system_check(false, false)){
        print_lpuart("( ˶°ㅁ°) !! SystemCheck failed in NORMAL state\n");
        #if DEBUGG == 0
        state = state == INIT ? INIT : SHITDOWN;
        return;
        #endif
    }

    update_adc_data(&acu);
    float totalV = get_total_voltage(&acu);
    if (fabsf(acu.ts_voltage - totalV) > 80U) {
        print_lpuart("∘ ∘ ∘ ( °ヮ° ) ? TS voltage mismatch");
        tsVoltErrCount++;
        if (tsVoltErrCount >= ERRMG_ACU_ERR) {
            tsVoltErrCount = ERRMG_ACU_ERR;
            print_lpuart("NORMAL (409) => SHITDOWN");
            #if DEBUGG == 0
            state = SHITDOWN;
            return;
            #endif
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
    print_errors_warning(&acu);
    if (a_check && b_check){
        write_bms_ok(state);
        print_lpuart("ACU & BMS ok 🫰\n");
    }
    return !a_check && !b_check;
}