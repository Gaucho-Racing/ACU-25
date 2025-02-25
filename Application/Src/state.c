#include "state.h"

extern void print_lpuart(char* arr);

void shitdown(){

}
void standby(){
    update_ts_current(&acu);
    float current = acu.ts_current;
    if (fabs(current) < 1) acu.cur_ref += current * 0.01;
    // acu.dcdc_ref += (read_adc_voltage(acu.adc, (uint8_t)ADC_MUX_DCDC_CURRENT) - acu.dcdc_ref) * 0.01;
    state_system_check(false, false);
    // cycle++;
    // cycle = cycle % 8;
}
void precharge(){
    // digitalWrite(PIN_DCDC_EN, LOW);
    acu.acu_volt_warnings[0] = acu.acu_volt_warnings[1] = acu.acu_volt_warnings[2] = 0;
    acu.acu_errors[5] = 0;

    // if (!acu.setRelayState(0)) {
    // state = SHUTDOWN;
    // }

    update_glv_voltage(&acu);
    update_shitdown_voltage(&acu);

    // print_lpuart("Precharge Start\n");
    // acu.resetLatch();
    // delay(100);

    update_glv_voltage(&acu);
    update_shitdown_voltage(&acu);

    if (fabs(acu.glv_voltage - acu.shutdown_volt) > ERRMG_GLV_SDC) {
        print_lpuart("Latch not closed, skill issue\n");
        // delay(1000);
        state = SHITDOWN;
        return;
    }

    print_lpuart("systemCheck\n");
    if (state_system_check(true, false)) {
        state = SHITDOWN;
        return;
    }
    print_lpuart("Close AIR-\n");
    // acu.setRelayState(0b100); // close AIR-
    can_send(&acu, ACU_Status_2); // I'm totally guessing it's this one LOL
    print_lpuart("Close precharge relay\n");
    // acu.setRelayState(0b101); // close precharge relay
    can_send(&acu, ACU_Status_2); // I'm totally guessing it's this one LOL;

    // check voltage, if difference > threshold after 2 seconds throw error
    // uint32_t startTime = millis();
    while (acu.ts_voltage < battery.stackVoltage[0] * PRECHARGE_THRESHOLD) {
        // print_lpuart("Precharging... "); print_lpuart(acu.getTsVoltage(false));
        if (state_system_check(false, false)) {
            print_lpuart("PreCharge (TsVoltage) => Shutdown\n");
            state = SHITDOWN;
            return;
        }

        update_glv_voltage(&acu);
        update_shitdown_voltage(&acu);

        if(fabs(acu.glv_voltage - acu.shutdown_volt) > ERRMG_GLV_SDC){
            // Serial.printf("Vglv: %f, Vsdp: %f\n", Vglv, Vsdp);
            print_lpuart("SDC voltage dropped while precharging!! Check connections\n");
            acu.acu_errors[5] = 1;
            can_send(&acu, ACU_Status_1);
            print_lpuart("PreCharge (ERRMG_GLV_SDC) => Shutdown\n");
            state = SHITDOWN;
            return;
        }
        // if (millis() - startTime > 5000) { // timeout, throw error
        //     acu.acu_errors[5] = 1;
        //     can_send(&acu, ACU_Status_1);
        //     state = SHITDOWN;
        //     print_lpuart("Precharge timeout, error\n");
        //     return;
        // }

        can_read(&acu, 1); // do all
    }

    // delay 3 seconds, for safety
    // startTime = millis();
    // CAN_message_t msg;
    // bool goToCharge = false; // change this to false on final build
    // while (millis() - startTime < 3000) {
        acu_check(&acu, (uint8_t)PRECHARGE, false);

    // if (can_prim.read(msg)) {
    // if (msg.id == Charger_Data) {
    // D_L1("Charger is plugged in, go to charging mode");
    // goToCharge = true;
    // }
    // }

    update_ts_voltage(&acu);
    get_total_voltage(&acu);
    if (acu.ts_voltage < battery.stackVoltage[0] * PRECHARGE_THRESHOLD) {
        state = SHITDOWN;
        acu.acu_errors[5] = 1;
        return;
    }

    // Serial.print("waiting... "); Serial.println(acu.getTsVoltage(false));
    // dumpCANbus();
    // delay(50);
    // acu.setRelayState(0b111); // close all relays
    // sendCANData(ACU_General2);

    print_lpuart("Precharge Done. Ready to drive. State Normal\n");
    // state = goToCharge ? CHARGE : NORMAL;
    // acu.cur_ref = acu.ACU_ADC.readVoltageTot(ADC_MUX_HV_CURRENT, 1024);
    return;
}
void charge(){

}
void normal(){

}

bool state_system_check(bool full_check, bool startup){
    acu_check(&acu, startup, (uint8_t)state);
    system_check(&battery, startup);
    return true;
}

void check_charge(){
    
}