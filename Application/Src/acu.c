#include "acu.h"

extern void print_lpuart(char* arr);
extern void print_can_msg(uint8_t *arr);
extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t CAN_TxBuffer[8];
extern uint8_t CAN_RxBuffer[8];

void acu_init(ACU * acu){

    acu->max_output_current = MAX_HV_CURRENT;
    acu->max_temp = MAX_DCDC_TEMP;
    acu->temps[0] = acu->temps[2] = 25;

    acu->cell_OT_Threshold = CELL_MAX_TEMP;
    acu->cell_UT_Threshold = CELL_MIN_TEMP;

    acu->acuErrCount = 0;
    acu->cur_ref = 0;
    acu->dcdc_ref = 2.5;
    
    return;
}
void acu_check(ACU * acu, uint8_t state, bool startup){
    update_all(acu);
    uint8_t lastAcuErrCount = acu->acuErrCount;
    bool hasErrors = false;

    // check overcurrent
    if(acu->ts_current > MAX_HV_CURRENT){
        print_lpuart("Overcurrent detected\n");
        hasErrors = true;
        acu->acuErrCount++;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_errors[3] = 1;
        }
        else if(acu->ts_current > MAX_HV_CURRENT*0.8){
            print_lpuart("High Current Warning\n");
        }
    }

    //dcdc current
    if(acu->dcdc_current > MAX_DCDC_CURRENT){
        print_lpuart("DCDC Overcurrent detected\n");
        hasErrors = true;
        acu->acuErrCount++;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_errors[3] = 1;
        }
    }

    //glv voltage
    if(acu->glv_voltage < MIN_GLV_VOLT){
        if (acu->glv_voltage > 3) print_lpuart("GLV Undervolt detected\n");
        acu->acuErrCount++;
        hasErrors = true;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_errors[2] = 1;
        }
    }
    if(acu->glv_voltage > MAX_GLV_VOLT){
        print_lpuart("GLV Overvolt detected\n");
        acu->acuErrCount++;
        hasErrors = true;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_errors[1] = 1;
        }
    }
    
    //fan ref voltage
    if(5.0 - acu->fan_Ref > ERRMG_5V){
        print_lpuart("5V Low detected\n");
        acu->acuErrCount++;
        hasErrors = true;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_errors[2] = 1;
        }
    } else if(acu->fan_Ref - 5.0 > ERRMG_5V){
        print_lpuart("5V High detected\n");
        acu->acuErrCount++;
        hasErrors = true;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_errors[1] = 1;
        }
    }

    //shutdown voltage, should be close to GLV
    if(fabs(acu->shutdown_volt - acu->glv_voltage) > ERRMG_GLV_SDC && !startup && state == 3){

        print_lpuart("Shutdown volt not close enough of GLV\n");
        char buff[32];
        sprintf(buff, "%.3f\n", fabs(acu->shutdown_volt - acu->glv_voltage));
        print_lpuart(buff);

        if(acu->shutdown_volt < acu->glv_voltage) {
            acu->acuErrCount++;
            hasErrors = true;
            if (acu->acuErrCount >= ERRMG_ACU_ERR){
                acu->acuErrCount = ERRMG_ACU_ERR;
                acu->acu_errors[2] = 1;
            }
        }
        else if(acu->shutdown_volt > acu->glv_voltage){
            acu->acuErrCount++;
            hasErrors = true;
            if (acu->acuErrCount >= ERRMG_ACU_ERR){
                acu->acuErrCount = ERRMG_ACU_ERR;
                acu->acu_errors[1] = 1;
            }
        }
    }
    acu->acuErrCount = (lastAcuErrCount == acu->acuErrCount && !hasErrors)? 0 : acu->acuErrCount;
}
bool can_polling(ACU * acu){
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, CAN_RxBuffer) == HAL_OK) {
            print_lpuart("Received data from FD_CAN!");
            return true;
        }
    }
    return false;
}
void can_read(ACU * acu, uint32_t id){

    switch (id){
        case Charger_Data_ACU:
            uint16_t values = 0.0;
            values += CAN_RxBuffer[0] << 8;
            values += CAN_RxBuffer[1];
            set_max_output_current(acu, (float)values);
            values += CAN_RxBuffer[2] << 8;
            values += CAN_RxBuffer[3];
            set_max_output_current(acu, (float)values);
            acu->chgr->chgr_errs |= CAN_RxBuffer[4] & 0x1;
            acu->chgr->chgr_errs |= CAN_RxBuffer[4] & 0x10;
            acu->chgr->chgr_errs |= CAN_RxBuffer[4] & 0x100;
            acu->chgr->chgr_errs |= CAN_RxBuffer[4] & 0x1000;
            acu->chgr->chgr_errs |= CAN_RxBuffer[4] & 0x10000;
            break;  
        case Debug_2_ACU:
            can_send(acu, ACU_Debug_2);
            break;  
        case Debug_FD_ACU:
            can_send(acu, ACU_Debug_FD);
            break;  
        case Ping_ACU:
            can_send(acu, ACU_Ping_Debug);
            break;  
        case Precharge_ACU:
            set_ts_active(acu, CAN_RxBuffer[0]);
            break;  
        case Config_Charge_ACU:
            break;  
        case Config_Operational_ACU:
            break;  
        case EM_Measurements_ACU:
            break;  
        case EM_Data_1_ACU:
            break;  
        case EM_Data_2_ACU:
            break;  
        case EM_Status_ACU:
            break;  
        case EM_Temperature_ACU:
            break;  
        case IMD_Response_ACU:
            break;  
        // case IMD_Isolation_ACU: 
        // case IMD_Voltage_ACU:
        // case IMD_IT_System_ACU: => same ID as IMD_Request_ACU
        case IMD_Request_ACU:
            break;  
        case IMD_General_ACU:
            break;
        default:
            break;
        bzero(CAN_RxBuffer, sizeof(CAN_RxBuffer));
    }
}

void can_send(ACU * acu, uint32_t id){

    TxHeader.Identifier = id;

    switch (id){
        case ACU_Debug_2:
            print_lpuart("send ACU_Debug_2...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK){
                print_lpuart("ACU_Debug_2 failed...\n");
            }
            break;       
        case ACU_Debug_FD:
            print_lpuart("send ACU_Debug_FD...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_64;
            if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK){
                print_lpuart("ACU_Debug_FD failed...\n");
            }
            break;     
        case ACU_Ping_Debug:
            print_lpuart("send ACU_Ping_Debug...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Ping_Debug failed...\n");
            }
            break;    
        case ACU_Ping_ECU:
            print_lpuart("send ACU_Ping_ECU...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_PingACU_Ping_ECU_Debug failed...\n");
            }
        break;       
        case ACU_Status_1:
            print_lpuart("send ACU_Status_1...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            float total_volt = get_total_voltage(acu);
            CAN_TxBuffer[0] = (uint16_t)total_volt >> 8;
            CAN_TxBuffer[1] = (uint16_t)total_volt;
            CAN_TxBuffer[2] = (uint16_t)acu->ts_voltage >> 8;
            CAN_TxBuffer[3] = (uint16_t)acu->ts_voltage;
            CAN_TxBuffer[4] = (uint16_t)acu->ts_current >> 8;
            CAN_TxBuffer[5] = (uint16_t)acu->ts_current;
            CAN_TxBuffer[6] = acu->acu_SOC;
            CAN_TxBuffer[7] = acu->glv_SOC;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Status_1 failed...\n");
            }

        break;       
        case ACU_Status_2:
            uint8_t buffer = 0;
            print_lpuart("send ACU_Status_2...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            CAN_TxBuffer[0] = (uint8_t)acu->volt_20v;
            CAN_TxBuffer[1] = (uint8_t)acu->volt_12v;
            CAN_TxBuffer[2] = (uint8_t)acu->volt_sdc;
            CAN_TxBuffer[3] = (uint8_t)acu->bty->min_cell_volt;
            CAN_TxBuffer[4] = (uint8_t)acu->bty->max_cell_temp;
            buffer+= acu->acu_errors[0] << 7;
            buffer+= acu->acu_errors[1] << 6;
            buffer+= acu->acu_errors[2] << 5;
            buffer+= acu->acu_errors[3] << 4;
            buffer+= acu->acu_errors[4] << 3;
            buffer+= acu->acu_volt_warnings[0] << 2;
            buffer+= acu->acu_volt_warnings[1] << 1;
            buffer+= acu->acu_volt_warnings[2];
            CAN_TxBuffer[5] = buffer;
            CAN_TxBuffer[6] = acu->acu_errors[5];
            buffer = 0;
            buffer+= acu->ir_precharge_state << 7;
            buffer+= acu->ir_state << 6;
            buffer+= acu->software_latch << 5;
            CAN_TxBuffer[7] = buffer;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Status_2 failed...\n");
            }
        break;       
        case ACU_Status_3:
            print_lpuart("send ACU_Status_3...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_12;
            CAN_TxBuffer[0] = (uint16_t)acu->hv_input_voltage >> 8;
            CAN_TxBuffer[1] = (uint16_t)acu->hv_input_voltage;
            CAN_TxBuffer[2] = (uint16_t)acu->hv_output_voltage >> 8;
            CAN_TxBuffer[3] = (uint16_t)acu->hv_output_voltage;
            CAN_TxBuffer[4] = (uint16_t)acu->hv_input_current >> 8;
            CAN_TxBuffer[5] = (uint16_t)acu->hv_input_current;
            CAN_TxBuffer[6] = (uint16_t)acu->hv_output_current >> 8;
            CAN_TxBuffer[7] = (uint16_t)acu->hv_output_current;
            CAN_TxBuffer[8] = CAN_TxBuffer[9] = CAN_TxBuffer[10] = CAN_TxBuffer[11] = 0;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Status_3 failed...\n");
            }
        break;       
        case ACU_Cell_Data_1:
            print_lpuart("send ACU_Cell_Data_1...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 0; cell < 32; cell+=2){
                CAN_TxBuffer[cell] = (uint8_t)(acu->bty->cell_volt[cell]);
                CAN_TxBuffer[cell+1] = (uint8_t)(acu->bty->cell_temp[cell]);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_1 failed...\n");
            }
        break;    
        case ACU_Cell_Data_2:
            print_lpuart("send ACU_Cell_Data_2...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 32; cell < 64; cell+=2){
                CAN_TxBuffer[cell] = (uint8_t)(acu->bty->cell_volt[cell]);
                CAN_TxBuffer[cell+1] = (uint8_t)(acu->bty->cell_temp[cell]);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
        break;    
        case ACU_Cell_Data_3:
            print_lpuart("ACU_Cell_Data_3... how did u get here?\n");
        break;    
        case ACU_Cell_Data_4:
            print_lpuart("ACU_Cell_Data_4... how did u get here?\n");
        break;    
        case ACU_Cell_Data_5:
            print_lpuart("ACU_Cell_Data_5... how did u get here?\n");
        break;    
        case ACU_DC_DC_Status:
            print_lpuart("send ACU_DC_DC_Status...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            CAN_TxBuffer[0] = (uint16_t)acu->input_voltage >> 8;
            CAN_TxBuffer[1] = (uint16_t)acu->input_voltage;
            CAN_TxBuffer[2] = (uint16_t)acu->ouput_voltage >> 8;
            CAN_TxBuffer[3] = (uint16_t)acu->ouput_voltage;
            CAN_TxBuffer[4] = (uint16_t)acu->input_current >> 8;
            CAN_TxBuffer[5] = (uint16_t)acu->input_current;
            CAN_TxBuffer[6] = (uint16_t)acu->ouput_current >> 8;
            CAN_TxBuffer[7] = (uint16_t)acu->ouput_current;
            CAN_TxBuffer[8] = (uint8_t)acu->dc_dc_temp; // wait what?
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_DC_DC_Status failed...\n");
            }
        break;   
        case ACU_Charger_Control:
            print_lpuart("send ACU_Charger_Control...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_5;
            CAN_TxBuffer[0] = (uint16_t)acu->volt_request >> 8;
            CAN_TxBuffer[1] = (uint16_t)acu->volt_request;
            CAN_TxBuffer[2] = (uint16_t)acu->temp_request >> 8;
            CAN_TxBuffer[3] = (uint16_t)acu->temp_request;
            CAN_TxBuffer[4] = acu->chg_ctrl & 1;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Charger_Control failed...\n");
            }
        break;
        default:
            break;
        bzero(CAN_RxBuffer, sizeof(CAN_TxBuffer));
    }
}
void can_dump(ACU *acu){
    can_send(acu, ACU_Cell_Data_1);
    can_send(acu, ACU_Cell_Data_2);
    can_send(acu, ACU_Status_1);
    can_send(acu, ACU_Status_2);
    can_send(acu, ACU_Status_3);
    // can_send(acu, Powertrain_Cooling);
    // can_send(acu, Charging_Cart_Config);
    // can_send(acu, IMD_Request);
    // sendCANData(IMD_Isolation_Detail);
    // sendCANData(IMD_Voltage);
    // sendCANData(IMD_IT_System);
}

// modifiers
void set_ts_active(ACU * acu, uint8_t set_bit){
    acu->ts_active = set_bit & 1;
}
void set_max_charge_voltage(ACU * acu, float value){
    acu->bty->max_charge_volt = value;
}
void set_max_charge_current(ACU * acu, float value){
    acu->max_output_current = value;
}
void set_max_output_current(ACU * acu, float value){
    acu->max_output_current = value;
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
    // acu->ts_voltage = (ACU_ADC.readVoltage(ADC_MUX_HV_VOLT) * 200);
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

// All cell voltages added up and returns sum as float
float get_total_voltage(ACU* acu){
    float total = 0.0;
    for(int i = 0; i < NUM_TOTAL_IC; i++){
        for(int j = 0; j < NUM_CELL_IC; j++){
            total += acu->bty->cell_volt[i*NUM_CELL_IC+j];
        }
    }
    return total;
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