#include "acu.h"

extern void print_lpuart(char* arr);
extern void print_can_msg(uint8_t *arr);

extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t CAN_TxBuffer[64];
extern uint8_t CAN_RxBuffer[64];

extern uint16_t adc_data[3];

void acu_init(ACU * acu){
    acu->lastChrgRecieveTime = 0.0;
    acu->relay_state = 0;
    acu->chg_ctrl = 0;
    acu->acuErrCount = 0;
    return;
}
void acu_check(ACU * acu, uint8_t state, bool startup){
    update_all(acu);
    uint8_t lastAcuErrCount = acu->acuErrCount;
    acu->acu_err_warns &= ~(ACU_CLEAR_WARN);
    bool hasErrors = false;

    // check overcurrent
    if(acu->ts_current > MAX_HV_CURRENT){
        print_lpuart("Overcurrent detected\n");
        hasErrors = true;
        acu->acuErrCount++;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_err_warns |= ACU_ERR_OVER_CURR;
        }
    }
    else if(acu->ts_current > MAX_HV_CURRENT*0.8){
        print_lpuart("High Current Warning\n");
    }

    //dcdc current
    if(acu->dcdc_current > MAX_DCDC_CURRENT){
        print_lpuart("DCDC Overcurrent detected\n");
        hasErrors = true;
        acu->acuErrCount++;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_err_warns |= ACU_ERR_OVER_CURR;
        }
    }

    //glv voltage
    if(acu->glv_voltage < MIN_GLV_VOLT){
        if (acu->glv_voltage > 3) print_lpuart("GLV Undervolt detected\n");
        acu->acuErrCount++;
        hasErrors = true;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_err_warns |= ACU_ERR_UNDER_VOLT;
        }
    }
    if(acu->glv_voltage > MAX_GLV_VOLT){
        print_lpuart("GLV Overvolt detected\n");
        acu->acuErrCount++;
        hasErrors = true;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_err_warns |= ACU_ERR_OVER_VOLT;
        }
    }

    //fan ref voltage ???? 

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
                acu->acu_err_warns |= ACU_ERR_UNDER_VOLT;
            }
        }
        else if(acu->shutdown_volt > acu->glv_voltage){
            acu->acuErrCount++;
            hasErrors = true;
            if (acu->acuErrCount >= ERRMG_ACU_ERR){
                acu->acuErrCount = ERRMG_ACU_ERR;
                acu->acu_err_warns |= ACU_ERR_OVER_VOLT;
            }
        }
    }
    acu->acuErrCount = (lastAcuErrCount == acu->acuErrCount && !hasErrors)? 0 : acu->acuErrCount;

    // ACU: check acu_volt_warnings
    if(acu->ts_voltage < UNDER_VOLTAGE_20V){
        acu->acu_err_warns |= ACU_ERR_UV_20_V;
    }
    if(acu->volt_12v < UNDER_VOLTAGE_12V){
        acu->acu_err_warns |= ACU_ERR_UV_12_V;
    }
    if(acu->volt_sdc < UNDER_VOLTAGE_SDCV){
        acu->acu_err_warns |= ACU_ERR_UV_SDC;
    }
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

void can_read_all(ACU* acu){

}

void can_read(ACU * acu, uint32_t id){
    float values = 0.0;
    switch (id){
        case Charger_Data_ACU:
            acu->lastChrgRecieveTime = HAL_GetTick();
            values = 0.0;
            values += CAN_RxBuffer[0] << 8;
            values += CAN_RxBuffer[1];
            acu->chgr->charger_output_voltage = values;

            values += CAN_RxBuffer[2] << 8;
            values += CAN_RxBuffer[3];
            acu->chgr->charger_output_current = values;
            acu->chgr->chgr_status = CAN_RxBuffer[4];
            break;  
        case Debug_2_ACU:
            print_lpuart(CAN_RxBuffer); // 'read' Debug Message
            can_send(acu, ACU_Debug_2);
            break;  
        case Debug_FD_ACU:
            print_lpuart(CAN_RxBuffer); // 'read' Debug Message
            can_send(acu, ACU_Debug_FD);
            break;  
        case Ping_ACU:
            can_send(acu, ACU_Ping_Debug);
            break;  
        case Precharge_ACU:
            print_lpuart("Setting ts_active\n");
            acu->ts_active = CAN_RxBuffer[0] & 1;
            break;  
        case Config_Charge_ACU:
            values = 0.0;
            values += CAN_RxBuffer[0] << 8;
            values += CAN_RxBuffer[1];
            acu->target_voltage = values;
            values = CAN_RxBuffer[2] << 8;
            values += CAN_RxBuffer[3];
            acu->target_current = values;
            break;  
        case Config_Operational_ACU:
            values = 0.0;
            values += CAN_RxBuffer[0] << 8;
            values += CAN_RxBuffer[1];
            acu->target_min_cell_volt = (float)values;
            values = CAN_RxBuffer[2] << 8;
            values += CAN_RxBuffer[3];
            acu->target_max_cell_temp = (float)values;
            break;  
        case ECU_Status_1:
            break;
        case ECU_Status_2:
            break;
        case ECU_Status_3:
            break;
        case ECU_Ping: // 4 bytes, reveal Timesetamp in millis()
            break;
        case EM_Measurements_ACU: // i think
            acu->em->em_current = (float)((unsigned long)(CAN_RxBuffer[3]<<24)|(unsigned long)(CAN_RxBuffer[2])<<16|CAN_RxBuffer[1]<<8|CAN_RxBuffer[0]);
            acu->em->em_voltage = (float)((unsigned long)(CAN_RxBuffer[7]<<24)|(unsigned long)(CAN_RxBuffer[6])<<16|CAN_RxBuffer[5]<<8|CAN_RxBuffer[4]);
            break;  
        case EM_Data_1_ACU:
            // do nothing?
            break;  
        case EM_Data_2_ACU:
            // do nothing?
            break;  
        case EM_Status_ACU:
            // do nothing?
            break;  
        case EM_Temperature_ACU:
            // do nothing?
            break;  
        case IMD_Response_ACU:
            acu->imd->id = CAN_RxBuffer[0];
            break;  
        case IMD_Isolation_ACU: 
            acu->imd->r_iso_negative = (uint16_t)(CAN_RxBuffer[0] << 8);
            acu->imd->r_iso_negative |= (uint16_t)(CAN_RxBuffer[1]);
            acu->imd->r_iso_positive = (uint16_t)(CAN_RxBuffer[2] << 8);
            acu->imd->r_iso_positive |= (uint16_t)(CAN_RxBuffer[3]);
            acu->imd->r_iso_original = (uint16_t)(CAN_RxBuffer[4] << 8);
            acu->imd->r_iso_original |= (uint16_t)(CAN_RxBuffer[5]);
            acu->imd->iso_meas_count = CAN_RxBuffer[6];
            acu->imd->isolation_quality = CAN_RxBuffer[7];
            break;  
        case IMD_Voltage_ACU:
            acu->imd->hv_system_voltage = (((uint16_t)(CAN_RxBuffer[1]) << 8) + CAN_RxBuffer[0] - 32128) * 0.05;
            break;  
        case IMD_IT_System_ACU: 
            break;
        case IMD_Request_ACU:
            acu->imd->id = CAN_RxBuffer[0];
            break;  
        case IMD_General_ACU:
            acu->imd->r_iso_corrected = (uint16_t)(CAN_RxBuffer[0] << 8);
            acu->imd->r_iso_corrected |= (uint16_t)CAN_RxBuffer[1];
            acu->imd->r_iso_status = CAN_RxBuffer[2];
            acu->imd->r_iso_meas_count = CAN_RxBuffer[3];
            acu->imd->status_warnings_alarms = (uint16_t)(CAN_RxBuffer[4] << 8);
            acu->imd->status_warnings_alarms |= (uint16_t)(CAN_RxBuffer[5]);
            acu->imd->status_device_activity = CAN_RxBuffer[6];
            break;
        default:
            break;
        bzero(CAN_RxBuffer, sizeof(CAN_RxBuffer));
    }
}

void can_send(ACU * acu, uint32_t id){

    TxHeader.Identifier = id;
    uint32_t millis = HAL_GetTick();
    bzero(CAN_TxBuffer, sizeof(CAN_TxBuffer));

    switch (id){
        case ACU_Debug_2: // 0x000
            print_lpuart("send ACU_Debug_2...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK){
                print_lpuart("ACU_Debug_2 failed...\n");
            }
            break;       
        case ACU_Debug_FD: // 0x001
            print_lpuart("send ACU_Debug_FD...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_64;
            if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK){
                print_lpuart("ACU_Debug_FD failed...\n");
            }
            break;     
        case ACU_Ping_Debug: // 0x002

            CAN_TxBuffer[0] = millis & 0x0001;
            CAN_TxBuffer[1] = (millis & 0x0010) >> 1;
            CAN_TxBuffer[2] = (millis & 0x0100) >> 2;
            CAN_TxBuffer[3] = (millis & 0x1000) >> 3;

            print_lpuart("send ACU_Ping_Debug...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Ping_Debug failed...\n");
            }
            break;    
        case ACU_Ping_ECU: // 0x002

            CAN_TxBuffer[0] = millis & 0x0001;
            CAN_TxBuffer[1] = (millis & 0x0010) >> 1;
            CAN_TxBuffer[2] = (millis & 0x0100) >> 2;
            CAN_TxBuffer[3] = (millis & 0x1000) >> 3;

            print_lpuart("send ACU_Ping_ECU...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_PingACU_Ping_ECU_Debug failed...\n");
            }
            break;       
        case ACU_Status_1: // 0x007
            print_lpuart("send ACU_Status_1...\n");

            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            float total_volt = get_total_voltage(acu);
            CAN_TxBuffer[0] = ((uint16_t)total_volt) >> 8;
            CAN_TxBuffer[1] = (uint16_t)total_volt;

            CAN_TxBuffer[2] = ((uint16_t)acu->ts_voltage) >> 8;
            CAN_TxBuffer[3] = (uint16_t)acu->ts_voltage;

            CAN_TxBuffer[4] = ((uint16_t)acu->ts_current) >> 8;
            CAN_TxBuffer[5] = (uint16_t)acu->ts_current;

            CAN_TxBuffer[6] = acu->acu_SOC;
            CAN_TxBuffer[7] = acu->glv_SOC;

            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Status_1 failed...\n");
            }

            break;       
        case ACU_Status_2:
            print_lpuart("send ACU_Status_2...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_7;
            CAN_TxBuffer[0] = 0; // 20v Voltage
            CAN_TxBuffer[1] = (uint8_t)acu->volt_12v;
            CAN_TxBuffer[2] = (uint8_t)acu->volt_sdc;
            CAN_TxBuffer[3] = (uint8_t)acu->bty->min_cell_volt;
            CAN_TxBuffer[4] = (uint8_t)acu->bty->max_cell_temp;
            CAN_TxBuffer[5] = (uint8_t)(acu->acu_err_warns & 0xFF);

            CAN_TxBuffer[6] = ((uint8_t)(acu->acu_err_warns >> 8)) & 0x01;
            CAN_TxBuffer[6] += (acu->ir_precharge_state << 1);
            CAN_TxBuffer[6] += (acu->ir_state << 2);
            CAN_TxBuffer[6] += (acu->software_latch << 3);
            
            CAN_TxBuffer[7] = 0;
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
            TxHeader.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 64; cell < 96; cell+=2){
                CAN_TxBuffer[cell] = (uint8_t)(acu->bty->cell_volt[cell]);
                CAN_TxBuffer[cell+1] = (uint8_t)(acu->bty->cell_temp[cell]);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
            break;    
        case ACU_Cell_Data_4:
            print_lpuart("ACU_Cell_Data_4... how did u get here?\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 96; cell < 128; cell+=2){
                CAN_TxBuffer[cell] = (uint8_t)(acu->bty->cell_volt[cell]);
                CAN_TxBuffer[cell+1] = (uint8_t)(acu->bty->cell_temp[cell]);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
            break;    
        case ACU_Cell_Data_5:
            print_lpuart("ACU_Cell_Data_5... how did u get here?\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 128; cell < 160; cell+=2){
                CAN_TxBuffer[cell] = (uint8_t)(acu->bty->cell_volt[cell]);
                CAN_TxBuffer[cell+1] = (uint8_t)(acu->bty->cell_temp[cell]);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
            break;    
        case ACU_DC_DC_Status: // 0x012
            print_lpuart("send ACU_DC_DC_Status...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            CAN_TxBuffer[0] = (uint16_t)acu->input_voltage >> 8;
            CAN_TxBuffer[1] = (uint16_t)acu->input_voltage;
            CAN_TxBuffer[2] = (uint16_t)acu->ouput_voltage >> 8;
            CAN_TxBuffer[3] = (uint16_t)acu->ouput_voltage;
            CAN_TxBuffer[4] = acu->input_current;
            CAN_TxBuffer[5] = acu->ouput_current;
            CAN_TxBuffer[6] = acu->dc_dc_temp;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_DC_DC_Status failed...\n");
            }
            break;   
        case ACU_Charger_Control:
            print_lpuart("send ACU_Charger_Control...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_5;
            CAN_TxBuffer[0] = (uint16_t)acu->target_voltage >> 8;
            CAN_TxBuffer[1] = (uint16_t)acu->target_voltage;
            CAN_TxBuffer[2] = (uint16_t)acu->target_temp >> 8;
            CAN_TxBuffer[3] = (uint16_t)acu->target_temp;
            CAN_TxBuffer[4] = acu->chg_ctrl & 1;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Charger_Control failed...\n");
            }
            break;
        default:
            break;
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

void update_adc_array_data(ACU* acu){
    acu->ts_voltage = adc_data[0];
    acu->volt_12v = adc_data[1];
    acu->volt_sdc = adc_data[2];
}

float update_and_get_ts_voltage(ACU* acu){
    acu->ts_voltage = adc_data[0];
    return acu->ts_voltage;
}

void update_all(ACU * acu){
    update_adc_array_data(acu); // updates: ts_voltage, some other voltage, sdc_voltage
    // updateRelayState();
}

void reset_latch(ACU *acu){
    return;
}