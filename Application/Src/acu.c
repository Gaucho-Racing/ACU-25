#include "acu.h"
extern void print_lpuart(char* arr);
extern void print_can_msg(uint8_t *arr);
 
extern volatile CAN_RX_message CAN_RxBuffer[256]; // Array to store received CAN data
extern volatile uint8_t CAN_RxBufferLevel; // Number of bytes to be read
extern volatile uint8_t CAN_RxBufferBottom; // Index of oldest data ==> increment this whenever the data is processed

extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern FDCAN_TxHeaderTypeDef TxHeader_Data;
extern FDCAN_RxHeaderTypeDef RxHeader_Data;
extern FDCAN_TxHeaderTypeDef TxHeader_Charger;
extern FDCAN_RxHeaderTypeDef RxHeader_Charger;

extern uint8_t CAN_TxData[64];
extern uint8_t CAN_RxData[64];
extern uint16_t adc_data[3];

extern volatile uint8_t p_top, p_bottom;
extern volatile uint8_t d_top, d_bottom;
extern volatile uint8_t c_top, c_bottom;
extern volatile uint8_t prim_q[64], data_q[64], charger_q[64]; 

extern volatile uint8_t CAN_1_flag;
extern volatile uint8_t CAN_2_flag;
extern volatile uint8_t CAN_3_flag;

void acu_init(ACU * acu){
    acu->lastChrgRecieveTime = 0.0f;
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
    else if(acu->ts_current > MAX_HV_CURRENT*0.8f){
        print_lpuart("High Current Warning\n");
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
    if(acu->glv_voltage < UNDER_VOLTAGE_GLV){
        acu->acu_err_warns |= ACU_ERR_UV_12_V;
    }
    if(acu->sdc_voltage < UNDER_VOLTAGE_SDCV){
        acu->acu_err_warns |= ACU_ERR_UV_SDC;
    }
}

void can_read_all(ACU* acu){
    while(CAN_RxBufferLevel > 0){
        can_read(acu, RxHeader.Identifier, (uint8_t *)(&CAN_RxBuffer[CAN_RxBufferBottom].data)); // parse the data
        bzero((void*)CAN_RxBuffer[CAN_RxBufferBottom].data, sizeof(CAN_RxBuffer[CAN_RxBufferBottom].data));
        CAN_RxBufferBottom++; // update buffer indices
        CAN_RxBufferLevel--; // update count
    }
}

void can_read(ACU * acu, uint32_t id, uint8_t * data){
    float values = 0.0f;
    uint32_t millis = HAL_GetTick();
    switch (id){
        case Debug_2_ACU:
            enqueue(ACU_Debug_2);
            break;  
        case Debug_FD_ACU:
            enqueue(ACU_Debug_FD);
            break;  
        case Ping_ACU:
            enqueue(ACU_Ping_Debug);
            break;  
        case ECU_Ping:
            enqueue(ACU_Ping_ECU);
            break;
        case Precharge_ACU:
            acu->ts_active = data[0] & 1;
            break;  
        case Charger_Data_ACU:
            acu->lastChrgRecieveTime = millis;
            values = 0.0f;
            values += data[0] << 8;
            values += data[1];
            acu->chgr->charger_output_voltage = values;

            values += data[2] << 8;
            values += data[3];
            acu->chgr->charger_output_current = values;
            acu->chgr->chgr_status = data[4];
            break;  
        case Config_Charge_ACU:
            values = 0.0f;
            values += data[0] << 8;
            values += data[1];
            acu->target_voltage = values;
            values = data[2] << 8;
            values += data[3];
            acu->target_current = values;
            break;  
        case Config_Operational_ACU:
            values = 0.0f;
            values += data[0] << 8;
            values += data[1];
            acu->config_min_cell_volt = (float)values;
            values = data[2] << 8;
            values += data[3];
            acu->config_max_cell_temp = (float)values;
            break;  
        case ECU_Status_1:
            break;
        case ECU_Status_2:
            break;
        case ECU_Status_3:
            break;
        case EM_Measurements_ACU: // i think
            acu->em->em_current = (float)((unsigned long)(data[3]<<24)|(unsigned long)(data[2])<<16|data[1]<<8|data[0]);
            acu->em->em_voltage = (float)((unsigned long)(data[7]<<24)|(unsigned long)(data[6])<<16|data[5]<<8|data[4]);
            break;  
        case EM_Data_1_ACU:
            // "Team Signal 1/2: Fuck if I know"
            break;  
        case EM_Data_2_ACU:
            // "Team Signal 3/4: Fuck if I know"
            break;  
        case EM_Status_ACU:
            acu->em->status = data[0];
            memcpy(&(acu->em->energy), (data+1), sizeof(float));
            break;  
        case EM_Temperature_ACU:
            uint8_t mux_signal = data[0] & 0b11;
            acu->em->min_temp = (uint8_t)(data[1]);
            acu->em->max_temp = (uint8_t)(data[2]);

            acu->em->num_sensors = (uint8_t)((data[0] & 0b001111)<<2);
            acu->em->temps[mux_signal*5] = (uint8_t)(data[3]);
            acu->em->temps[mux_signal*5+1] = (uint8_t)(data[4]);
            acu->em->temps[mux_signal*5+2] = (uint8_t)(data[5]);
            acu->em->temps[mux_signal*5+3] = (uint8_t)(data[6]);
            acu->em->temps[mux_signal*5+4] = (uint8_t)(data[7]);

            break;  
        case IMD_Response_ACU:
            acu->imd->id = data[0];
            break;  
        case IMD_Isolation_ACU: 
            acu->imd->r_iso_negative = (uint16_t)(data[0] << 8);
            acu->imd->r_iso_negative |= (uint16_t)(data[1]);
            acu->imd->r_iso_positive = (uint16_t)(data[2] << 8);
            acu->imd->r_iso_positive |= (uint16_t)(data[3]);
            acu->imd->r_iso_original = (uint16_t)(data[4] << 8);
            acu->imd->r_iso_original |= (uint16_t)(data[5]);
            acu->imd->iso_meas_count = data[6];
            acu->imd->isolation_quality = data[7];
            break;  
        case IMD_Voltage_ACU:
            acu->imd->hv_system_voltage = (((uint16_t)(data[1]) << 8) + data[0] - 32128) * 0.05f;
            break;  
        case IMD_IT_System_ACU: 
            break;
        case IMD_Request_ACU:
            acu->imd->id = data[0];
            break;  
        case IMD_General_ACU:
            acu->imd->r_iso_corrected = (uint16_t)(data[0] << 8);
            acu->imd->r_iso_corrected |= (uint16_t)data[1];
            acu->imd->r_iso_status = data[2];
            acu->imd->r_iso_meas_count = data[3];
            acu->imd->status_warnings_alarms = (uint16_t)(data[5] << 8);
            acu->imd->status_warnings_alarms |= (uint16_t)(data[4]);
            acu->imd->status_device_activity = data[6];
            break;
        default:
            break;
    }
}

void dequeue(ACU* acu){
    // priority 1: CAN_Primary
    switch(CAN_1_flag){ 
        case 1: // ACU_Debug_2
            memcpy(CAN_TxData, CAN_RxData, 4*sizeof(uint8_t));
            TxHeader.Identifier = ACU_Debug_2;
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxData) != HAL_OK){
                print_lpuart("ACU_Debug_2 failed...\n");
            }
            CAN_1_flag = 0;
            break;
        case 2: // ACU_Ping_Debug
            memcpy(CAN_TxData, CAN_RxData, 4*sizeof(uint8_t));
            TxHeader.Identifier = ACU_Ping_Debug;
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Ping_Debug > hfdcan2 failed...\n");
            }
            CAN_1_flag = 0;
            break;
        case 3: // ACU_Ping_ECU
            memcpy(CAN_TxData, CAN_RxData, 4*sizeof(uint8_t));
            TxHeader.Identifier = ACU_Ping_ECU;
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Ping_ECU > hfdcan1 failed...\n");
            }
            CAN_1_flag = 0;
            break;   
        case 4:
            memcpy(CAN_TxData, CAN_RxData, 64*sizeof(uint8_t));
            TxHeader.Identifier = ACU_Debug_FD;
            TxHeader.DataLength = FDCAN_DLC_BYTES_64;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Debug_FD > hfdcan1 failed...\n");
            }
            CAN_1_flag = 0;
            break;
        case 5: // ACU_Status_1
            TxHeader.Identifier = ACU_Status_1;
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            float total_volt = get_total_voltage(acu);
            CAN_TxData[0] = ((uint16_t)(total_volt * 100.0f)) >> 8;
            CAN_TxData[1] = (uint16_t)(total_volt * 100.0f);
            CAN_TxData[2] = ((uint16_t)(acu->ts_voltage * 100.0f)) >> 8;
            CAN_TxData[3] = (uint16_t)(acu->ts_voltage * 100.0f);
            CAN_TxData[4] = ((uint16_t)((acu->ts_current + 327.68f) * 100.0f)) >> 8;
            CAN_TxData[5] = (uint16_t)((acu->ts_current + 327.68f) * 100.0f);
            CAN_TxData[6] = acu->acu_SOC * 51 * 0.2;
            CAN_TxData[7] = acu->glv_SOC * 51 * 0.2;
            if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxData) != HAL_OK){
                print_lpuart("ACU_Status_1 failed...\n");
            }
            CAN_1_flag = 0;
            break;
        case 6: // ACU_Status_2
            TxHeader.Identifier = ACU_Status_2;
            TxHeader.DataLength = FDCAN_DLC_BYTES_7;
            CAN_TxData[0] = 0; // 20v Voltage
            CAN_TxData[1] = (uint8_t)acu->glv_voltage * 10.0f;
            CAN_TxData[2] = (uint8_t)acu->sdc_voltage * 10.0f;
            CAN_TxData[3] = (uint8_t)(acu->bty->min_cell_volt -2.0f) * 100.0f;
            CAN_TxData[4] = (uint8_t)(acu->bty->max_cell_temp * 4.0f);
            CAN_TxData[5] = (uint8_t)(acu->acu_err_warns & 0xFF);
            CAN_TxData[6] = ((uint8_t)(acu->acu_err_warns >> 8)) & 0x01;
            CAN_TxData[6] += (acu->ir_precharge_state << 1);
            CAN_TxData[6] += (acu->ir_state << 2);
            CAN_TxData[6] += (acu->software_latch << 3);
            CAN_TxData[7] = 0;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Status_2 failed...\n");
            }
            CAN_1_flag = 0;
            break;
        case 7: // ACU_Status_3
            TxHeader.Identifier = ACU_Status_3;
            TxHeader.DataLength = FDCAN_DLC_BYTES_12;
            CAN_TxData[0] = (uint16_t)(acu->hv_input_voltage * 100.0f) >> 8;
            CAN_TxData[1] = (uint16_t)(acu->hv_input_voltage * 100.0f);
            CAN_TxData[2] = (uint16_t)(acu->hv_output_voltage * 100.0f) >> 8;
            CAN_TxData[3] = (uint16_t)(acu->hv_output_voltage * 100.0f);
            CAN_TxData[4] = (uint16_t)(acu->hv_input_current * 1000.0f) >> 8;
            CAN_TxData[5] = (uint16_t)(acu->hv_input_current * 1000.0f);
            CAN_TxData[6] = (uint16_t)(acu->hv_output_current * 1000.0f) >> 8;
            CAN_TxData[7] = (uint16_t)(acu->hv_output_current * 1000.0f);
            CAN_TxData[8] = CAN_TxData[9] = CAN_TxData[10] = CAN_TxData[11] = 0;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Status_3 failed...\n");
            }
            CAN_1_flag = 0;
            break;   
        default:
            CAN_1_flag = 0;
            break;
    }
    // priority 2: CAN_Data
    switch(CAN_2_flag){
        case 1: // ACU_Cell_Data_1
            TxHeader_Data.Identifier = ACU_Cell_Data_1;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 0; cell < 32; cell+=2){
                CAN_TxData[cell] = (uint8_t)((acu->bty->cell_volt[cell] - 2.0f) * 100.0f);
                CAN_TxData[cell+1] = (uint8_t)(acu->bty->cell_temp[cell]*4.0f);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_1 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 2: // ACU_Cell_Data_2
            TxHeader_Data.Identifier = ACU_Cell_Data_2;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 32; cell < 64; cell+=2){
                CAN_TxData[cell] = (uint8_t)((acu->bty->cell_volt[cell] - 2.0f) * 100.0f);
                CAN_TxData[cell+1] = (uint8_t)(acu->bty->cell_temp[cell]*4.0f);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
            print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 3: // ACU_Cell_Data_3
            TxHeader_Data.Identifier = ACU_Cell_Data_3;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 64; cell < 96; cell+=2){
                CAN_TxData[cell] = (uint8_t)((acu->bty->cell_volt[cell] - 2.0f) * 100.0f);
                CAN_TxData[cell+1] = (uint8_t)(acu->bty->cell_temp[cell]*4.0f);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 4: // ACU_Cell_Data_4
            TxHeader_Data.Identifier = ACU_Cell_Data_4;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 96; cell < 128; cell+=2){
                CAN_TxData[cell] = (uint8_t)((acu->bty->cell_volt[cell] - 2.0f) * 100.0f);
                CAN_TxData[cell+1] = (uint8_t)(acu->bty->cell_temp[cell]*4.0f);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 5: // ACU_Cell_Data_5
            TxHeader_Data.Identifier = ACU_Cell_Data_5;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 128; cell < 160; cell+=2){
                CAN_TxData[cell] = (uint8_t)((acu->bty->cell_volt[cell] - 2.0f) * 100.0f);
                CAN_TxData[cell+1] = (uint8_t)(acu->bty->cell_temp[cell]*4.0f);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        default:
            CAN_2_flag = 0;
            break;
    }
    // priority 3: CAN_Charger
    switch(CAN_3_flag){
        case 1:
            TxHeader_Charger.Identifier = ACU_Charger_Control;
            TxHeader_Charger.DataLength = FDCAN_DLC_BYTES_5;
            CAN_TxData[0] = (uint16_t)acu->target_voltage >> 8;
            CAN_TxData[1] = (uint16_t)acu->target_voltage;
            CAN_TxData[2] = (uint16_t)acu->target_current >> 8;
            CAN_TxData[3] = (uint16_t)acu->target_current;
            CAN_TxData[4] = acu->chg_ctrl & 1;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_Charger, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Charger_Control failed...\n");
            }
            CAN_3_flag = 0;
            break;
        default:
            break;
    }
}
void enqueue(uint32_t id){
    uint8_t primary_full = (p_top != 0 && p_top == p_bottom);
    uint8_t data_full = (d_top != 0 && d_top == d_bottom);
    uint8_t charger_full = (c_top != 0 && c_top == c_bottom);    
    switch (id){
        case ACU_Debug_2:
            if(primary_full) return;
            prim_q[p_top] = 1;
            p_top++;
            break;       
        case ACU_Ping_Debug:
            if(primary_full) return;
            prim_q[p_top] = 2;
            p_top++;
            break;     
        case ACU_Ping_ECU:
            if(primary_full) return;
            prim_q[p_top] = 3;
            p_top++;
            break;    
        case ACU_Debug_FD:
            if(primary_full) return;
            prim_q[p_top] = 4;
            p_top++;
            break;       
        case ACU_Status_1:
            if(primary_full) return;
            prim_q[p_top] = 5;
            p_top++;
            break;       
        case ACU_Status_2:
            if(primary_full) return;
            prim_q[p_top] = 6;
            p_top++;
            break;       
        case ACU_Status_3:
            if(primary_full) return;
            prim_q[p_top] = 7;
            p_top++;
            break;       
        case ACU_Cell_Data_1:
            if(data_full) return;
            data_q[d_top] = 1;
            d_top++;
            break;    
        case ACU_Cell_Data_2:
            if(data_full) return;
            data_q[d_top] = 2;
            d_top++;
            break;    
        case ACU_Cell_Data_3:
            if(data_full) return;
            data_q[d_top] = 3;
            d_top++;
            break;    
        case ACU_Cell_Data_4:
            if(data_full) return;
            data_q[d_top] = 4;
            d_top++;
            break;    
        case ACU_Cell_Data_5:
            if(data_full) return;
            data_q[d_top] = 5;
            d_top++;
            break;     
        case ACU_Charger_Control:
            if(charger_full) return;
            charger_q[c_top] = 1;
            c_top++;
            break;
        // case ACU_DC_DC_Status:
            // deprecated for now
            // break;  
        default:
            break;
    }
}
void can_dump(ACU *acu){
    print_lpuart("DUMPING ACU DATA...\n");
    enqueue(ACU_Cell_Data_1);
    enqueue(ACU_Cell_Data_2);
    enqueue(ACU_Cell_Data_3);
    enqueue(ACU_Cell_Data_4);
    enqueue(ACU_Cell_Data_5);
    enqueue(ACU_Status_1);
    enqueue(ACU_Status_2);
    enqueue(ACU_Status_3);
    enqueue(ACU_Charger_Control);
    // enqueue(ACU_DC_DC_Status);
    // enqueue(IMD_General_ACU);
    // enqueue(IMD_Voltage_ACU);
    // enqueue(IMD_IT_System_ACU);
    print_lpuart("DONE DUMPING ACU DATA...\n");
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
    acu->glv_voltage = adc_data[1];
    acu->sdc_voltage = adc_data[2];
}

void update_all(ACU * acu){
    update_adc_array_data(acu); // updates: ts_voltage, glv voltage, sdc_voltage
}

void reset_latch(ACU *acu){
    return;
}