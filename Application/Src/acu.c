#include "acu.h"
extern uint8_t get_state();
extern void print_lpuart(char* arr);
extern void print_can_msg(uint8_t *arr);
 
extern volatile CAN_RX_message CAN_RxBuffer[256]; // Array to store received CAN data
extern volatile uint8_t CAN_RxBufferBottom; // Index of oldest data ==> increment this whenever the data is processed
extern volatile uint8_t CAN_RxBufferTop;
extern volatile uint8_t CAN_RxBufferLevel;

extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern FDCAN_TxHeaderTypeDef TxHeader_Data;
extern FDCAN_RxHeaderTypeDef RxHeader_Data;
extern FDCAN_TxHeaderTypeDef TxHeader_Charger;
extern FDCAN_RxHeaderTypeDef RxHeader_Charger;

extern uint8_t CAN_TxData[64];
extern uint8_t CAN_RxData[64];
extern uint16_t adc_data[6];

extern volatile uint8_t p_top, p_bottom, p_level, d_top, d_bottom, d_level, c_top, c_bottom, c_level;
extern volatile uint8_t prim_q[64], data_q[64], charger_q[64]; 
extern volatile uint8_t CAN_1_flag, CAN_2_flag, CAN_3_flag;

void acu_init(ACU * acu){
    acu->lastChrgRecieveTime = 0.0f;
    acu->chg_ctrl = NO_CHARGE;
    acu->acuErrCount = 0;
    return;
}

/// @brief ACU check => current, glv voltage, shut down voltage, warnings
/// @param acu 
/// @param state 
/// @param startup 
/// @return True if passes, False otherwise
bool acu_check(ACU * acu, bool startup){

    update_adc_data(acu);
    acu->relay_state = 0; // DOUBLE CHECK THIS

    // clean the slate
    acu->acu_err_warns &= ~(ACU_CLEAR_ERRR);
    acu->acu_err_warns &= ~(ACU_CLEAR_WARN);
    
    uint8_t lastAcuErrCount = acu->acuErrCount;
    bool hasErrors = false;

    // check overcurrent
    if(acu->ts_current > MAX_HV_CURRENT){
        #if DEBUGG
            print_lpuart("Overcurrent detected\n");
        #endif
        hasErrors = true;
        acu->acuErrCount++;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_err_warns |= ACU_ERR_OVER_CURR;
        }
    }
    else if(acu->ts_current > MAX_HV_CURRENT*0.8f){
        print_lpuart("High Current Warning\n"); // JUST A PRINT STATEMENT
    }

    // skip dcdc current checks?  bc they are handled elsewhere?

    //glv voltage
    if(acu->glv_voltage < MIN_GLV_VOLT){
        if (acu->glv_voltage > 3) {
            #if DEBUGG
            print_lpuart("GLV Undervolt detected\n");
            #endif
        }
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

    //sdc_volt_w should be close to glv_voltage
    if(fabsf(acu->sdc_volt_w - acu->glv_voltage) > GLV_SDC_LOW && !startup && get_state() == 3){
        print_lpuart("Shutdown volt not close enough of GLV\n");

        #if DEBUGG
        char buff[32];
        sprintf(buff, "%.3f\n", fabsf(acu->sdc_volt_w - acu->glv_voltage));
        print_lpuart(buff);
        #endif

        if(acu->sdc_volt_w < acu->glv_voltage) {
            acu->acuErrCount++;
            hasErrors = true;
            if (acu->acuErrCount >= ERRMG_ACU_ERR){
                acu->acuErrCount = ERRMG_ACU_ERR;
                acu->acu_err_warns |= ACU_ERR_UNDER_VOLT;
            }
        }
        else if(acu->sdc_volt_w > acu->glv_voltage){
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
    if(acu->sdc_volt_w < UNDER_VOLTAGE_SDCV){
        acu->acu_err_warns |= ACU_ERR_UV_SDC;
    }
    return !hasErrors;
}

/// @brief Checks CAN_RxBuffer and parses everything that's incoming
/// @param acu 
void can_read_handler(ACU* acu){
    while(CAN_RxBufferTop > CAN_RxBufferBottom){

        FDCAN_GlobalTypeDef * type = CAN_RxBuffer[CAN_RxBufferBottom].instance;
        uint32_t id = CAN_RxBuffer[CAN_RxBufferBottom].identifier;

        can_read(acu, type, id, (uint8_t *)(&CAN_RxBuffer[CAN_RxBufferBottom].data));
        bzero((void*)CAN_RxBuffer[CAN_RxBufferBottom].data, sizeof(CAN_RxBuffer[CAN_RxBufferBottom].data));

        CAN_RxBufferBottom++;
        CAN_RxBufferLevel--;
    }
}

/// @brief Parses single CAN message
/// @param acu 
/// @param which_can 
/// @param id 
/// @param size 
/// @param data 
void can_read(ACU * acu, FDCAN_GlobalTypeDef * which_can, uint32_t id, uint8_t * data){
    float values = 0.0f;
    uint32_t millis = HAL_GetTick();
    switch (id){
        case Debug_2_ACU:
            enqueue(ACU_Debug_2_Debug, which_can);
            break;  
        case Debug_FD_ACU:
            enqueue(ACU_Debug_FD, which_can);
            break;  
        case Debug_Ping_ACU:
            enqueue(ACU_Ping_Debug, which_can);
            break;  
        case ECU_Ping_ALL:
            enqueue(ACU_Ping_ECU, which_can);
            break;
        case Precharge_ACU:
            acu->ts_active = data[0] & 1;
            break;  
        case Charger_Data_ACU:
            acu->lastChrgRecieveTime = millis;
            values = 0.0f;
            values += data[0] << 8;
            values += data[1];
            acu->chgr->charger_output_voltage = values * 0.1; // this should be sent to somewhere

            values += data[2] << 8;
            values += data[3];
            acu->chgr->charger_output_current = values * 0.1; // this should be sent to somewhere
            acu->chgr->chgr_status = data[4]; // need to check for this when checking acu
            break;  
        case Config_Charge_ACU:
            values = 0.0f;
            values += data[0] << 8;
            values += data[1];
            acu->target_voltage = values * 0.1;
            values = data[2] << 8;
            values += data[3];
            acu->target_current = values* 0.1;
            break;  
        case Config_Ops_ACU:
            values = 0.0f;
            values += data[0] << 8;
            values += data[1];
            acu->bty->min_volt_thresh = values * 0.1;
            values = data[2] << 8;
            values += data[3];
            acu->bty->max_temp_thresh = values * 0.1;
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
            acu->em->min_temp = (uint8_t)(data[1] * 0.5);
            acu->em->max_temp = (uint8_t)(data[2] * 0.5);

            acu->em->num_sensors = (uint8_t)((data[0] & 0b001111)<<2);
            acu->em->temps[mux_signal*5] = (uint8_t)(data[3] * 0.5);
            acu->em->temps[mux_signal*5+1] = (uint8_t)(data[4] * 0.5);
            acu->em->temps[mux_signal*5+2] = (uint8_t)(data[5] * 0.5);
            acu->em->temps[mux_signal*5+3] = (uint8_t)(data[6] * 0.5);
            acu->em->temps[mux_signal*5+4] = (uint8_t)(data[7] * 0.5);

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

/// @brief Takes top message request off buffer and sends over corresponding CAN
/// @param acu 
void dequeue(ACU* acu){
    if(CAN_1_flag == 0 && CAN_2_flag == 0 && CAN_3_flag == 0) return;
    // priority 1: CAN_Primary
    switch(CAN_1_flag){ 
        case 1: // ACU_Debug_2_Debug
            memcpy(CAN_TxData, CAN_RxData, 4*sizeof(uint8_t));
            TxHeader.Identifier = ACU_Debug_2_Debug;
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxData) != HAL_OK){
                print_lpuart("ACU_Debug_2_Debug failed...\n");
            }
            CAN_1_flag = 0;
            break;
        case 2: // ACU_Ping_Debug
            memcpy(CAN_TxData, CAN_RxData, 4*sizeof(uint8_t));
            TxHeader.Identifier = ACU_Ping_Debug;
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Ping_Debug > hfdcan1 failed...\n");
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
            CAN_TxData[0] = 0; // 20v GLV voltage
            CAN_TxData[1] = (uint8_t)acu->voltage_12v * 10.0f;
            CAN_TxData[2] = (uint8_t)acu->sdc_volt_w * 10.0f;
            CAN_TxData[3] = (uint8_t)(acu->bty->min_cell_volt -2.0f) * 100.0f;
            CAN_TxData[4] = (uint8_t)(acu->bty->max_cell_temp * 4.0f);
            CAN_TxData[5] = (uint8_t)(acu->acu_err_warns & 0xFF); // takes [OT, OV, UV, OC, UC, UV_20v, UV_GLV, UV_SDC]
            CAN_TxData[6] = ((uint8_t)(acu->acu_err_warns >> 8)) & 0xff; // takes precharge error to lsb
            CAN_TxData[6] |= (acu->relay_state & AIR_PLUS) >> 1;
            CAN_TxData[6] |= (acu->relay_state & AIR_MINUS) >> 1;
            CAN_TxData[6] |= acu->acu_latch >> 3; 
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Status_2 failed...\n");
            }
            CAN_1_flag = 0;
            break;
        case 7: // ACU_Status_3
            CAN_1_flag = 0;
            break;   
        default:
            CAN_1_flag = 0;
            break;
    }
    // priority 2: CAN_Data
    switch(CAN_2_flag){
        case 1: // ACU_Ping_Debug
            memcpy(CAN_TxData, CAN_RxData, 4*sizeof(uint8_t));
            TxHeader_Data.Identifier = ACU_Ping_Debug;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_4;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Ping_Debug > hfdcan2 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 2: // ACU_Ping_ECU
            memcpy(CAN_TxData, CAN_RxData, 4*sizeof(uint8_t));
            TxHeader.Identifier = ACU_Ping_ECU;
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Ping_ECU > hfdcan2 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 3: // ACU_Cell_Data_1
            TxHeader_Data.Identifier = ACU_Cell_Data_1;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 0; cell < 32; cell+=2){
                CAN_TxData[cell] = fconstrain((acu->bty->cell_volt[cell] - 2.0f) * 100.0f);
                CAN_TxData[cell+1] = fconstrain((acu->bty->cell_temp[cell]*4.0f));
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_1 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 4: // ACU_Cell_Data_2
            TxHeader_Data.Identifier = ACU_Cell_Data_2;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 32; cell < 64; cell+=2){
                CAN_TxData[cell] = fconstrain((acu->bty->cell_volt[cell] - 2.0f) * 100.0f);
                CAN_TxData[cell+1] = fconstrain((acu->bty->cell_temp[cell]*4.0f));
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
            print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 5: // ACU_Cell_Data_3
            TxHeader_Data.Identifier = ACU_Cell_Data_3;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 64; cell < 96; cell+=2){
                CAN_TxData[cell] = fconstrain((acu->bty->cell_volt[cell] - 2.0f) * 100.0f);
                CAN_TxData[cell+1] = fconstrain((acu->bty->cell_temp[cell]*4.0f));
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 6: // ACU_Cell_Data_4
            TxHeader_Data.Identifier = ACU_Cell_Data_4;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 96; cell < 128; cell+=2){
                CAN_TxData[cell] = fconstrain((acu->bty->cell_volt[cell] - 2.0f) * 100.0f);
                CAN_TxData[cell+1] = fconstrain((acu->bty->cell_temp[cell]*4.0f));
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 7: // ACU_Cell_Data_5
            TxHeader_Data.Identifier = ACU_Cell_Data_5;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 128; cell < 160; cell+=2){
                CAN_TxData[cell] = fconstrain((acu->bty->cell_volt[cell] - 2.0f) * 100.0f);
                CAN_TxData[cell+1] = fconstrain((acu->bty->cell_temp[cell]*4.0f));
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
            CAN_TxData[0] = (((uint16_t)acu->target_voltage) * 10) >> 8;
            CAN_TxData[1] = ((uint16_t)acu->target_voltage) * 10;
            CAN_TxData[2] = (((uint16_t)acu->target_current) * 10) >> 8;
            CAN_TxData[3] = ((uint16_t)acu->target_current) * 10;
            CAN_TxData[4] = acu->chg_ctrl;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_Charger, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Charger_Control failed...\n");
            }
            CAN_3_flag = 0;
            break;
        default:
            break;
    }
}

/// @brief Takes incoming CAN message request and put into send buffer
/// @param id 
/// @param which_can 
void enqueue(uint32_t id, FDCAN_GlobalTypeDef * which_can){
    uint8_t primary_full = (p_level == 64U);
    uint8_t data_full = (d_level == 64U);
    uint8_t charger_full = (c_level == 64U);    
    switch (id){
        case ACU_Debug_2_Debug:
            if(primary_full) return;
            prim_q[p_top] = 1;
            p_top++; p_level++;
            break;       
        case ACU_Ping_Debug:
            if(which_can == FDCAN1){
                if(primary_full) return;
                prim_q[p_top] = 2;
                p_top++; p_level++;
            }
            else if (which_can == FDCAN2){
                if(data_full) return;
                data_q[d_top] = 1;
                d_top++; d_level++;
            }
            break;     
        case ACU_Ping_ECU:
            if(which_can == FDCAN1){
                if(primary_full) return;
                prim_q[p_top] = 3;
                p_top++; p_level++;
            }
            else if(which_can == FDCAN2){
                if(data_full) return;
                data_q[d_top] = 2;
                d_top++; d_level++;
            }
            break;    
        case ACU_Debug_FD:
            if(data_full) return;
            data_q[d_top] = 3;
            d_top++; d_level++;
            break;       
        case ACU_Status_1:
            if(primary_full) return;
            prim_q[p_top] = 5;
            p_top++; p_level++;
            break;       
        case ACU_Status_2:
            if(primary_full) return;
            prim_q[p_top] = 6;
            p_top++; p_level++;
            break;       
        case ACU_Status_3:
            if(primary_full) return;
            prim_q[p_top] = 7;
            p_top++; p_level++;
            break;       
        case ACU_Cell_Data_1:
            if(data_full) return;
            data_q[d_top] = 4;
            d_top++; d_level++;
            break;    
        case ACU_Cell_Data_2:
            if(data_full) return;
            data_q[d_top] = 5;
            d_top++; d_level++;
            break;    
        case ACU_Cell_Data_3:
            if(data_full) return;
            data_q[d_top] = 6;
            d_top++; d_level++;
            break;    
        case ACU_Cell_Data_4:
            if(data_full) return;
            data_q[d_top] = 6;
            d_top++; d_level++;
            break;    
        case ACU_Cell_Data_5:
            if(data_full) return;
            data_q[d_top] = 7;
            d_top++; d_level++;
            break;     
        case ACU_Charger_Control:
            if(charger_full) return;
            charger_q[c_top] = 1;
            c_top++; c_level++;
            break;
        // case ACU_DC_DC_Status:
            // deprecated for now
            // break;  
        default:
            break;
    }
}

/// @brief All cell voltages added up and returns sum as float
/// @param acu 
/// @return total voltage as a float
float get_total_voltage(ACU* acu){
    float total = 0.0;
    for(int i = 0; i < NUM_TOTAL_IC; i++){
        for(int j = 0; j < NUM_CELL_IC; j++){
            total += acu->bty->cell_volt[i*NUM_CELL_IC+j];
        }
    }
    return total;
}

/// @brief Constraints the value and casts to uint8_t
/// @param min 
/// @param max 
/// @param value 
/// @return the uint8_t
uint8_t fconstrain(float value){
    if(value > 255.0) value = 255.0;
    else if (value < 0.0) value = 0.0;
    return (uint8_t)value;
}

/// @brief updates adc_data[]
/// @param acu 
void update_adc_data(ACU* acu){
    acu->ts_current = adc_data[0];
    acu->ts_voltage = adc_data[1];
    acu->sdc_volt_w = adc_data[2];
    acu->sdc_volt_v = adc_data[3];
    acu->voltage_12v = adc_data[4];
    acu->water_sense = adc_data[5];
}