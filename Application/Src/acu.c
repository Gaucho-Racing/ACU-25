#include "acu.h"

extern uint8_t get_state();
extern void print_lpuart(char* arr);
 
extern bool check_ts_active;
extern char print_buffer[1000];
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
extern volatile uint8_t prim_q[256], data_q[256], charger_q[256];
extern volatile uint8_t CAN_1_flag, CAN_2_flag, CAN_3_flag;

data_union unicorn;

/// @brief converts data stored in ACU to one that can be sent over CAN bus
/// @param offset offset
/// @param source the float that ACU stores
/// @param size the number of bytes to copy
/// @param type the type of number we want to convert to
void prepare_can_send(uint8_t offset, float source, uint8_t type){
    if(type == UNSIGNED_16){
        uint16_t tempVar = (uint16_t)source;
        memcpy(&CAN_TxData[offset], &tempVar, sizeof(uint16_t));
    }
    else{
        int16_t tVar = (int16_t)source;
        memcpy(&CAN_TxData[offset], &tVar, sizeof(int16_t));
    }
}

/// @brief initialize acu variables, reads data, updates adc
/// @param acu 
void acu_init(ACU * acu){

    acu->relay_state = 0;
    acu->em->em_current = 0;
    acu->em->em_voltage = 0;
    acu->em->energy = 0;
    acu->em->min_temp = 0;
    acu->em->max_temp = 0;
    acu->em->num_sensors = 0;
    acu->em->status = 2;

    acu->imd->id = 0;
    acu->imd->hv_system_voltage = 0;
    acu->imd->r_iso_status = 0;
    acu->imd->r_iso_meas_count = 0;

    acu->imd->isolation_quality = 0;
    acu->imd->status_device_activity = 0; 
    acu->imd->r_iso_corrected = 0;
    acu->imd->status_warnings_alarms = 0;

    acu->imd->r_iso_negative = 0;
    acu->imd->r_iso_positive = 0;
    acu->imd->r_iso_original = 0;
    acu->imd->iso_meas_count = 0;

    acu->chgr->chgr_status = 0;
    acu->chgr->chgr_state = 0;
    acu->chgr->charger_output_current = 0;
    acu->chgr->charger_output_voltage = 0;

    acu->lastChrgRecieveTime = 0.0f;
    acu->chg_ctrl = NO_CHARGE;
    acu->acuErrCount = 0;
    acu->acu_err_warns = 0;
    
    acu->target_voltage = 0.0f;
    acu->bat_soc = 0.0f;

    // fix later
    // uint8_t count = 0;
    // while (fabsf(get_total_voltage(acu) - 1.235f) > ERRMG_ISNS_VREF) {
    //     // HV current too far from zero. Check hardware.
    //     if (count > 10) {
    //         for (uint8_t i = 0; i < NUM_TOTAL_IC; i++){
    //             acu->bty->stack_voltage[i] = 1.235f; 
    //         }
    //         break;
    //     }
    //     count++;
    //     LL_mDelay(500);
    // }
}

/// @brief ACU check => current, glv voltage, shut down voltage, warnings
/// @param acu 
/// @param state 
/// @param startup 
/// @return True if passes, False otherwise
bool acu_check(ACU * acu, bool startup){

    // clean the slate
    acu->acu_err_warns &= ~(ACU_CLEAR_ERRR);
    acu->acu_err_warns &= ~(ACU_CLEAR_WARN);
    
    uint8_t lastAcuErrCount = acu->acuErrCount;
    
    bool hasErrors = false;

    // (2) check overcurrent
    if(acu->ts_current > MAX_HV_CURRENT){
        print_lpuart("Max HV Current detected\n");
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

    //glv voltage => TODO: check this
    if(acu->voltage_12v < 3/*MIN_GLV_VOLT*/){
        print_lpuart("GLV Undervolt detected\n");
        acu->acuErrCount++;
        hasErrors = true;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_err_warns |= ACU_ERR_UNDER_VOLT;
        }
    }
    if(acu->voltage_12v > MAX_GLV_VOLT){
        print_lpuart("GLV Overvolt detected\n");
        acu->acuErrCount++;
        hasErrors = true;
        if (acu->acuErrCount >= ERRMG_ACU_ERR){
            acu->acuErrCount = ERRMG_ACU_ERR;
            acu->acu_err_warns |= ACU_ERR_OVER_VOLT;
        }
    }

    // sdc_volt_v should be close to voltage_12v (glv_voltage)
    if(fabsf(acu->sdc_volt_v - acu->voltage_12v) > GLV_SDC_LOW && !startup && get_state() == 3){
        print_lpuart("SD Volt not close enough to GLV\n");

        if(acu->sdc_volt_v < acu->voltage_12v) {
            print_lpuart("ACU_ERR_UNDER_VOLT detected\n");
            acu->acuErrCount++;
            hasErrors = true;
            if (acu->acuErrCount >= ERRMG_ACU_ERR){
                acu->acuErrCount = ERRMG_ACU_ERR;
                acu->acu_err_warns |= ACU_ERR_UNDER_VOLT;
            }
        }
        else if(acu->sdc_volt_v > acu->voltage_12v){
            print_lpuart("ACU_ERR_OVER_VOLT detected\n");
            acu->acuErrCount++;
            hasErrors = true;
            if (acu->acuErrCount >= ERRMG_ACU_ERR){
                acu->acuErrCount = ERRMG_ACU_ERR;
                acu->acu_err_warns |= ACU_ERR_OVER_VOLT;
            }
        }
    }

    if(acu->water_sense < 1.5f){
        // TODO: turn off pump & high voltage
        // hasErrors = true;
        // acu->acuErrCount++;
        turn_of_hv_pump(0);
        print_lpuart("Error in water sense! Ur cooked! [Ignoring for now]\n");
    }

    acu->acuErrCount = (lastAcuErrCount == acu->acuErrCount && !hasErrors)? 0 : acu->acuErrCount;

    if(acu->voltage_12v < UNDER_VOLTAGE_GLV){
        acu->acu_err_warns |= ACU_ERR_UV_12_V;
    }
    if(acu->sdc_volt_v < UNDER_VOLTAGE_SDCV){
        acu->acu_err_warns |= ACU_ERR_UV_SDC;
    }

    // IMD iso failure
    if (acu->imd->status_warnings_alarms > 0 && acu->imd->status_warnings_alarms < 2048){
        
        // TODO: uncomment this later
        // hasErrors = true; undo this for now

        print_imd_err_warn(acu);
        print_lpuart("IMD failures exist!\n");
        acu->imd->status_warnings_alarms = 0; 
    }
    return !hasErrors;
}

/// @brief convert byte arrays to float
/// @param data array of bytes
/// @param size size of array
/// @return 
float magical_union_flt(uint8_t data[], uint8_t size, bool big_endian){
    memset(&unicorn, 0, sizeof(unicorn));
    if (big_endian == false){
        for(size_t i = 0; i < size; i++){
            unicorn.byts[i] = data[i];
        }
    }
    else{
        for(size_t i = 0; i < (int)size; i++){
            unicorn.byts[size-i-1] = (int)data[i];
        }
    }
    return unicorn.flt;
}

/// @brief Charger Data Receive: Calculating Bytes 1-4
/// @param data data being sent/received, 2 bytes total
/// @param weird true if weird
/// @return uint16_t value to store
int16_t magical_union_chgr_rcv(uint8_t data[], bool weird){
    memset(&unicorn, 0, sizeof(unicorn));
    if(!weird){
        unicorn.byts[0] = data[1];
        unicorn.byts[1] = data[0];
    }
    else{
        unicorn.byts[0] = data[1];
        unicorn.byts[1] = data[0] & 0x7F; // mask out the "highest bit" => direction
    }
    return unicorn.i16;
}

void magical_union_chgr_send(uint8_t * buffer, float data){
    memset(&unicorn, 0, sizeof(unicorn));
    uint16_t rounded = roundf(data);
    unicorn.u16 = rounded;
    buffer[0] = unicorn.byts[1];
    buffer[1] = unicorn.byts[0];
}

/// @brief convert byte arrays to uint16_t
/// @param data array of bytes
/// @return 
uint16_t magical_union_u16(uint8_t data[]){
    memset(&unicorn, 0, sizeof(unicorn));
    unicorn.byts[0] = data[0];
    unicorn.byts[1] = data[1];
    return unicorn.u16;
}

int32_t magical_union_i32(uint8_t data[], bool big_endian){
    memset(&unicorn, 0, sizeof(unicorn));
    if(big_endian == false){
        unicorn.byts[0] = data[0];
        unicorn.byts[1] = data[1];
        unicorn.byts[2] = data[2];
        unicorn.byts[3] = data[3];
    }
    else{
        unicorn.byts[0] = data[3];
        unicorn.byts[1] = data[2];
        unicorn.byts[2] = data[1];
        unicorn.byts[3] = data[0];
    }
    return unicorn.i32;
}

/// @brief Checks CAN_RxBuffer & parses messages in FIFO order
/// @attention Case: too many messages to parse, maybe we have some MAX_PARSE val?
/// @param acu 
void can_read_handler(ACU* acu){
    while(CAN_RxBufferLevel > 0){
        FDCAN_GlobalTypeDef * type = CAN_RxBuffer[CAN_RxBufferBottom].instance;
        uint32_t id = CAN_RxBuffer[CAN_RxBufferBottom].identifier;
        can_read(acu, type, id, (uint8_t *)(&CAN_RxBuffer[CAN_RxBufferBottom].data));
        bzero((void*)CAN_RxBuffer[CAN_RxBufferBottom].data, sizeof(CAN_RxBuffer[CAN_RxBufferBottom].data));
        CAN_RxBufferBottom++;
        CAN_RxBufferLevel--;
    }
}

/// @brief Parses a single CAN message
/// @param acu avu in question
/// @param which_can between FDCAN_1, FDCAN_2, FDCAN_3
/// @param id CAN Message ID
/// @param size number of bytes
/// @param data data in question
void can_read(ACU * acu, FDCAN_GlobalTypeDef * which_can, uint32_t id, uint8_t * data){
    uint32_t curr = HAL_GetTick();
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
            acu->ts_active = data[0] & 1U; // @details: command to precharge
            check_ts_active  = true;
            if(get_state() != 1){
                print_lpuart("[WARNING]: Precharge msg when != PRECHARGE???\n");
            }
            break;
        case Charger_Data_ACU:
            acu->lastChrgRecieveTime = curr;
            acu->chgr->charger_output_voltage = (int16_t)(magical_union_chgr_rcv(data, false) * 0.1);   
            acu->chgr->charger_output_current = (int16_t)(magical_union_chgr_rcv(data+2, true) * 0.1f);
            acu->chgr->chgr_state = ((data+2)[0] & 0x80);
            acu->chgr->chgr_status = data[4];
            break;  
        case Config_Charge_ACU:
            acu->target_voltage = magical_union_flt(data, 2, false) * 0.1f;
            acu->target_current = magical_union_flt(data+2, 2, false) * 0.1f;
            break;  
        case Config_Ops_ACU:
            acu->bty->min_volt_thresh = magical_union_flt(data, 2, false) * 0.1f;
            acu->bty->max_temp_thresh = magical_union_flt(data+2, 2, false) * 0.1f;
            break;  
        case EM_Measurements_ACU: // @remark: double check this
            acu->em->em_current = magical_union_flt(data, 4, true);
            acu->em->em_voltage = magical_union_flt(data+4, 4, true);
            break;  
        case EM_Data_1_ACU:
            acu->em->team_data[0] = magical_union_i32(data, true);
            acu->em->team_data[1] = magical_union_i32(data+4, true);
            break;  
        case EM_Data_2_ACU:
            acu->em->team_data[2] = magical_union_i32(data, true);
            acu->em->team_data[3] = magical_union_i32(data+4, true);
            break;  
        case EM_Status_ACU:
            acu->em->status = data[0];
            acu->em->energy = magical_union_flt(data+1, 4, true);
            break;  
        case EM_Temperature_ACU:
            uint8_t mux_signal = data[0] & 0b11100000;
            acu->em->num_sensors = (data[0] & 0b00011111) << 3;

            acu->em->min_temp = (uint8_t)(data[1] * 0.5f);
            acu->em->max_temp = (uint8_t)(data[2] * 0.5f);

            acu->em->temps[mux_signal*5] = (float)(data[3] * 0.5f);
            acu->em->temps[mux_signal*5+1] = (float)(data[4] * 0.5f);
            
            if(mux_signal != 6){
                acu->em->temps[mux_signal*5+2] = (float)(data[5] * 0.5f);
                acu->em->temps[mux_signal*5+3] = (float)(data[6] * 0.5f);
                acu->em->temps[mux_signal*5+4] = (float)(data[7] * 0.5f);
            }
            break;  
        case IMD_Response_ACU:
            acu->imd->id = data[0];
            break;  
        case IMD_Isolation_ACU: 
            acu->imd->r_iso_negative = magical_union_u16(data);
            acu->imd->r_iso_positive = magical_union_u16(data+2);
            acu->imd->r_iso_original = magical_union_u16(data+4);
            acu->imd->iso_meas_count = data[6];
            acu->imd->isolation_quality = data[7];
            break;  
        case IMD_Voltage_ACU:
            acu->imd->hv_system_voltage = (magical_union_u16(data) - 32128) * 0.05f;
            break;  
        case IMD_IT_System_ACU: 
            break;
        case IMD_Request_ACU:
            acu->imd->id = data[0];
            break;  
        case IMD_General_ACU:
            acu->imd->r_iso_corrected = magical_union_u16(data);
            acu->imd->r_iso_status = data[2];
            acu->imd->r_iso_meas_count = data[3];
            acu->imd->status_warnings_alarms = magical_union_u16(data+4);
            acu->imd->status_device_activity = data[6];
            break;
        default:
            break;
    }
}

/// @brief FIFO transfer of CAN messages
/// @param acu 
void dequeue(ACU* acu){
    if(CAN_1_flag == 0 && CAN_2_flag == 0 && CAN_3_flag == 0){
        return;
    }
    // priority 1: CAN_Primary
    switch(CAN_1_flag){ 
        case 1: // ACU_Debug_2_Debug
            memcpy(CAN_TxData, CAN_RxData, 8*sizeof(uint8_t));
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
 
            CAN_TxData[0] = (((uint16_t)((acu->bty->battery_total_voltage) * 100.0f)) & 0xFF); 
            CAN_TxData[1] = ((((uint16_t)((acu->bty->battery_total_voltage) * 100.0f)) >> 8) & 0xFF); 
            CAN_TxData[2] = (uint8_t)((uint16_t)(acu->ts_voltage * 100.0f) & 0xFF); 
            CAN_TxData[3] = (uint8_t)((uint16_t)(acu->ts_voltage * 100.0f) >> 8) & 0xFF;
            CAN_TxData[4] = (uint8_t)((int8_t)(acu->ts_current * 100.0f) & 0xFF);         
            CAN_TxData[5] = (((uint16_t)(acu->ts_current * 100.0f) >> 8) & 0xFF);  
            CAN_TxData[6] = (uint8_t)(calculate_acu_soc(acu) * 51.0f * 0.2f);
            CAN_TxData[7] = (uint8_t)(calculate_glv_soc(acu) * 51.0f * 0.2f);
            if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxData) != HAL_OK){
                print_lpuart("ACU_Status_1 failed...\n");
            }
            CAN_1_flag = 0;
            break;
        case 6: // ACU_Status_2
            TxHeader.Identifier = ACU_Status_2;
            TxHeader.DataLength = FDCAN_DLC_BYTES_7;
            CAN_TxData[0] = 0; // 20v GLV voltage
            CAN_TxData[1] = (uint8_t)(acu->voltage_12v * 10.0f);
            CAN_TxData[2] = (uint8_t)(acu->sdc_volt_v * 10.0f);
            CAN_TxData[3] = (uint8_t)((acu->bty->min_cell_volt - 2.0f) * 100.0f);
            CAN_TxData[4] = (uint8_t)(acu->bty->max_cell_temp * 4.0f);
            CAN_TxData[5] = (uint8_t)(acu->acu_err_warns & 0xFF); // takes [OT, OV, UV, OC, UC, UV_20v, UV_GLV, UV_SDC]
            CAN_TxData[6] = acu->acu_err_warns & ACU_PRECHARGE >> 7; // takes precharge error to lsb
            CAN_TxData[6] |= (acu->relay_state & AIR_MINUS) >> 1;
            CAN_TxData[6] |= (acu->relay_state & AIR_PLUS) >> 1;
            CAN_TxData[6] |= (acu->relay_state & ACU_LATCH) << 3; 

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
            TxHeader.Identifier = ACU_Ping_ECU; // @attention check if it's sent through CAN1 or CAN2
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Ping_ECU > hfdcan2 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 3: // ACU_Cell_Data_1
            TxHeader_Data.Identifier = ACU_Cell_Data_1;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;

            // cell_volt range: [0:32]
            // cell_temp range: [0:64]
            for(uint8_t cell = 0U; cell < 32U; cell++){
                float max_temp = fmaxf(acu->bty->cell_temp[cell*2]*4.0f, acu->bty->cell_temp[cell*2+1]*4.0f);
                CAN_TxData[2*cell] = fconstrain((acu->bty->cell_volt[cell] - 2.0f) * 100.0f);
                CAN_TxData[2*cell+1] = fconstrain(max_temp);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_1 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 4: // ACU_Cell_Data_2
            TxHeader_Data.Identifier = ACU_Cell_Data_2;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;

            // cell_volt range: [32:64]
            // cell_temp range: [64:128]
            for(uint8_t cell = 0U; cell < 32U; cell++){
                float max_temp = fmaxf(acu->bty->cell_temp[cell*2+64]*4.0f, acu->bty->cell_temp[cell*2+64+1]*4.0f);
                CAN_TxData[2*cell] = fconstrain((acu->bty->cell_volt[cell+32] - 2.0f) * 100.0f);
                CAN_TxData[2*cell+1] = fconstrain(max_temp);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
            print_lpuart("ACU_Cell_Data_2 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 5: // ACU_Cell_Data_3
            TxHeader_Data.Identifier = ACU_Cell_Data_3;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;

            // cell_volt range: [64:96]
            // cell_temp range: [128:192]
            for(uint8_t cell = 0U; cell < 32U; cell++){
                float max_temp = fmaxf(acu->bty->cell_temp[cell*2+128]*4.0f, acu->bty->cell_temp[cell*2+128+1]*4.0f);
                CAN_TxData[2*cell] = fconstrain((acu->bty->cell_volt[cell+64] - 2.0f) * 100.0f);
                CAN_TxData[2*cell+1] = fconstrain(max_temp);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_3 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 6: // ACU_Cell_Data_4
            TxHeader_Data.Identifier = ACU_Cell_Data_4;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;

            // cell_volt range: [96:128]
            // cell_temp range: [192:256]
            for(uint8_t cell = 0U; cell < 64U; cell+=2){
                // float max_temp = fmaxf(acu->bty->cell_temp[cell*2+128]*4.0f, acu->bty->cell_temp[cell*2+128+1]*4.0f);
                // CAN_TxData[2*cell] = fconstrain((acu->bty->cell_volt[cell+64] - 2.0f) * 100.0f);
                // CAN_TxData[2*cell+1] = fconstrain(max_temp);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_4 failed...\n");
            }
            CAN_2_flag = 0;
            break;
        case 7: // ACU_Cell_Data_5
            TxHeader_Data.Identifier = ACU_Cell_Data_5;
            TxHeader_Data.DataLength = FDCAN_DLC_BYTES_64;
            for(uint8_t cell = 0U; cell < 12U; cell+=2){
                // float max_temp = fmaxf(acu->bty->cell_temp[2*cell+256]*4.0f, acu->bty->cell_temp[2*cell+256+1]*4.0f);
                // CAN_TxData[cell] = fconstrain((acu->bty->cell_volt[cell+128] - 2.0f) * 100.0f);
                // CAN_TxData[cell+1] = fconstrain(max_temp);
            }
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Data, CAN_TxData) != HAL_OK) {
                print_lpuart("ACU_Cell_Data_5 failed...\n");
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
            magical_union_chgr_send(CAN_TxData, acu->target_voltage * 10.0f);
            magical_union_chgr_send(CAN_TxData+2, acu->target_current * 10.0f);
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
    __disable_irq();
    uint8_t primary_full = (p_level >= 255U);
    uint8_t data_full = (d_level >= 255U);
    uint8_t charger_full = (c_level >= 255U);
     __enable_irq();
    switch (id){
        case ACU_Debug_2_Debug:
            if(primary_full) {
                return;
            }
            __disable_irq();
            prim_q[p_top] = 1;
            p_top++; p_level++;
            __enable_irq();   
            break;       
        case ACU_Ping_Debug:
            if(which_can == FDCAN1){
                if(primary_full) {
                    return;
                }
                __disable_irq();
                prim_q[p_top] = 2;
                p_top++; p_level++;
                __enable_irq();   
            }
            else if (which_can == FDCAN2){
                if(data_full) {
                    return;
                }
                __disable_irq();
                data_q[d_top] = 1;
                d_top++; d_level++;
                __enable_irq();  
            }
            break;     
        case ACU_Ping_ECU:
            if(which_can == FDCAN1){
                if(primary_full) {
                    return;
                }
                __disable_irq();
                prim_q[p_top] = 3;
                p_top++; p_level++;
                __enable_irq();  
            }
            else if(which_can == FDCAN2){
                if(data_full) {
                    return;
                }
                __disable_irq();
                data_q[d_top] = 2;
                d_top++; d_level++;
                __enable_irq();  
            }
            break;    
        case ACU_Debug_FD:
            if(data_full) {
                return;
            }
            __disable_irq();
            data_q[d_top] = 3;
            d_top++; d_level++;
            __enable_irq();  
            break;       
        case ACU_Status_1:
            // print_lpuart("enqueue ACU_Status_1\n");
            if(primary_full)  {
                return;
            }
            __disable_irq();
            prim_q[p_top] = 5;
            p_top++; p_level++;
            __enable_irq();  
            break;       
        case ACU_Status_2:
            // print_lpuart("enqueue ACU_Status_2\n");
            if(primary_full)  {
                return;
            }
            __disable_irq();
            prim_q[p_top] = 6;
            p_top++; p_level++;
            __enable_irq();  
            break;       
        case ACU_Status_3:
            break;       
        case ACU_Cell_Data_1:
            if(data_full)  {
                return;
            }
            __disable_irq();
            data_q[d_top] = 4;
            d_top++; d_level++;
            __enable_irq();  
            break;    
        case ACU_Cell_Data_2:
            if(data_full) {
                return;
            }
            __disable_irq();
            data_q[d_top] = 5;
            d_top++; d_level++;
            __enable_irq();  
            break;    
        case ACU_Cell_Data_3:
            if(data_full) {
                return;
            }
            __disable_irq();
            data_q[d_top] = 6;
            d_top++; d_level++;
            __enable_irq();  
            break;    
        case ACU_Cell_Data_4:
            if(data_full) {
                return;
            }
            __disable_irq();
            data_q[d_top] = 6;
            d_top++; d_level++;
            __enable_irq();  
            break;    
        case ACU_Cell_Data_5:
            if(data_full) {
                return;
            }
            __disable_irq();
            data_q[d_top] = 7;
            d_top++; d_level++;
            __enable_irq();  
            break;     
        case ACU_Charger_Control:
            if(charger_full)  {
                return;
            }
            __disable_irq();
            charger_q[c_top] = 1;
            c_top++; c_level++;
            __enable_irq();  
            break;
        default:
            print_lpuart("enqueue WTF\n");
            break;
    }
}

/// @brief All cell voltages added up and returns sum as float
/// @param acu 
/// @return total voltage as a float
float get_total_voltage(ACU* acu){
    return acu->bty->battery_total_voltage;
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
    acu->ts_current += ((adc_data[0] * 0.0005f - 1.235f) / 0.005f - acu->ts_current) * 0.05f; // 5mV/A
    acu->ts_voltage += (adc_data[1] * 0.0005f * 400.0f - acu->ts_voltage) * 0.2f; // 1:400 voltage divider
    acu->sdc_volt_w += (adc_data[2] * 0.0005f * 10.0f - acu->sdc_volt_w) * 0.2f; // 1:10 voltage divider
    acu->sdc_volt_v += (adc_data[3] * 0.0005f * 10.0f - acu->sdc_volt_v) * 0.2f; // 1:10 voltage divider
    acu->voltage_12v += (adc_data[4] * 0.0005f * 10.07475f - acu->voltage_12v) * 0.2f; // 1:10 voltage divider
    acu->water_sense += (adc_data[5] * 0.0005f - acu->water_sense) * 0.2f;  // keep raw voltage
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/// @attention @OWEN PLS CHECK IF THIS IS RIGHT
float calculate_acu_soc(ACU* acu){
    // return (acu->bty->min_cell_volt) / acu->target_voltage;
    float cell_open_volt = acu->bty->battery_total_voltage + acu->ts_current * CELL_INT_RESISTANCE * NUM_TOTAL_IC * 16U;
    float zero_chg_volt = NUM_TOTAL_IC * 16U * CELL_EMPTY_VOLTAGE;
    float full_chg_volt = NUM_TOTAL_IC * 16U * CELL_FULL_VOLTAGE;
    acu->bat_soc += (map(cell_open_volt, zero_chg_volt, full_chg_volt, 0.0F, 255.0F) - acu->bat_soc) * 0.1;
    return acu->bat_soc;
}

/// @attention @OWEN PLS CHECK IF THIS IS RIGHT
float calculate_glv_soc(ACU* acu){
    return (acu->voltage_12v) / 12.0f; // ehhh this could also be wrong
}

/// @brief just prints adc_data
/// @param acu
void print_adc_data(ACU *acu){
    print_lpuart("ADC Data: -------------------------------\n");
    bzero(print_buffer, sizeof(print_buffer));
    sprintf(print_buffer, "ts_curr: %.3f | ts_volt: %.3f | sdc_w: %.3f | sdc_v: %.3f | 12v: %.3f | water_sense: %.3f\n", 
        acu->ts_current, acu->ts_voltage, acu->sdc_volt_w,  acu->sdc_volt_v, acu->voltage_12v, acu->water_sense);
    print_lpuart(print_buffer);
}

/// @brief prints imd data
/// @param acu 
void print_imd_data(ACU* acu){
    print_lpuart("IMD Data: -------------------------------\n");
    bzero(print_buffer, sizeof(print_buffer));
    sprintf(print_buffer,
        "R_ISO_Corrected: %hu | R_ISO_Status: %u | ISO_Meas_Count: %u\n"
        "Status_Warnings_Alarms: %hu | Status_Device_Activity: %u | HV_System: %.2f\n",
        (unsigned short int)(acu->imd->r_iso_corrected),
        (unsigned int)(acu->imd->r_iso_status),
        (unsigned int)(acu->imd->iso_meas_count),
        (unsigned short int)(acu->imd->status_warnings_alarms),
        (unsigned int)(acu->imd->status_device_activity),
        acu->imd->hv_system_voltage
    );
    print_lpuart(print_buffer);
    print_lpuart("-----------------------------------------\n");
}

/// @brief prints the errors and warnings of the IMD monitor
/// @param acu 
void print_imd_err_warn(ACU* acu){
    print_lpuart("IMD Err/Warns: --------------------------\n");
    char buff[50];
    bzero(buff, sizeof(buff));
    sprintf(buff, "IMD ERR TYPE %u\n", acu->imd->status_warnings_alarms);
    print_lpuart(buff);

    if(acu->imd->status_warnings_alarms & IMD_ERROR_ACTIVE){
        bzero(buff, sizeof(buff));
        sprintf(buff, "⛔: IMD Error\n");
        print_lpuart(buff);
    }
    if(acu->imd->status_warnings_alarms & HV_POS_CONN_FAIL){
        bzero(buff, sizeof(buff));
        sprintf(buff, "➕: HV pos connection failed\n");
        print_lpuart(buff);
    }
    if(acu->imd->status_warnings_alarms & HV_NEG_CONN_FAIL){
        bzero(buff, sizeof(buff));
        sprintf(buff, "➖: HV neg connection failed\n");
        print_lpuart(buff);
    }
    if(acu->imd->status_warnings_alarms & EARTH_CONNN_FAIL){
        bzero(buff, sizeof(buff));
        sprintf(buff, "🛸: Earth connection failed\n");
        print_lpuart(buff);
    }
    if(acu->imd->status_warnings_alarms & ISO_ALARM_ERRROR){
        bzero(buff, sizeof(buff));
        sprintf(buff, ": 🚨ISO alarm error\n");
        print_lpuart(buff);
    }
    if(acu->imd->status_warnings_alarms & ISO_WARN_ERRRROR){
        bzero(buff, sizeof(buff));
        sprintf(buff, "🇺🇳: ISO warning error\n");
        print_lpuart(buff);
    }
    if(acu->imd->status_warnings_alarms & ISO_OUTDATED_ERR){
        bzero(buff, sizeof(buff));
        sprintf(buff, "👵🏻: ISO outdated...\n");
        print_lpuart(buff);
    }
    if(acu->imd->status_warnings_alarms & UN_BALANCE_ALARM){
        bzero(buff, sizeof(buff));
        sprintf(buff, "⚖️: Unbalanced alarm\n");
        print_lpuart(buff);
    }
    if(acu->imd->status_warnings_alarms & UNDERVOLTG_ALARM){
        bzero(buff, sizeof(buff));
        sprintf(buff, "🪫: Undervoltage alarm\n");
        print_lpuart(buff);
    }
    if(acu->imd->status_warnings_alarms & UNSAFE_TOO_START){
        bzero(buff, sizeof(buff));
        sprintf(buff, "☠️: Unsafe to start\n");
        print_lpuart(buff);
    }
    if(acu->imd->status_warnings_alarms & EARTH_LIFT_OPENN){
        bzero(buff, sizeof(buff));
        sprintf(buff, "🚠: Earth lift open\n");
        print_lpuart(buff);
    }
}

  
void print_relay_status(uint8_t relay){
    print_lpuart("Relay: ----------------------------------\n");
    char buff[100];
    sprintf(buff, "Relay (HEX): %02X\n", relay);
    print_lpuart(buff);
}

/// @brief prints acu target volts & current. 
/// @param acu 
void print_targets(ACU * acu){
    print_lpuart("Targets: --------------------------------\n");
    char buff[100];
    bzero(buff, sizeof(buff));
    sprintf(buff, "Target Voltage: %.3f | Target Current: %.3f\n", 
        acu->target_voltage, acu->target_current);
    print_lpuart(buff);
}

/// @brief prints charger data
/// @param acu 
void print_charger_data(ACU* acu){
    print_lpuart("Charger Data: ---------------------------\n");
    char buff[150];
    bzero(buff, sizeof(buff));
    sprintf(buff,
        "Charger voltage: %hd |Charger current: %hd\n",
        acu->chgr->charger_output_voltage,
        acu->chgr->charger_output_current
    );
    print_lpuart(buff);
    print_lpuart("-----------------------------------------\n\n");

    print_lpuart("Charger Err/Warns: ----------------------\n");
    char bufff[50];
    if(acu->chgr->chgr_status & CHARGER_HW_FAIL){
        bzero(bufff, sizeof(bufff));
        sprintf(bufff, "⚙️: Hardware Failure\n");
        print_lpuart(bufff);
    }
    if(acu->chgr->chgr_status & CHARGER_OV_TEMP){
        bzero(bufff, sizeof(bufff));
        sprintf(bufff, "🥵: Over Temp\n");
        print_lpuart(bufff);
    }
    if(acu->chgr->chgr_status & CHARGER_IN_VOLT){
        bzero(bufff, sizeof(bufff));
        sprintf(bufff, "⚡: Wrong input voltage\n");
        print_lpuart(bufff);
    }
    if(acu->chgr->chgr_status & CHARGER_CONNECT){
        bzero(bufff, sizeof(bufff));
        sprintf(bufff, "🐻‍❄️: Wrong polarity or NC\n");
        print_lpuart(bufff);
    }
    if(acu->chgr->chgr_status & CHARGER_COOMMMM){
        bzero(bufff, sizeof(bufff));
        sprintf(bufff, "⏱️: Timeout\n");
        print_lpuart(bufff);
    }
}

/// @brief prints acu errors & warnings
/// @param acu 
void print_errors_warning(ACU * acu){
    print_lpuart("Err/Warns: ------------------------------\n");
    char buff[30];
    if(acu->acu_err_warns & ACU_ERR_OVER_TEMP){
        bzero(buff, sizeof(buff));
        sprintf(buff, "Error: ACU Overtemperature\n");
        print_lpuart(buff);
    }
    if(acu->acu_err_warns & ACU_ERR_OVER_VOLT){
        bzero(buff, sizeof(buff));
        sprintf(buff, "Error: ACU Overvoltage\n");
        print_lpuart(buff);
    }
    if(acu->acu_err_warns & ACU_ERR_UNDER_VOLT){
        bzero(buff, sizeof(buff));
        sprintf(buff, "Error: ACU Undervoltage\n");
        print_lpuart(buff);
    }
    if(acu->acu_err_warns & ACU_ERR_OVER_CURR){
        bzero(buff, sizeof(buff));
        sprintf(buff, "Error: ACU Overcurrent\n");
        print_lpuart(buff);
    }
    if(acu->acu_err_warns & ACU_PRECHARGE){
        bzero(buff, sizeof(buff));
        sprintf(buff, "Error: ACU Precharge Issue\n");
        print_lpuart(buff);
    }
    if(acu->acu_err_warns & ACU_ERR_UV_20_V){
        bzero(buff, sizeof(buff));
        sprintf(buff, "Error: Undervoltage for 20v\n");
        print_lpuart(buff);
    }
    if(acu->acu_err_warns & ACU_ERR_UV_12_V){
        bzero(buff, sizeof(buff));
        sprintf(buff, "Error: Undervoltage for 12v\n");
        print_lpuart(buff);
    }
    if(acu->acu_err_warns & ACU_ERR_UV_SDC){
        bzero(buff, sizeof(buff));
        sprintf(buff, "Error: Undervoltage for SDC\n");
        print_lpuart(buff);
    }
    print_lpuart("-----------------------------------------\n");
}

/// @brief writes the BMS OK signal to PC8
/// @param state
void write_bms_ok(bool state){
    if (state){
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);
    }
    else{
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
    }
}

/// @brief writes the IR- signal to PC5
/// @param state
void write_IRneg(bool state){
    if (state){
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_5);
    }
    else{
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_5); 
    }
}

/// @brief writes the IR+ signal to PC4
/// @param state
void write_IRpos(bool state){
    if (state){
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);
    }
    else{ 
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4);
    }
}

/// @brief writes the precharge relay signal to PB0
/// @param state
void write_prechg(bool state){
    if (state){
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0);
    }
    else{
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
    }
}

/// @brief writes the acu latch signal to PB1
/// @param state
void write_acu_latch(bool state){
    if(state){
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
    }
    else{
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
    }
}

// deal wiht this later
void turn_of_hv_pump(bool state){
    // deal wiht this later
}

/// @brief writes the debug LED signal to PA15
/// @param state
void write_LED(bool state){
    if (state){
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);
    }
    else{
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);
    }
}

/// @brief resets (switch on) the ACU SDC latch (connect SDC_W to SDC_V)
void sdc_reset(){
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
    LL_mDelay(1);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
}

/// @brief updates acu.relay_state based on relay control pins levels
/// @param acu
void update_relay_state(ACU* acu){
    acu->relay_state = 0;
    if(LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_4)){
        acu->relay_state |= AIR_PLUS;
    }
    if(LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_5)){
        acu->relay_state |= AIR_MINUS;
    }
    if(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_0)){
        acu->relay_state |= RELAY_PRE;
    }
    if(!LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_1)){
        acu->relay_state |= ACU_LATCH;
    }
}