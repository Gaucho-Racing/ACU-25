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
    return;
}
void configure_tx_header(ACU * acu, uint32_t id){
    TxHeader.Identifier = id; // Ping ID
}

// check if the values are correct
void acu_check(ACU * acu){
    update_all(acu);
    // bool hasErrors = false;
    return;
}

bool can_polling(ACU * acu){
    
    // Configure TxHeader
    TxHeader.Identifier = 0x321; // Ping ID
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // Polling for received messages
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1) > 0) {
        
        // FIFO has data, read the message
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, CAN_RxBuffer) == HAL_OK) {
            print_lpuart("Received data from FD_CAN!");
            return true;
        }
    }
    // HAL_Delay(100);
    return false;
}
void can_read(ACU * acu, uint32_t id){
    switch (id){
        case Charger_Data_ACU:
            break;  
        case Debug_2_ACU:
            break;  
        case Debug_FD_ACU:
            break;  
        case Ping_ACU:
            can_send(acu, ACU_Ping_Debug);
            break;  
        case Precharge_ACU:
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
    }
}
void can_send(ACU * acu, uint32_t id){

    TxHeader.Identifier = id;

    switch (id){
        case ACU_Debug_2:
            break;       
        case ACU_Debug_FD:
            break;     
        case ACU_Ping_Debug:
            print_lpuart("send ACU_Ping_Debug...\n");
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN_TxBuffer) != HAL_OK) {
                print_lpuart("ACU_Ping_Debug failed...\n");
            }
            break;    
        case ACU_Ping_ECU:
        break;       
        case ACU_Status_1:
        break;       
        case ACU_Status_2:
        break;       
        case ACU_Status_3:
        break;       
        case ACU_Cell_Data_1:
        break;    
        case ACU_Cell_Data_2:
        break;    
        case ACU_Cell_Data_3:
        break;    
        case ACU_Cell_Data_4:
        break;    
        case ACU_Cell_Data_5:
        break;    
        case ACU_DC_DC_Status:
        break;   
        case ACU_Charger_Control:
        break;
        default:
            break;
    }
}
void can_dump(ACU *acu){
    for (uint8_t i = 0; i < NUM_TOTAL_IC; i++) {
        // can_send(acu, Cell_Voltages);
    }
    // can_send(acu, ACU_General);
    // can_send(acu, ACU_General2);
    // can_send(acu, Powertrain_Cooling);
    // can_send(acu, Charging_Cart_Config);
    // can_send(acu, IMD_Request);
    // sendCANData(IMD_Isolation_Detail);
    // sendCANData(IMD_Voltage);
    // sendCANData(IMD_IT_System);
}

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