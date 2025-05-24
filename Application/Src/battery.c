#include "battery.h"
#include "state.h"

extern uint8_t cycle;
extern bcc_status_t bcc_error;
extern uint8_t bcc_cooked_count;
extern volatile uint8_t TPL_RxBuffer[256]; // Array to store received SPI data => Replace with struct holding CAN RxBuffer
extern volatile uint8_t TPL_RxBufferLevel; // Number of bytes to be read
extern volatile uint8_t TPL_RxBufferBottom; // Index of oldest data

extern void set_state(uint8_t value);
extern void print_lpuart(char* arr);
extern void write_LED(bool state);
extern void print_bcc_status(bcc_status_t stat);

/// @brief for bcc spi read
/// @param buffer 
/// @param length 
/// @return 0 for success, 1 for FAILURE 🤦‍♂️
uint8_t bcc_read_string(uint8_t *buffer, uint16_t length){
    for (uint16_t i = 0; i < length; i++) {
        uint32_t counter = 0;
        while (TPL_RxBufferLevel == 0) {
            BCC_MCU_WaitUs(1);
            if(counter++ > SPI_LOOP_TIMEOUT) {
                // print_lpuart("spi timeout from loop\n");
                return 1;
            }
        }
        __disable_irq();
        buffer[i] = TPL_RxBuffer[TPL_RxBufferBottom];
        TPL_RxBufferBottom++;
        TPL_RxBufferLevel--;
        __enable_irq();
    }
    return 0;
  }
  
  /// @brief for bcc spi send
  /// @param data 
  /// @param length 
  /// @return 0 for success, 1 for FAILURE 🤦‍♂️
uint8_t bcc_send_string(const uint8_t *data, uint16_t length) {
    uint32_t counter = 0;
    BCC_MCU_WriteCsbPin(0, 0); // CS LOW
    BCC_MCU_WaitUs(2); // delay required by MC33664
    while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {
        if(counter++ > SPI_LOOP_TIMEOUT) return 1;
        BCC_MCU_WaitUs(1);
    }
    for (uint16_t i = 0; i < length; i++) {
        while(!LL_SPI_IsActiveFlag_TXE(SPI1));
        LL_SPI_TransmitData8(SPI1, data[i]);
    }
    
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    BCC_MCU_WaitUs(1); // delay required by MC33664
    BCC_MCU_WriteCsbPin(0, 1); // CS HIGH
    return 0;
}
  

void clear_faults(bcc_drv_config_t * drvConfig)
{
    for (uint8_t ic = 1; ic <= NUM_CELL_IC; ic++)
    {
        bcc_cid_t cid = (bcc_cid_t)ic;
        bcc_error = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_CELL_OV);
        bcc_error = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_CELL_UV);
        bcc_error = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_CB_OPEN);
        bcc_error = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_CB_SHORT);
        bcc_error = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_GPIO_STATUS);
        bcc_error = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_AN_OT_UT);
        bcc_error = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_GPIO_SHORT);
        bcc_error = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_COMM);
        bcc_error = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_FAULT1);
        bcc_error = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_FAULT2);
        bcc_error = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_FAULT3);
        if(bcc_error != BCC_STATUS_SUCCESS) return;
    }
}

bool init_registers(Battery * bty)
{
    uint8_t cid, i;
    for (cid = 1; cid <= bty->drvConfig.devicesCnt; cid++)
    {
        for (i = 0; i < INIT_REG_CNT; i++)
        {
            if (init_regs[i].value != init_regs[i].defaultVal)
            {
                bcc_error = BCC_Reg_Write(&(bty->drvConfig), (bcc_cid_t)cid,
                        init_regs[i].address, init_regs[i].value);
                    if(bcc_error != BCC_STATUS_SUCCESS) return false;
            }
        }
    }
    return true;
}


void print_temp(const float * temperatures, const uint8_t cid, uint8_t index){
    char printBuff[32];
    sprintf(printBuff, "Row %u, Cell %u: %.3f\n", cid, index, temperatures[index]);
    print_lpuart(printBuff);
}

void print_volt(const float * voltages, const uint8_t cid, uint8_t index){
    char printBuff[32];
    sprintf(printBuff, "Row %u, Cell %u: %.3f\n", cid, index, voltages[index]);
    print_lpuart(printBuff);
}

/// @brief turn off cell_balancing for all cells
/// @param bty 
// TODO: review this
void reset_discharge(Battery * bty){
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++)
    {
        for(uint8_t j = 0; j < NUM_CELL_IC; j++){
            if ((bcc_error = config_cell_balancing(bty, (bcc_cid_t)(i), j, 0, 0))!= BCC_STATUS_SUCCESS){ // resets after default = 30 seconds
                print_bcc_status(bcc_error);
                set_state(SHITDOWN);
                return;
            }
        }
    }
}

/// @brief 
/// @param bty 
/// @param read_volt True if we're just reading voltages
/// @param read_temp True if we're just reading temps
/// @return SUCCESS if read was successful, else failed
bcc_status_t read_device_measurements(Battery * bty, uint8_t read_volt, uint8_t read_temp) 
{
    uint32_t measurements[NUM_CELL_IC];
    int16_t temp_measures[NUM_CELL_IC];

    bzero(measurements, sizeof(measurements));
    bty->max_cell_volt = 0;
    bty->max_cell_temp = 0;
    bty->min_cell_temp = 10000.0f;
    bty->min_cell_volt = 10000.0f;
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        BCC_CB_Pause(&(bty->drvConfig), (bcc_cid_t)(i+1), true); // pause b4 read

        if(read_volt){

            // CELL VOLTAGES
            BCC_Meas_StartAndWait(&(bty->drvConfig), (bcc_cid_t)(i+1), BCC_AVG_1);
            bcc_error = BCC_Meas_GetCellVoltages(&(bty->drvConfig), (bcc_cid_t)(i+1), measurements);
            if(bcc_error != BCC_STATUS_SUCCESS) {
                bcc_cooked_count++;
                if(bcc_cooked_count == 0){
                    print_lpuart("\nerror in BCC_Meas_GetCellVoltages: ");
                    print_bcc_status(bcc_error);
                    set_state(INIT);
                    #if DEBUGG == 0
                    return bcc_error;
                    #endif
                }
            }
            for (uint8_t j = 0; j < NUM_CELL_IC; j++){
                bty->cell_volt[(i*NUM_CELL_IC)+j] = (measurements[j] * 1e-6f);
                bty->max_cell_volt = fmaxf(bty->max_cell_volt, bty->cell_volt[(i*NUM_CELL_IC)+j]);
                bty->min_cell_volt = fminf(bty->min_cell_volt, bty->cell_volt[(i*NUM_CELL_IC)+j]);
            }

            bzero(measurements, sizeof(measurements));
            bcc_error = BCC_Meas_GetStackVoltage(&(bty->drvConfig), (bcc_cid_t)(i+1), measurements);
            bty->stack_voltage[i] = *measurements * 1e-6f; // theres ~ .3 difference from manual calcs
            
            if(bcc_error != BCC_STATUS_SUCCESS) {
                bcc_cooked_count++;
                if(bcc_cooked_count == 0){
                    print_lpuart("\nerror in BCC_Meas_GetStackVoltage: ");
                    print_bcc_status(bcc_error);
                    set_state(INIT);
                    #if DEBUGG == 0
                    return bcc_error;
                    #endif
                }
            }
        }

        
        if(read_temp){
            bzero(temp_measures, sizeof(temp_measures));
            bcc_error = BCC_Meas_GetIcTemperature(&(bty->drvConfig), (bcc_cid_t)(i+1), BCC_TEMP_CELSIUS, temp_measures);
            bty->icTemp[i] = temp_measures[i] * 0.1f;

            if(bcc_error != BCC_STATUS_SUCCESS) {
                bcc_cooked_count++;
                if(bcc_cooked_count == 0){
                    print_lpuart("error in BCC_Meas_GetIcTemperature: ");
                    print_bcc_status(bcc_error);
                    set_state(INIT);
                    #if DEBUGG == 0
                    return bcc_error;
                    #endif
                }
            }
        }

        // CELL TEMPS
        if(true && read_temp){
            for(uint8_t j = 0; j < 32; j++) { // TODO: fix this loop
                uint8_t readByte;
                bcc_error = BCC_EEPROM_Read(&(bty->drvConfig), (bcc_cid_t)(i+1), j+1, &readByte);
                if (bcc_error == BCC_STATUS_SUCCESS) {
                    bty->cell_temp[(i*NUM_CELL_IC)+j] = (float)(readByte * 0.1f);
                    bty->max_cell_temp = fmaxf(bty->max_cell_temp, bty->cell_temp[(i*NUM_CELL_IC)+j]);
                    bty->min_cell_temp = fminf(bty->min_cell_temp, bty->cell_temp[(i*NUM_CELL_IC)+j]);
                }
                if(bcc_error != BCC_STATUS_SUCCESS) {
                    bcc_cooked_count++;
                    if(bcc_cooked_count == 0){
                        print_lpuart("error in BCC_EEPROM_Read: ");
                        print_bcc_status(bcc_error);
                        set_state(INIT);
                        #if DEBUGG == 0
                        return bcc_error;
                        #endif
                    }
                }
            }
        }

        bcc_error = BCC_CB_Pause(&(bty->drvConfig), (bcc_cid_t)(i+1), false); // resume after read
        if (bcc_error != BCC_STATUS_SUCCESS) {
            print_lpuart("failed BCC_CB_Pause: ");
            print_bcc_status(bcc_error);
            #if DEBUGG == 0
            return bcc_error;
            #endif
        }
    }
    return BCC_STATUS_SUCCESS;
}

/// @brief checks which cells need to be discharged
/// @param bty hopefully not a cooked battery
/// @return True if success, False otherwise
// TODO: review this
bool do_cell_balancing(Battery * bty){

    float threshold = (bty->min_cell_volt + bty->max_cell_volt) * 0.5f;
    bcc_error = BCC_STATUS_SUCCESS;
    
    // turn off cell balancing
    reset_discharge(bty); // TODO: check this

    // cell balancing ~
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++)
    {
        uint8_t to_discharge = 0;
        for(uint8_t j = 0; j < NUM_CELL_IC; j++){
            if((bty->cell_volt[i*NUM_CELL_IC+j] > threshold || bty->cell_volt[i*NUM_CELL_IC + j] - bty->min_cell_volt > 0.02f)){
                to_discharge = 1;
            }
        }
        if(to_discharge == 1){
            for(uint8_t j = 0; j < NUM_CELL_IC; j++){
                // resets after default = 30 seconds
                if ((bcc_error = config_cell_balancing(bty, (bcc_cid_t)(i), j, 0, 1))!= BCC_STATUS_SUCCESS){ 
                    print_bcc_status(bcc_error);
                    return false;
                }
            }
        }
        to_discharge = 0;
        
    }
    return true;
}

/// @brief cell_balancing for specific cell
/// @param bty hopefully not a cooked battery
/// @param cid IC
/// @param cellIndex cell from IC
/// @param all configure all?
/// @param enable turn on or off?
/// @return status from configuration
// TODO: review this
bcc_status_t config_cell_balancing(Battery * bty, bcc_cid_t cid, uint8_t cellIndex, bool all, bool enable)
{
    bcc_error = BCC_STATUS_SUCCESS;
    // set all
    if(all){
        for(uint8_t i = 0; i < NUM_TOTAL_IC; i++)
        {
            for(uint8_t j = 0; j < NUM_CELL_IC; j++){

                bcc_status_t errors;
                bty->cell_balancing[(i*NUM_CELL_IC)+j] = 0;
                if((errors = BCC_CB_SetIndividual(&(bty->drvConfig), (bcc_cid_t)(i+1), j, true, 0)) != BCC_STATUS_SUCCESS){
                    print_lpuart("config_cell_balancing: line 273\n");
                    print_bcc_status(errors);
                    bcc_error = errors;
                }
                bty->cell_balancing[(i*NUM_CELL_IC)+j] = enable == true ? 255 : 0;
            }
        }
        return bcc_error;
    }
    else{
        // sanity check
        if(cellIndex >= NUM_CELL_IC){
            print_lpuart("Invalid cell index");
            return bcc_error;
        }

        // set individuals
        if((bcc_error = BCC_CB_SetIndividual(&(bty->drvConfig), (bcc_cid_t)(cid+1), cellIndex, enable, 0)) != BCC_STATUS_SUCCESS) {
            print_lpuart("config_cell_balancing: line 292\n");
            return bcc_error;
        }
        bty->cell_balancing[(uint8_t)cid*cellIndex+cellIndex] = enable == true ? 255 : 0;
    }
    return bcc_error;
}

/// @brief initialize cell balancing
/// @param bty battery
/// @return 0 if failure, 1 if success
bool init_cell_balancing(Battery * bty){
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++)
    {
        bcc_error = BCC_CB_Enable(&(bty->drvConfig), (bcc_cid_t)(i+1),  true);
        if(bcc_error != BCC_STATUS_SUCCESS) {
            for(uint8_t j = 0; j < NUM_CELL_IC; j++){
                bty->cell_balancing[i*NUM_CELL_IC+j] = 100;
            }
            print_lpuart("init_cell_balancing issue: ");
            print_bcc_status(bcc_error);
            return 0;
        }
        for(uint8_t j = 0; j < NUM_CELL_IC; j++){
            bty->cell_balancing[i*NUM_CELL_IC+j] = 0;
        }
    }
    return 1;
}

/// @brief Check cell temps with bounds, updates errs
/// @param bty the hopefully not cooked battery
/// @return 1 if it passes, 0 otherwise
bool check_temp(Battery *bty){
    bool success = 1;
    bty->cell_temp_errors = 0;

    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        for(uint8_t j = 0; j < (NUM_CELL_IC); j++){
            // overtemp check
            if(bty->cell_temp[i*NUM_CELL_IC + j] > bty->max_temp_thresh){
                bty->cell_temp_errors++;
                bty->faults |= BATTERY_FAULT_CELL_OT; // probably not the correct one
                success = 0;
            }
            // undertemp check
            if(bty->cell_temp[i*NUM_CELL_IC + j] < bty->min_temp_thresh){
                bty->cell_temp_errors++;
                bty->faults |= BATTERY_FAULT_CELL_UT; // probably not the correct one
                success = 0;
            }
        }
    }
    return success;
}

/// @brief Check cell volts with bounds, updates errs
/// @param bty the hopefully not cooked battery
/// @return 1 if it passes, 0 otherwise
bool check_volt(Battery *bty) {
    uint8_t success = 1;
    bty->cell_volt_errors = 0;

    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        for(uint8_t j = 0; j < NUM_CELL_IC; j++){

            // max_volt check
            if(bty->cell_volt[i*NUM_CELL_IC + j] > bty->max_volt_thresh){
                bty->cell_volt_errors++;
                bty->faults |= BATTERY_FAULT_CELL_OV;
                success = 0;
            }

            // min_volt check
            if(bty->cell_volt[i*NUM_CELL_IC + j] < bty->min_volt_thresh){
                bty->cell_volt_errors++;
                bty->faults |= BATTERY_FAULT_CELL_UV;
                success = 0;
            }
        }
        
        // check stack voltage vs real sum aren't too different
        float manual_sum = 0;
        for (uint8_t j = 0; j < NUM_CELL_IC; j++) manual_sum += bty->cell_volt[(i*NUM_CELL_IC)+j];
        if(fabsf(bty->stack_voltage[i] - manual_sum) > 0.5f){
            bty->cell_volt_errors++;
            print_lpuart("stack voltage vs manual sum of cell volts is > 0.5!\n");
            success = 0;
        }
    }
    return success;
}

/// @brief reads cell volts & cell temps, updates neccessary struct data
/// @param bty 
/// @param fullcheck 
/// @return 1 if passes, 0 otherwise
bool battery_check(Battery *bty, bool fullcheck){

    bcc_error = BCC_STATUS_SUCCESS;
    bool success = 1;
    if(fullcheck){
        bcc_error = read_device_measurements(bty, true, true);
        if(bcc_error != BCC_STATUS_SUCCESS) print_bcc_status(bcc_error);
    }
    else if(cycle <= 7){ 
        bcc_error = read_device_measurements(bty, false, true); // read temps only
        if(bcc_error != BCC_STATUS_SUCCESS) print_bcc_status(bcc_error);
    }
    // check temp
    success = check_temp(bty);
    if(!success) print_lpuart("check_temp not successful\n");
    
    // read volts
    bcc_error = read_device_measurements(bty, true, false); // read volts only
    if(bcc_error != BCC_STATUS_SUCCESS) print_bcc_status(bcc_error);

    // check volts
    success = check_volt(bty);
    if(!success) print_lpuart("check_volt not successful\n");
    return success;
}

// dump temp measurements
void print_temperature(Battery * bty){    
    print_lpuart("Cell Temp: ------------------------------\n");

    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        float min_temp = __FLT_MAX__, max_temp = __FLT_MIN__;
        for (uint8_t j = 0; j < NUM_CELL_IC; j++){
            
            const float curr_temp = bty->cell_temp[(i*NUM_CELL_IC) + j];
            print_temp(bty->cell_temp, i, ((i*NUM_CELL_IC) + j));
            min_temp = fminf(min_temp, curr_temp);
            max_temp = fmaxf(max_temp, curr_temp);  
        }
        char buff[64];
        sprintf(buff, "Min temp: %.3f | Max temp: %.3f\n", min_temp, max_temp);
        print_lpuart(buff);
    }
    print_lpuart("-----------------------------------------\n");
}

// dump voltage measurements
void print_voltage(Battery *bty){
    
    print_lpuart("Cell Voltage: ------------------------------\n");
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        float min_volt = __FLT_MAX__, max_volt = __FLT_MIN__;
        for (uint8_t j = 0; j < NUM_CELL_IC; j++){
            
            const float curr_volt = bty->cell_volt[(i*NUM_CELL_IC) + j];
            print_volt(bty->cell_volt, i, ((i*NUM_CELL_IC) + j));
            min_volt = fminf(min_volt, curr_volt);
            max_volt = fmaxf(max_volt, curr_volt);  
        }
        print_lpuart("-- -- -- -- -- -- -- -- -- -- -- -- -- --\n");
        char buff[100];
        bzero(buff, sizeof(buff));
        sprintf(buff, "Min volt: %.3f | Max volt: %.3f\nStack Voltage: %.3f | IC Temp: %.3f\n", min_volt, max_volt, bty->stack_voltage[i], bty->icTemp[i]);
        print_lpuart(buff);
    }
    print_lpuart("-----------------------------------------\n");
}