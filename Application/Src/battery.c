#include "battery.h"

extern uint8_t bcc_cooked_count;
extern bcc_status_t bcc_error;
extern void print_lpuart(char* arr);
extern void print_bcc_status(bcc_status_t bccStatus);
extern void print_bcc_fault(bcc_fault_status_t fault);


void clear_faults(bcc_drv_config_t * drvConfig)
{
    bcc_status_t status;
    for (uint8_t ic = 1; ic <= NUM_CELL_IC; ic++)
    {
        bcc_cid_t cid = (bcc_cid_t)ic;
        status = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_CELL_OV);
        status = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_CELL_UV);
        status = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_CB_OPEN);
        status = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_CB_SHORT);
        status = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_GPIO_STATUS);
        status = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_AN_OT_UT);
        status = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_GPIO_SHORT);
        status = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_COMM);
        status = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_FAULT1);
        status = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_FAULT2);
        status = BCC_Fault_ClearStatus(drvConfig, cid, BCC_FS_FAULT3);
        if(status != BCC_STATUS_SUCCESS) return;
    }
}

bcc_status_t init_registers(Battery * bty)
{
    uint8_t cid, i;
    bcc_status_t status;
    for (cid = 1; cid <= bty->drvConfig.devicesCnt; cid++)
    {
        for (i = 0; i < INIT_REG_CNT; i++)
        {
            if (init_regs[i].value != init_regs[i].defaultVal)
            {
                status = BCC_Reg_Write(&(bty->drvConfig), (bcc_cid_t)cid,
                        init_regs[i].address, init_regs[i].value);
                    if(status != BCC_STATUS_SUCCESS) return status;
            }
        }
    }
    return BCC_STATUS_SUCCESS;
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

void reset_discharge(Battery * bty){
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++)
    {
        for(uint8_t j = 0; j < NUM_CELL_IC; j++){
            if ((bcc_error = config_cell_balancing(bty, (bcc_cid_t)(i), j, 0, 1))!= BCC_STATUS_SUCCESS){ // resets after default = 30 seconds
                print_bcc_status(bcc_error);
                return;
            }
        }
    }
}

bcc_status_t read_device_measurements(Battery * bty, uint8_t read_volt, uint8_t read_temp) 
{
    
    uint32_t measurements[NUM_CELL_IC];
    int16_t temp_measures[NUM_CELL_IC];
    bcc_status_t error;

    bzero(measurements, sizeof(measurements));
    bty->max_cell_volt = 0;
    bty->max_cell_temp = 0;
    bty->min_cell_temp = 10000.0;
    bty->min_cell_volt = 10000.0;
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        BCC_CB_Pause(&(bty->drvConfig), (bcc_cid_t)(i+1), true); // pause b4 read

        if(true){

            // CELL VOLTAGES
            BCC_Meas_StartAndWait(&(bty->drvConfig), (bcc_cid_t)(i+1), BCC_AVG_1);
            error = BCC_Meas_GetCellVoltages(&(bty->drvConfig), (bcc_cid_t)(i+1), measurements);
            if(error != BCC_STATUS_SUCCESS) {
                bcc_cooked_count++;
                if(bcc_cooked_count == 0){
                    print_lpuart("\nerror in BCC_Meas_GetCellVoltages: ");
                    print_bcc_status(error);
                    return error;
                }
            }
            for (uint8_t j = 0; j < NUM_CELL_IC; j++){
                bty->cell_volt[(i*NUM_CELL_IC)+j] = (measurements[j] * 1e-6f);
                bty->max_cell_volt = fmax(bty->max_cell_volt, bty->cell_volt[(i*NUM_CELL_IC)+j]);
                bty->min_cell_volt = fmin(bty->min_cell_volt, bty->cell_volt[(i*NUM_CELL_IC)+j]);
            }
            // print_voltage(bty);

            bzero(measurements, sizeof(measurements));
            error = BCC_Meas_GetStackVoltage(&(bty->drvConfig), (bcc_cid_t)(i+1), measurements);
            bty->stackVoltage[i] = *measurements * 1e-6f; // theres ~ .3 difference from manual calcs

            // RIP debugging
            // float manual_totaled = 0;
            // char buff_read[32], buff[32], buff_man[32];
            // for (uint8_t j = 0; j < NUM_CELL_IC; j++) manual_totaled += bty->cell_volt[(i*NUM_CELL_IC)+j];
            // sprintf(buff_man, "Manual Stack Volt: %.3f\n", manual_totaled);
            // sprintf(buff_read, "Read Stack Volt: %.3f\n", bty->stackVoltage[i]);
            // sprintf(buff, "read - manual: %.3f\n", (bty->stackVoltage[i] - manual_totaled));
            // print_lpuart(buff_man); print_lpuart(buff_read); print_lpuart(buff);

            if(error != BCC_STATUS_SUCCESS) {
                bcc_cooked_count++;
                if(bcc_cooked_count == 0){
                    print_lpuart("\nerror in BCC_Meas_GetStackVoltage: ");
                    print_bcc_status(error);
                    return error;
                }
            }
        }

        // CELL TEMPS
        if(true){
            bzero(temp_measures, sizeof(temp_measures));
            error = BCC_Meas_GetIcTemperature(&(bty->drvConfig), (bcc_cid_t)(i+1), BCC_TEMP_CELSIUS, temp_measures);
            bty->icTemp[i] = temp_measures[i] * 0.1f;

            // // RIP debugging
            // #if DEBUG == 1
            //     char buffer[32];
            //     sprintf(buffer, "Read IC Temp: %.3f\n", bty->icTemp[i]);
            //     print_lpuart(buffer);
            // #endif

            if(error != BCC_STATUS_SUCCESS) {
                bcc_cooked_count++;
                if(bcc_cooked_count == 0){
                    print_lpuart("error in BCC_Meas_GetIcTemperature: ");
                    print_bcc_status(error);
                    return error;
                }
            }
        }

        if(true){
            for(uint8_t j = 0; j < 32; j++) { // TODO: fix this loop
                uint8_t readByte;
                error = BCC_EEPROM_Read(&(bty->drvConfig), (bcc_cid_t)(i+1), j+1, &readByte);
                if (error == BCC_STATUS_SUCCESS) {
                    bty->cell_temp[i*NUM_CELL_IC + j] = (float)(readByte * 0.1f);
                    bty->max_cell_temp = fmax(bty->max_cell_temp, bty->cell_temp[(i*NUM_CELL_IC)+j]);
                    bty->min_cell_temp = fmin(bty->min_cell_temp, bty->cell_temp[(i*NUM_CELL_IC)+j]);
                }
                if(error != BCC_STATUS_SUCCESS) {
                    bcc_cooked_count++;
                    if(bcc_cooked_count == 0){
                        print_lpuart("error in BCC_EEPROM_Read: ");
                        print_bcc_status(error);
                        return error;
                    }
                }
            }
        }

        BCC_CB_Pause(&(bty->drvConfig), (bcc_cid_t)(i+1), false); // resume after read
    }
    return BCC_STATUS_SUCCESS;
}

void update_cell_voltages(Battery * bty){
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        BCC_CB_Pause(&(bty->drvConfig), (bcc_cid_t)(i+1), true); // pause b4 read

        if(true){
            uint32_t measurements[NUM_CELL_IC];
            BCC_Meas_StartAndWait(&(bty->drvConfig), (bcc_cid_t)(i+1), BCC_AVG_1);
            bcc_status_t err = BCC_Meas_GetCellVoltages(&(bty->drvConfig), (bcc_cid_t)(i+1), measurements);
            if(err != BCC_STATUS_SUCCESS) {
                bcc_cooked_count++;
                if(bcc_cooked_count == 0){
                    print_lpuart("\nerror in update_cell_voltages: ");
                    print_bcc_status(err);
                    return;
                }
            }
            for (uint8_t j = 0; j < NUM_CELL_IC; j++){
                bty->cell_volt[(i*NUM_CELL_IC)+j] = (measurements[j] * 1e-6f);
            }
        }
    }
}

// assumes we read device measurements already
bool do_cell_balancing(Battery * bty, bool all){

    float threshold = (bty->min_cell_volt + bty->max_cell_volt)/2;
    bcc_status_t err = BCC_STATUS_SUCCESS;

    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++)
    {
        for(uint8_t j = 0; j < NUM_CELL_IC; j++){
            if((bty->cell_volt[i*NUM_CELL_IC+j] > threshold || bty->cell_volt[i*NUM_CELL_IC + j] - bty->min_cell_volt > 0.02) /*&& bty->bal_temp[i*NUM_CELL_IC + j] < 70*/){
                
                if ((err = config_cell_balancing(bty, (bcc_cid_t)(i), j, 0, 1))!= BCC_STATUS_SUCCESS){ // resets after default = 30 seconds
                    print_bcc_status(err);
                    return false;
                }
            }
        }
    }
    return true;
}

bcc_status_t config_cell_balancing(Battery * bty, bcc_cid_t cid, uint8_t cellIndex, bool all, bool enable)
{
    bcc_status_t errors = BCC_STATUS_SUCCESS;
    // set all
    if(all){
        for(uint8_t i = 0; i < NUM_TOTAL_IC; i++)
        {
            for(uint8_t j = 0; j < NUM_CELL_IC; j++){

                bcc_status_t errors;
                bty->cell_balancing[i*NUM_CELL_IC+j] = 0;
                if((errors = BCC_CB_SetIndividual(&(bty->drvConfig), (bcc_cid_t)(i+1), j, true, 0)) != BCC_STATUS_SUCCESS){
                    print_lpuart("failed BCC_CB_SetIndividual\n");
                    print_bcc_status(errors);
                    return -1;
                }
                bty->cell_balancing[i*NUM_CELL_IC+j] = enable == true ? 255 : 0;
            }
        }
        return errors;
    }
    else{

        // sanity check
        if(cellIndex >= NUM_CELL_IC){
            print_lpuart("Invalid cell index");
            return errors;
        }

        // set individuals
        if((errors = BCC_CB_SetIndividual(&(bty->drvConfig), cid, cellIndex, enable, 0)) != BCC_STATUS_SUCCESS) {
            print_lpuart("Failed BCC_CB_SetIndividual\n");
            return errors;
        }
        bty->cell_balancing[(uint8_t)cid*cellIndex+cellIndex] = enable == true ? 255 : 0;
    }
    return errors;
}

bcc_status_t check_temp(Battery *bty){
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        for(uint8_t j = 0; j < (NUM_CELL_IC); i++){
            if(bty->cell_temp[i*NUM_CELL_IC + j] > CELL_MAX_TEMP){
                bty->faults = BCC_FS_AN_OT_UT;
                // print_lpuart("overtemp: ");
                // print_bcc_fault(BCC_FS_AN_OT_UT);
                return BCC_STATUS_DIAG_FAIL;
            }
            if(bty->cell_temp[i*NUM_CELL_IC + j] < CELL_MIN_TEMP){
                bty->faults = BCC_FS_AN_OT_UT;
                // print_lpuart("undertemp: ");
                // print_bcc_fault(BCC_FS_AN_OT_UT);
                return BCC_STATUS_DIAG_FAIL;
            }
        }
    }
    return BCC_STATUS_SUCCESS;
}

bcc_status_t check_volt(Battery *bty) {
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        for(uint8_t j = 0; j < NUM_CELL_IC; j++){
            if(bty->cell_volt[i*NUM_CELL_IC + j] > CELL_MAX_VOLT){
                bty->faults = BCC_FS_CELL_OV;
                print_lpuart("over volt: ");
                print_bcc_fault(BCC_FS_CELL_OV);
                return BCC_STATUS_DIAG_FAIL;
            }
            else if(bty->cell_volt[i*NUM_CELL_IC + j] < CELL_MIN_VOLT){
                bty->faults = BCC_FS_CELL_UV;
                print_lpuart("under volt: ");
                print_bcc_fault(BCC_FS_CELL_UV);
                return BCC_STATUS_DIAG_FAIL;
            }
        }
        // check stack voltage vs real sum aren't too different
        float manual_sum = 0;
        for (uint8_t j = 0; j < NUM_CELL_IC; j++) manual_sum += bty->cell_volt[(i*NUM_CELL_IC)+j];
        if(fabs(bty->stackVoltage[i] - manual_sum) > 0.5){
            print_lpuart("stack voltage vs manual sum of cell volts is > 0.5!\n");
            return BCC_STATUS_DIAG_FAIL;
        }
    }
    return BCC_STATUS_SUCCESS;
}

bcc_status_t check_fuse(Battery *bty){
    return BCC_STATUS_SUCCESS;
}

void battery_check(Battery *bty, bool fullcheck){
    bcc_status_t errors = BCC_STATUS_SUCCESS;
    if(fullcheck){

        // read temps
        errors = read_device_measurements(bty, false, true);
        if(errors != BCC_STATUS_SUCCESS) print_bcc_status(errors);

        // read and check fuse
        // this->checkAllFuse();
        // check_fuse(bty);
    }
    else {
        // read temps
        // updateTemp();

        // read and check fuse
        // checkFuse();
    }
    errors = check_temp(bty);
    if(errors != BCC_STATUS_SUCCESS) print_bcc_status(errors);

    // read and check voltages
    errors = read_device_measurements(bty, true, false);
    if(errors != BCC_STATUS_SUCCESS) print_bcc_status(errors);

    errors = check_volt(bty);
    if(errors != BCC_STATUS_SUCCESS) print_bcc_status(errors);
}

bool check_faults(Battery *bty) {
    
    uint16_t fault;
    bcc_status_t status = BCC_STATUS_SUCCESS; // status of all devices
    bool faults = false;

    for(uint8_t i = 0; i <= NUM_TOTAL_IC; i++){    
        if((status = BCC_Fault_GetStatus(&(bty->drvConfig), (bcc_cid_t)i, &fault)) != BCC_STATUS_SUCCESS){
            faults = true;
        }
    }
    return faults;
}

// dump cell balancing measurements
void print_cell_balancing(Battery * bty){
    print_lpuart("Cell Balancing: --------------------------\n");
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        for (uint8_t j = 0; j < NUM_CELL_IC; j++){
            char buff[64];
            sprintf(buff, "Row %u, Cell %u: %d, Volt: %.3f\n", i, ((i*NUM_CELL_IC) + j), bty->cell_balancing[(i*NUM_CELL_IC) + j], bty->cell_volt[(i*NUM_CELL_IC) + j]);
            print_lpuart(buff);
        }        
    }
    print_lpuart("-----------------------------------------\n");
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
        sprintf(buff, "Min volt: %.3f | Max volt: %.3f\nStack Voltage: %.3f | IC Temp: %.3f\n", min_volt, max_volt, bty->stackVoltage[i], bty->icTemp[i]);
        print_lpuart(buff);
    }
    print_lpuart("-----------------------------------------\n");
}