#include "battery.h"

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

bcc_status_t read_device_measurements(Battery * bty) 
{
    
    uint32_t measurements[NUM_CELL_IC];
    int16_t temp_measures[NUM_CELL_IC];
    bcc_status_t error;

    bzero(measurements, NUM_TOTAL_IC);

    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        BCC_CB_Pause(&(bty->drvConfig), (bcc_cid_t)(i+1), true); // pause b4 read

        if(true){ // get cell & stack voltage
            BCC_Meas_StartAndWait(&(bty->drvConfig), (bcc_cid_t)(i+1), BCC_AVG_1);
            error = BCC_Meas_GetCellVoltages(&(bty->drvConfig), (bcc_cid_t)(i+1), measurements);
            if(error != BCC_STATUS_SUCCESS) return error;
            for(uint8_t j = 0; j < NUM_CELL_IC; j++){
                bty->cell_volt[i*NUM_CELL_IC + j] = measurements[j] * 1e-6f;
            }
            bzero(measurements, NUM_TOTAL_IC);
            error = BCC_Meas_GetStackVoltage(&(bty->drvConfig), (bcc_cid_t)(i+1), measurements);
            if(error != BCC_STATUS_SUCCESS) return error;
        }

        if(true){ // get IC temperature
            bzero(temp_measures, NUM_CELL_IC);
            error = BCC_Meas_GetIcTemperature(&(bty->drvConfig), (bcc_cid_t)(i+1), BCC_TEMP_CELSIUS, temp_measures);
            bty->icTemp[i] = temp_measures[i] * 0.1f; // when printing make sure to multiply by 0.1
        }

        // if(true){
        //     for(uint8_t j = 0; j < 32; j++) { // TODO: fix this loop
        //         uint8_t readByte;
        //         error = BCC_EEPROM_Read(&(bty->drvConfig), (bcc_cid_t)(i+1), j+1, &readByte);
        //         if (error == BCC_STATUS_SUCCESS) bty->cell_temp[i*NUM_CELL_IC + j] = (float)readByte;
        //     }
        // }

        BCC_CB_Pause(&(bty->drvConfig), (bcc_cid_t)(i+1), false); // resume after read
    }
    return BCC_STATUS_SUCCESS;
}

bcc_status_t config_cell_balancing(Battery * bty, bcc_cid_t cid, uint8_t cellIndex, bool all, bool enable)
{
    bcc_status_t errors = BCC_STATUS_SUCCESS;

    // set groups
    if(all){
        for(uint8_t i = 0; i < NUM_TOTAL_IC; i++) // should be 1 right now
        {
            for(uint8_t j = 0; j < NUM_CELL_IC; j++) // should be 14 right now
            {
                
                if((errors = BCC_CB_SetIndividual(&(bty->drvConfig), (bcc_cid_t)i+1,  j, enable, 0)) != BCC_STATUS_SUCCESS) {
                    // BCC_Fault_GetStatus(&(bty->drvConfig), i, buff);
                    // BCC_Fault_GetStatus(&(bty->drvConfig), i, buff);
                    return errors;
                }
                bty->cell_balancing[i*j+j] = enable == true ? 255 : 0;
            }
        }
        return errors;
    }
    
    if(cellIndex >= NUM_CELL_IC){
        // print("Invalid cell index");
        return errors;
    }

    // set individuals
    if((errors = BCC_CB_SetIndividual(&(bty->drvConfig), cid, cellIndex, enable, 0)) != BCC_STATUS_SUCCESS) {
        // BCC_Fault_GetStatus(&(bty->drvConfig), cid, buff);
        // BCC_Fault_GetStatus(&(bty->drvConfig), cid, buff);
        return errors;
    }
    bty->cell_balancing[(uint8_t)cid*cellIndex+cellIndex] = enable == true ? 255 : 0;
    return errors;
}

bcc_status_t check_temp(Battery *bty){
    for(int i = 0; i < NUM_TOTAL_IC; i++){
        for(int j = 0; j < (NUM_CELL_IC); i++){
            if(bty->cell_temp[i*NUM_CELL_IC + j] > CELL_MAX_TEMP){
                bty->faults = BCC_FS_AN_OT_UT;
                return BCC_STATUS_DIAG_FAIL;
            }
            if(bty->cell_temp[i*NUM_CELL_IC + j] < CELL_MIN_TEMP){
                bty->faults = BCC_FS_AN_OT_UT;
                return BCC_STATUS_DIAG_FAIL;
            }
        }
    }
    return BCC_STATUS_SUCCESS;
}

bcc_status_t check_volt(Battery *bty) {
    for(int i = 0; i < NUM_TOTAL_IC; i++){
        for(int j = 0; j < NUM_CELL_IC; j++){
            if(bty->cell_volt[i*NUM_CELL_IC + j] > CELL_MAX_VOLT){
                bty->faults = BCC_FS_CELL_OV;
                return BCC_STATUS_DIAG_FAIL;
            }
            else if(bty->cell_volt[i*NUM_CELL_IC + j] < CELL_MIN_VOLT){
                bty->faults = BCC_FS_CELL_UV;
                return BCC_STATUS_DIAG_FAIL;
            }
        }
    }
    return BCC_STATUS_SUCCESS;
}

bcc_status_t check_fuse(Battery *bty){
    return BCC_STATUS_SUCCESS;
}

bool system_check(Battery *bty, bool startup){
    bcc_status_t errors = read_device_measurements(bty);
    if(errors != BCC_STATUS_SUCCESS){
        return false;
    }
    if((errors = check_temp(bty))!= BCC_STATUS_SUCCESS){
        return false;
    }
    if((errors = check_volt(bty))!= BCC_STATUS_SUCCESS){
        return false;
    }
    if((errors = check_fuse(bty))!= BCC_STATUS_SUCCESS){
        return false;
    }
    return true;
}

bool check_faults(Battery *bty) {
    
    uint16_t fault;
    bcc_status_t status = BCC_STATUS_SUCCESS; // status of all devices
    bool faults = false;

    for(int i = 0; i <= NUM_TOTAL_IC; i++){    
        if((status = BCC_Fault_GetStatus(&(bty->drvConfig), (bcc_cid_t)i, &fault)) != BCC_STATUS_SUCCESS){
            faults = true;
        }
    }
    return faults;
}

// dump temp measurements
void print_temperature(Battery * bty){
    float_t min_temp = __FLT_MAX__, max_temp = __FLT_MIN__;
    print("Cell Temp: ------------------------------\n");
    for(int i = 0; i < (NUM_TOTAL_IC); i++){

        print("Row ");
        print_decimal(i);
        print(" => ");

        for (int j = 0; j < NUM_CELL_IC; j++){
            
            print("[Cell ");
            print_decimal(j);
            min_temp = fmin(min_temp, bty->cell_temp[i]);
            max_temp = fmax(max_temp, bty->cell_temp[i]);

            print_float(bty->cell_temp[i] * 0.1);
            if(bty->cell_temp[i] < CELL_MIN_TEMP){
                print("| Under] ");

            } else if (bty->cell_temp[i] > CELL_MAX_TEMP){
                print("| Over] ");
            }
            else{
                print("] ");
            }
        }
        print("\n");
        print("Min temp: ");
        print_float(min_temp);
        print(" | Max temp: ");
        print_float(max_temp);
        print("\n");
    }
    print("-----------------------------------------\n");
}

// dump voltage measurements
void print_voltage(Battery *bty){
    float_t min_volt = __FLT_MAX__, max_volt = __FLT_MIN__;
    print("Cell Voltage: --------------------------\n");
    for(int i = 0; i < (NUM_TOTAL_IC); i++){
        
        print("Row ");
        print_decimal(i);
        print(" => ");

        for (int j = 0; j < NUM_CELL_IC; j++){
            
            print("[Cell ");
            print_decimal(j);

            min_volt = fmin(min_volt, bty->cell_volt[i]);
            max_volt = fmax(max_volt, bty->cell_volt[i]);

            print_float(bty->cell_volt[i]);
            if(bty->cell_volt[i] < CELL_MIN_VOLT){
                print("| Under] ");

            } else if (bty->cell_volt[i] > CELL_MAX_VOLT){
                print("| Over] ");
            }
            else{
                print("] ");
            }
        }
        print("\n");
        print("Min volt: ");
        print_float(min_volt);
        print(" | Max volt: ");
        print_float(max_volt);
        print("\n");
    }
    print("-----------------------------------------\n");
}