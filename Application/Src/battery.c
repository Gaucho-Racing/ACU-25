#include "battery.h"

bcc_status_t read_device_measurements(Battery * bty) {
    
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

bcc_status_t battery_init(Battery *bty){
    // init BCC, TPL, Regs
    bcc_status_t errors = BCC_STATUS_SUCCESS;
    
    errors = init_registers(bty);
    if(errors != BCC_STATUS_SUCCESS) return errors;

    // configure cell balancing
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++)
    {
        bcc_status_t status = BCC_CB_Enable(&(bty->drvConfig), (bcc_cid_t)i+1,  true);
        if(status != BCC_STATUS_SUCCESS) return status;
    }

    read_device_measurements(bty);
    BCC_MCU_WaitUs(500);

    // print("cell_OV_Threshold: ");
    // print_float(CELL_MAX_VOLT);
    // print(", cell_UV_Threshold: ");
    // print_float(CELL_MIN_VOLT);
    // print("\n");

    if(errors != BCC_STATUS_SUCCESS) return errors;

    // diagnose cell voltages
    // check_volt();
    // if(errors != BCC_STATUS_SUCCESS) return errors;

    // diagnose cell temp
    // print("cell_OT_Threshold: ");
    // print_float(CELL_MAX_TEMP);
    // print(", cell_UT_Threshold: ");
    // print_float(CELL_MIN_TEMP);
    // print("\n");

    // check_temp();

    if(errors != BCC_STATUS_SUCCESS) return errors;

    // enable cell balancing
    // toggleCellBalancing(true, true, BCC_CID_UNASSIG, 0);
    return BCC_STATUS_SUCCESS;
}
