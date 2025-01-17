#include "acu.h"

uint8_t system_check(uint8_t startup) {
    return;
}
void read_device_measurements(){
    return;
}
void check_temperature(){
    return;
}
void check_voltage(){
    return;
}
void check_fuse(){
    return;
}
void check_status(){
    return;   
}
void cell_balancing(uint8_t all, uint8_t enable, uint8_t cluster_id, uint8_t cell_idx){
    return;
}
void print_measurements(){
    return;
}
uint8_t init(battery * bat){
    bat->check_fuse = &check_fuse;
    bat->check_status = &check_status;
    bat->cell_balancing = &cell_balancing;
    bat->check_voltage = &check_voltage;
    bat->check_temperature = &check_temperature;
    bat->read_device_measurements = &read_device_measurements;

    return 0;
}

int main(){
    battery bcc;
    init(&bcc);

    return 0;
}
// Master
// HAL_SPI_Transmit(&hspi1, TxBuff, 1, 1000); // Blocking mode
// HAL_Delay(100);

// HAL_SPI_Transmit_IT(&hspi1, TxBuff, 1); // Interrupt mode
// HAL_Delay(100);

// Slave
// HAL_SPI_Receive(&hspi1, RxBuff, 1, 1000); //Blocking mode
// HAL_Delay(100);

// HAL_SPI_Receive_IT(&hspi1, RxBuff, 1); // Interrupt mode
// HAL_Delay(100);