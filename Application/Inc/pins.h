#ifndef PINS_H
#define PINS_H
#pragma once

// Full duplex ==> comm with battery
#define TX 0
#define RX 0
#define SCK 0
#define SS 1

// Half duplex ==> slave comm with ECU
#define TX_2 0
#define RX_2 0
#define SCK_2 0
#define SS_2 1

#define PIN_EN 6
#define PIN_INT 9

#endif