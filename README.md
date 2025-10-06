# STM32F103C8T6 I2C Slave Communication Demo

## Overview
This project demonstrates I2C slave communication using the WeAct STM32F103C8T6 Blue Pill Plus board. The device operates as an I2C slave and receives 4-byte messages from an I2C master, logging the data via UART.

## Features
- **I2C Slave Mode**: Listens on I2C address 0x37 (55 << 1)
- **UART Logging**: Outputs received I2C data at 115200 baud
- **LED Feedback**: Onboard LED (PB2) indicates system status
- **Button Input**: KEY button (PA0) triggers fast LED toggle mode
- **Interrupt-Driven**: Uses interrupts for both I2C reception and button input

## Hardware Configuration

### Peripherals Used
- **I2C2**: Slave mode, 100kHz, 7-bit addressing (address 0x37)
- **USART1**: 115200 baud, 8N1 for debug output
- **GPIO PA0**: KEY button input with pull-down, EXTI interrupt enabled
- **GPIO PB2**: Onboard LED output
- **CAN1**: Initialized but not currently used in application

### Pin Assignments
- **PA0**: KEY button (input, pull-down, rising/falling edge interrupt)
- **PB2**: Onboard LED (output)
- **PB10**: I2C2_SCL
- **PB11**: I2C2_SDA
- **PA9**: USART1_TX
- **PA10**: USART1_RX

## Operation

### LED Behavior
1. **Normal Mode**: LED toggles slowly (every 1 second)
2. **Fast Mode**: LED toggles rapidly (every 80ms) for 2 seconds after:
   - KEY button is pressed
   - I2C data is received

### I2C Communication
- Device acts as I2C slave at address 0x07
- Expects 3-byte messages from I2C master
- Received data is logged via UART in format: `[second:millisecond] received [byte0 byte1 byte2]`
- When no data is received, periodic status messages are sent: `No data for Xls`

### Button Input
- Press KEY button (PA0) to trigger fast LED toggle mode
- Fast mode lasts for 2 seconds, then returns to normal slow toggle

## UART Output Format
```
...
[   168:123] received [ 16  15 240]
[   168:123] received [ 32  15 240]
[   168:123] received [ 64  15 240]
[   168:123] received [128  15 240]
...
```

## Building and Flashing
This is an STM32CubeIDE project. To build and flash:
1. Open the project in STM32CubeIDE
2. Build the project (Ctrl+B)
3. Flash to the target device using ST-Link

## Testing I2C Communication
Use an I2C master device (Arduino, Raspberry Pi, etc.) to send 4-byte messages to address 0x37:

### Example with Arduino:
```cpp
// Include the required Wire library for I2C
#include <Wire.h>
uint8_t x = 1;

void setup() {
  Wire.begin();
  Wire.setClock(100000);
}

void loop() 
{
  Wire.beginTransmission(7); // transmit to device #55
  Wire.write(x);                 // sends x 
  Wire.write(0x0F);              // sends x
  Wire.write(0xF0);              // sends x
  Wire.endTransmission();     // stop transmitting

  delay(200);
  
  x <<= 1;
  if (x == 0) {
    x = 1;
  }  
}
```

### Example with i2c-tools (Linux):
```bash
i2cset -y 1 0x37 0x10 0x20 0x30 0x40 i
```

## System Clock
- External crystal: 8 MHz HSE
- PLL multiplier: x9
- System clock: 72 MHz

## Known Improvements Needed
- CAN peripheral is initialized but not used - consider removing if not needed
- Add I2C error recovery mechanism
- Implement watchdog timer for enhanced reliability
- Add button debouncing for cleaner interrupt handling

## License
This software is provided AS-IS.
