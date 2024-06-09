# Using-STM32-Flash-Memory-as-EEPROM

This project demonstrates how to count button presses on an STM32F407 Discovery board and store the count in Flash memory to retain it across power cycles.

## Hardware

- **Board:** STM32F407 Discovery
- **Button:** User button connected to GPIOA Pin 0

## Project Description

The goal of this project is to create a counter that increments each time the user button on the STM32F407 Discovery board is pressed. The count is stored in the Flash memory, ensuring that the counter retains its value even after the board is powered off and on again. This is useful for applications where you need to keep a persistent count or state across power cycles.

## How it Works

1. **Initialization:**
   - The system initializes the hardware, including the button and the LED.
   - The last stored count is read from the Flash memory during initialization.

2. **Button Press Detection:**
   - The system continuously polls the state of the user button.
   - When a button press is detected (with debounce handling), the count is incremented.

3. **Flash Memory Storage:**
   - The updated count is stored in the Flash memory.
   - The Flash memory sector is erased before writing the new count to ensure data integrity.

## Software Components

- **STM32 HAL Libraries:** Provides hardware abstraction layer for STM32 peripherals.
- **Flash Memory Handling:** Functions to read from and write to Flash memory.
- **Button Handling:** Debounce logic to handle button presses accurately.
- **LED Control:** Simple LED toggling to provide visual feedback.

## Getting Started

### Prerequisites

- **IDE:** STM32CubeIDE or any other compatible IDE for STM32 development.
- **Toolchain:** ARM GCC toolchain for compiling the code.
- **STM32CubeMX:** (Optional) for generating initialization code.

### Cloning the Repository

Clone the repository to your local machine using the following command:

```sh
git clone https://github.com/enveecto/Using-STM32-Flash-Memory-as-EEPROM

