A program for the STM32L476RG microcontroller sending device rotation data to the computer. The device acts as a controller for a game "[Ścigałka](https://github.com/Triadziuch/Scigalka)".

# Description
This repository contains files for a program designed for the STM32L476RG microcontroller with the attached IKS01A3 board and an LCD Keypad Shield. The program transmits information to the computer via the UART interface about the device's position, which is read from sensors on the IKS01A3 board. Additionally, the device asynchronously receives data from the computer about the state of the "Racer" game, formats it, and displays it on the LCD Keypad Shield screen.

## Rotation data transmitted via UART:
![5 0 - 7](https://github.com/Triadziuch/STM32-Project-files/assets/75269577/2d5413a4-7a7d-4dac-bcdd-5f28fad6ad73)


## LCD Displaying data received via UART:
![2 2 - 5](https://github.com/Triadziuch/STM32-Project-files/assets/75269577/e8a522a3-cdae-4906-bf52-13842347878b)
