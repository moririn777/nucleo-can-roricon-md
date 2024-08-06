# README  
## PINの機能  
|PIN|機能|
|:---:|:---:|
|PA0|encoder1_A|  
|PA1|encoder1_B|  
|PA2|USART2_Tx|  
|PA3|motor1_dir|  
|PA4|motor1_pwm|
|PA5|motor1_dir|
|PA6|motor1_pwm|
|PA8|encoder2_A|
|PA9|encoder2_B|
|PA11|CAN_RX|
|PA12|CAN_TX|
|PA13|SYS_JTCK-SWCLK|
|PA14|SYS_JTMS-SWDIO|
|PA15|USART2_RX|
|PB0|encoder1_X|
|PB1|encoder2_X|  
|PB3|GreenLED|  

## iocファイルの設定
### Pinout&Configurationの設定

### Pinout view
![PinImage](/picture/pin.png)
### GPIO Mode and Configration  
![GPIO_Configration](/picture/GPIO.png)  
LEDは命名しなくてもよい  
### NVIC interrupt Table
![NVIC](/picture/NVIC_Mode.png)  
EXTI line 0 interrupt  
EXTI line 1 interrupt  
CAN TX interrupt  
CAN RX0 interrupt  
### RCC Mode and Configuration
![RCC](/picture/RCC.png)  
High Speed Clock : BYPASS Clock Source  
### SYS Mode and Configuration
![SYS](/picture/SYS.png)  
Debug : Serial Write  
Timebase Source SysTick  
### TSC Mode and Configration
![TSC](/picture/TSC.png)
### Analog ADC* Mode and Configuration
すべてDisable  
### Timers
#### **TIM1 TIM2** 
![TIM1](/picture/TIM.png)  
Combined Channels : Encoder Mode  
### Parmeter Settings 
![Configuration](/picture/TIM_config.png)  
他のパラメーターは変更しない  
#### **TIM3**
![TIM3](/picture/TIM3.png)  
![TIM3_config](/picture/TIM3_config.png)  
### Counter Setting
Prescaler : 0  
Coutor Mode : 255  
#### **TIM15**
![TIM15](/picture/TIM15.png)  
すべてDisable
### CAN  
#### CAN Mode and Configuration
![CAN](/picture/can.png)  
### Mode 
Activated  
### Configration  
#### Bit Timings Parameters
Prescaler 4  
Time Quanta in Bit Segment 1 : 11 Times  
Time Quanta in Bit Segment 2 :  4 Times  
ReSynchronization Jump Width :  1 Times 
### USART1  
![USART1](/picture/USART1.png)  
Mode : Disable  
### USART2  
![USART2](/picture/USART2.png)  
#### Mode
Mode : Asynchronous
#### Configuration  
Prescaler : 115200 Bit/s  
Word Length : 8 Bits
