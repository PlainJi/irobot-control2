## For STM32  
USART1  PA9_TX    PA10_RX    Used for Debug  
USART2  PA2_TX    PA3_RX     Report Encoder&Voltage to JetsonNano(Orange TX, Yellow RX)  
  Send  
    Encoder: ~>E+12345,+12345\n  
    Battery: ~>B+12345\n  
    Imu:     ~>I%f%f%f%f%f%f%f%f%f%f  
  Recv  
    Expected pulses: $+12345,+12345\n  
USART3  PB10_TX   PB11_RX    Connected to Bluetooth. Used for Debug.   

## For JetsonNano  
UART2   PIN8_TX   PIN10_RX   PIN6_GND   /dev/ttyTHS1  
  Recv  
    Encoder: ~>E+12345,+12345\n  
    Battery: ~>B+12345\n  
    Imu:     ~>I%f%f%f%f%f%f%f%f%f%f  
  Send  
    Expected pulses: $+12345,+12345\n  

## For IMU MPU6050  
Gyro Range +-500 degree/s  
`MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500);`  
Acc  Range +-2   g/s  
`MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);`  
Frequency  
21 Hz  
`#define DEFAULT_MPU_HZ (21)`  