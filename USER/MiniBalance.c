#include "stm32f10x.h"
#include "sys.h"

u16 led_freq=1;
s32 DesireL = 0, DesireR = 0;
s32 SpeedL = 50, SpeedR = 50;
float Velocity_Kp = 6.540, Velocity_Ki = 7.270, Velocity_Kd = 0;
int Voltage;                      //电池电压采样相关的变量
u8 Flag_Stop=1, report_flag=0, bluetooth_report=0, scope_report=0, save_flag=0, send_param_flag=0;  //停止标志位和 显示标志位 默认停止 显示打开
int app_1_encL_report = 0, app_2_encR_report = 0, app_3_vol_report = 0, app_4_angle_report = 0;
u8 speed_limit = 130;   // 限速0.3m/s，轮子一圈1040个脉冲，轮直径0.238m，控制10Hz，0.3/0.238*1040/10=131

u8 Way_Angle = 1;  //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
u8 Flag_Qian, Flag_Hou, Flag_Left, Flag_Right, Flag_sudu = 1;  //蓝牙遥控相关的变量
int Encoder_Left, Encoder_Right;  //左右编码器的脉冲计数
int Moto1, Moto2;                 //电机PWM变量 应是Motor的 向Moto致敬
int Temperature;                  //显示温度
float Angle_Balance=0, Gyro_Balance=0, Gyro_Turn=0;  //平衡倾角 平衡陀螺仪 转向陀螺仪
float Show_Data_Mb;  //全局显示变量，用于显示需要查看的数据
u32 Distance=0;        //超声波测距
float Acceleration_Z;  // Z轴加速度计
u16 PID_Parameter[10], Flash_Parameter[10];  // Flash相关数组

int main(void) {
  delay_init();                //=====延时函数初始化
  uart_init(256000);           //=====串口初始化为
  uart1_send("uart1 ok\r\n");
  JTAG_Set(JTAG_SWD_DISABLE);  //=====关闭JTAG接口
  JTAG_Set(SWD_ENABLE);  //=====打开SWD接口 可以利用主板的SWD接口调试
  LED_Init();            //=====初始化与 LED 连接的硬件接口
  KEY_Init();            //=====按键初始化
  MY_NVIC_PriorityGroupConfig(2);  //=====中断分组
  //=====初始化PWM 10KHZ，用于驱动电机 如需初始化电调接口
  MiniBalance_PWM_Init(7199, 0);
  uart2_init(230400);             //=====串口2初始化
  uart3_init(9600);               //=====串口3初始化
  Encoder_Init_TIM2();            //=====编码器接口
  Encoder_Init_TIM4();            //=====初始化编码器2
  Adc_Init();                     //=====adc初始化
  IIC_Init();                     //=====IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化
  DMP_Init();                     //=====初始化DMP
  MiniBalance_EXTI_Init();        //=====MPU6050 5ms定时中断初始化
  Flash_Read();

  while (1) {
    if (report_flag) {
      report_flag = 0;
      ReportEncoderBattery();

      if (bluetooth_report) {
        APP_Show();
      }
      if (scope_report) {
        DataScope();
      }
    }
    if (save_flag) {
        Flash_Write();
        save_flag = 0;
    }
    if (send_param_flag) {
      printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$", (int)(Velocity_Kp * 1000), (int)(Velocity_Ki * 1000), (int)(Velocity_Kd * 1000), SpeedL*100, SpeedR*100, Voltage, bluetooth_report, 0, 0);
      send_param_flag = 0;
    }
  }
}
