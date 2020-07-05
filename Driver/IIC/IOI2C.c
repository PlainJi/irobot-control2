#include "ioi2c.h"

#include "delay.h"
#include "sys.h"


void IIC_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //使能PB端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;  //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       // 50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);  //根据设定参数初始化GPIOB
}

int IIC_Start(void) {
  SDA_OUT();  // sda线输出
  IIC_SDA = 1;
  if (!READ_SDA) return 0;
  IIC_SCL = 1;
  delay_us(1);
  IIC_SDA = 0;  // START:when CLK is high,DATA change form high to low
  if (READ_SDA) return 0;
  delay_us(1);
  IIC_SCL = 0;  //钳住I2C总线，准备发送或接收数据
  return 1;
}

void IIC_Stop(void) {
  SDA_OUT();  // sda线输出
  IIC_SCL = 0;
  IIC_SDA = 0;  // STOP:when CLK is high DATA change form low to high
  delay_us(1);
  IIC_SCL = 1;
  IIC_SDA = 1;  //发送I2C总线结束信号
  delay_us(1);
}

int IIC_Wait_Ack(void) {
  u8 ucErrTime = 0;
  SDA_IN();  // SDA设置为输入
  IIC_SDA = 1;
  delay_us(1);
  IIC_SCL = 1;
  delay_us(1);
  while (READ_SDA) {
    ucErrTime++;
    if (ucErrTime > 50) {
      IIC_Stop();
      return 0;
    }
    delay_us(1);
  }
  IIC_SCL = 0;  //时钟输出0
  return 1;
}

void IIC_Ack(void) {
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 0;
  delay_us(1);
  IIC_SCL = 1;
  delay_us(1);
  IIC_SCL = 0;
}

void IIC_NAck(void) {
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 1;
  delay_us(1);
  IIC_SCL = 1;
  delay_us(1);
  IIC_SCL = 0;
}

void IIC_Send_Byte(u8 txd) {
  u8 t;
  SDA_OUT();
  IIC_SCL = 0;  //拉低时钟开始数据传输
  for (t = 0; t < 8; t++) {
    IIC_SDA = (txd & 0x80) >> 7;
    txd <<= 1;
    delay_us(1);
    IIC_SCL = 1;
    delay_us(1);
    IIC_SCL = 0;
    delay_us(1);
  }
}

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) {
  int i;
  if (!IIC_Start()) return 1;
  IIC_Send_Byte(addr << 1);
  if (!IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg);
  IIC_Wait_Ack();
  for (i = 0; i < len; i++) {
    IIC_Send_Byte(data[i]);
    if (!IIC_Wait_Ack()) {
      IIC_Stop();
      return 0;
    }
  }
  IIC_Stop();
  return 0;
}

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf) {
  if (!IIC_Start()) return 1;
  IIC_Send_Byte(addr << 1);
  if (!IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg);
  IIC_Wait_Ack();
  IIC_Start();
  IIC_Send_Byte((addr << 1) + 1);
  IIC_Wait_Ack();
  while (len) {
    if (len == 1)
      *buf = IIC_Read_Byte(0);
    else
      *buf = IIC_Read_Byte(1);
    buf++;
    len--;
  }
  IIC_Stop();
  return 0;
}

u8 IIC_Read_Byte(unsigned char ack) {
  unsigned char i, receive = 0;
  SDA_IN();  // SDA设置为输入
  for (i = 0; i < 8; i++) {
    IIC_SCL = 0;
    delay_us(2);
    IIC_SCL = 1;
    receive <<= 1;
    if (READ_SDA) receive++;
    delay_us(2);
  }
  if (ack)
    IIC_Ack();  //发送ACK
  else
    IIC_NAck();  //发送nACK
  return receive;
}

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr) {
  unsigned char res = 0;

  IIC_Start();
  IIC_Send_Byte(I2C_Addr);  //发送写命令
  res++;
  IIC_Wait_Ack();
  IIC_Send_Byte(addr);
  res++;  //发送地址
  IIC_Wait_Ack();
  // IIC_Stop();//产生一个停止条件
  IIC_Start();
  IIC_Send_Byte(I2C_Addr + 1);
  res++;  //进入接收模式
  IIC_Wait_Ack();
  res = IIC_Read_Byte(0);
  IIC_Stop();  //产生一个停止条件

  return res;
}

u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data) {
  u8 count = 0;

  IIC_Start();
  IIC_Send_Byte(dev);  //发送写命令
  IIC_Wait_Ack();
  IIC_Send_Byte(reg);  //发送地址
  IIC_Wait_Ack();
  IIC_Start();
  IIC_Send_Byte(dev + 1);  //进入接收模式
  IIC_Wait_Ack();

  for (count = 0; count < length; count++) {
    if (count != length - 1)
      data[count] = IIC_Read_Byte(1);  //带ACK的读数据
    else
      data[count] = IIC_Read_Byte(0);  //最后一个字节NACK
  }
  IIC_Stop();  //产生一个停止条件
  return count;
}

u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8 *data) {
  u8 count = 0;
  IIC_Start();
  IIC_Send_Byte(dev);  //发送写命令
  IIC_Wait_Ack();
  IIC_Send_Byte(reg);  //发送地址
  IIC_Wait_Ack();
  for (count = 0; count < length; count++) {
    IIC_Send_Byte(data[count]);
    IIC_Wait_Ack();
  }
  IIC_Stop();  //产生一个停止条件

  return 1;  // status == 0;
}

u8 IICreadByte(u8 dev, u8 reg, u8 *data) {
  *data = I2C_ReadOneByte(dev, reg);
  return 1;
}

u8 IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data) {
  return IICwriteBytes(dev, reg, 1, &data);
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8
data) *功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
                reg	   寄存器地址
                bitStart  目标字节的起始位
                length   位长度
                data    存放改变目标字节位的值
返回   成功 为1
                失败为0
*******************************************************************************/
u8 IICwriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data) {
  u8 b;
  if (IICreadByte(dev, reg, &b) != 0) {
    u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
    data <<= (8 - length);
    data >>= (7 - bitStart);
    b &= mask;
    b |= data;
    return IICwriteByte(dev, reg, b);
  } else {
    return 0;
  }
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
                reg	   寄存器地址
                bitNum  要修改目标字节的bitNum位
                data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1
                失败为0
*******************************************************************************/
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data) {
  u8 b;
  IICreadByte(dev, reg, &b);
  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  return IICwriteByte(dev, reg, b);
}
