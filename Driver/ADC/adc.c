#include "adc.h"

s32 Voltage;

void Adc_Init(void) {
	ADC_InitTypeDef ADC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  //ʹ��ADC1ͨ��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
  //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);
  // PA4 ��Ϊģ��ͨ����������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  //ģ����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
  ADC_DeInit(ADC1);
  // ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  //ģ��ת�������ڵ�ͨ��ģʽ
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  //ģ��ת�������ڵ���ת��ģʽ
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  //ת��������������ⲿ��������
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  // ADC�����Ҷ���
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  //˳����й���ת����ADCͨ������Ŀ
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���
  ADC_Init(ADC1, &ADC_InitStructure);
  //ʹ��ָ����ADC1
  ADC_Cmd(ADC1, ENABLE);
  //ʹ�ܸ�λУ׼
  ADC_ResetCalibration(ADC1);
  //�ȴ���λУ׼����
  while (ADC_GetResetCalibrationStatus(ADC1));
  //����ADУ׼
  ADC_StartCalibration(ADC1);
  //�ȴ�У׼����
  while (ADC_GetCalibrationStatus(ADC1));
}

u16 Get_Adc(u8 ch) {
  //����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
  // ADC1,ADCͨ��,����ʱ��Ϊ239.5����
  ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);
  //ʹ��ָ����ADC1�����ת����������
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  //�ȴ�ת������
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  //�������һ��ADC1�������ת�����
  return ADC_GetConversionValue(ADC1);
}

int Get_battery_volt(void) {
  //�����ѹ���������ԭ��ͼ�򵥷������Եõ�
  return Get_Adc(Battery_Ch) * 3.3 * 11.5 * 100 / 1.5 / 4096;
}
