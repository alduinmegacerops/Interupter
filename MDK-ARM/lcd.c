#include "lcd.h"
#include "main.h"
#include "stm32f1xx_hal.h"

GPIO_InitTypeDef  GPIO_InitStructure;

//---Функция задержки---//
void delay(int a)
{
    int i = 0;
    int f = 0;
    while(f<a)
    {
        while(i<60)
        	{i++;}
        f++;
    }
}

//---Нужная функция для работы с дисплее, по сути дергаем ножкой EN---//
void PulseLCD()
{
    LCM_OUT &= ~LCM_PIN_EN;
    delay(220);
    LCM_OUT |= LCM_PIN_EN;
    delay(220);
    LCM_OUT &= (~LCM_PIN_EN);
    delay(220);
}

//---Отсылка байта в дисплей---//
void SendByte(char ByteToSend, int IsData)
{
    LCM_OUT &= (~LCM_PIN_MASK);
    LCM_OUT |= (ByteToSend & 0xF0);

    if (IsData == 1)
        LCM_OUT |= LCM_PIN_RS;
    else
        LCM_OUT &= ~LCM_PIN_RS;
    PulseLCD();
    LCM_OUT &= (~LCM_PIN_MASK);
    LCM_OUT |= ((ByteToSend & 0x0F) << 4);

    if (IsData == 1)
        LCM_OUT |= LCM_PIN_RS;
    else
        LCM_OUT &= ~LCM_PIN_RS;

    PulseLCD();
}

//---Установка позиции курсора---//
void Cursor(char Row, char Col)
{
    char address;
		switch(Row)
		{
			case 0:
				address = 0;
			break;
			
			case 1:
				address = 0x40;
			break;
			
			case 2:
				address = 0x14;
			break;
			
			case 3:
				address = 0x54;
		}
    /*if (Row == 0)
        address = 0;
    else
        address = 0x40;*/
    address |= Col;
    SendByte(0x80 | address, 0);
}

//---Очистка дисплея---//
void ClearLCDScreen()
{
    SendByte(0x01, 0);
    SendByte(0x02, 0);
}

//---Инициализация дисплея---//
void InitializeLCD(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStructure.Pin =RS_Pin | EN_Pin| DB4_Pin | DB5_Pin | DB6_Pin | DB7_Pin;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    LCM_OUT &= ~(LCM_PIN_MASK);
    delay(32000);
    delay(32000);
	delay(32000);
    LCM_OUT &= ~LCM_PIN_RS;
	LCM_OUT &= ~LCM_PIN_EN;
	LCM_OUT = 0x20;
    PulseLCD();
    SendByte(0x28, 0);
    SendByte(0x0E, 0);
    SendByte(0x06, 0);
	SendByte(0x02, 0);
}

//---Печать строки---//
void PrintStr(char *Text)
{
    char *c;
    c = Text;
    while ((c != 0) && (*c != 0))
    {
        SendByte(*c, 1);
        c++;
    }
}

void PrintMassiv(char* st)
{
    uint8_t i=0;
    while (st[i]!=0)
    {
        SendByte(st[i], 1);
				delay(1);
        i++;
    }
}
