 #ifndef __MAIN_H
#define	__MAIN_H

//---Переопределяем порты для подключения дисплея, для удобства---//

#define     LCM_OUT               GPIOB->ODR
#define     LCM_PIN_RS            RS_Pin         // PB0
#define     LCM_PIN_EN            EN_Pin         // PB1
#define     LCM_PIN_D7            DB7_Pin        // PB7
#define     LCM_PIN_D6            DB6_Pin        // PB6
#define     LCM_PIN_D5            DB5_Pin        // PB5
#define     LCM_PIN_D4            DB4_Pin        // PB4
#define     LCM_PIN_MASK  ((LCM_PIN_RS | LCM_PIN_EN | LCM_PIN_D7 | LCM_PIN_D6 | LCM_PIN_D5 | LCM_PIN_D4))

 void delay(int a);
 void PulseLCD(void);
 void SendByte(char ByteToSend, int IsData);
 void Cursor(char Row, char Col);
 void ClearLCDScreen(void);
 void InitializeLCD(void);
 void PrintStr(char *Text);
 void PrintMassiv(char* st);
 
#endif
