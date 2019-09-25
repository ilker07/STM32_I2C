


#include  "stm32f10x.h"                   // Device header
#include "delay.h"

#define SLAVE_ADDR	0x41								// Slave address

void I2C1_init(void);
int I2C1_byteWrite(char saddr, char data);


int main(void) {
	delay_init();
	//2C1_init();		
	
	RCC->APB2ENR   |=1<<2;//A portu aktif.
	
	
	GPIOA->CRL |= 0x00008020; //PA3 buton pull-down	
	
	while(1) {
		//I2C1_byteWrite(SLAVE_ADDR, 0x12);
		GPIOA->BRR |=1<<1;
		delay_ms(10);		
		GPIOA->BSRR |=1<<1;
		delay_ms(10);
	}	
}																				// End of main

void I2C1_init(void) {
	
	
	
	RCC->APB2ENR   |=1<<2;//A portu aktif.
	RCC->APB2ENR   |=1<<3;//B portu aktif.
	RCC->APB1ENR |= 1<<21; //I2C aktif.
	RCC->APB2ENR   |=1<<0;//AFIO portu aktif.
	
	GPIOA->CRL |= 0x00008020; //PA3 buton pull-down
	//GPIOA->BSRR |=1<<1;
	GPIOB->CRH = 0x000000FF; //SCL ve SDA  AF Open Drain  SCL => PB6  SDA =>PB7
	//GPIOB->CRH = 0;
	
	I2C1->CR1   = 0x8000;									// Software reset I2C1
	I2C1->CR1  &= ~0x8000;								// Out of reset
	I2C1->CR1 |= (1 << 15);
  I2C1->CR1 &=~(1 << 15);
	I2C1->CR2    =36;
	I2C1->CCR    =180;
	I2C1->TRISE  =37;
	
	
	

	I2C1->CR1     |=(1<<0);//Peripheral aktif. 	
}

int I2C1_byteWrite(char saddr, char data) {
	volatile int temp;
	
	while(I2C1->SR2 & 2);									// Wait until bus not busy
	
	I2C1->CR1 |= 0x100;										// Generate start
	while(!(I2C1->SR1 & 1));							// Wait until start flag is set
	
	I2C1->DR = saddr << 1;								// Transmit slave address
	while(!(I2C1->SR1 & 2));							// Wait until addr flag is set
	temp = I2C1->SR2;											// Clear addr flag
	
	while(!(I2C1->SR1 & 0x80));						// Wait until data register empty
	
	I2C1->DR = data;
	while(!(I2C1->SR1 & 4));							// Wait until transfer finished
	
	I2C1->CR1 |= 0x200;										// Generate stop
	return 0;
}





/*
#include "stm32f10x.h"                  // Device header
#include "delay.h"

void pinAyarlari(void);
void i2c_Master_Ayarlari(void);
void veriGonder(uint8_t data);
void Ayar(void);


int main()
{
	
	delay_init();
	Ayar();
  //pinAyarlari();
	//i2c_Master_Ayarlari();
	//veriGonder();//uint8_t data
	
		
	
	while(1)
	{
		
		
	
		
		
		
		uint8_t butonDurum=GPIOA->IDR & 0x00001000;	//Buton Okuma.
		veriGonder(butonDurum);
		
		
		
	}



}
void Ayar()
{
	RCC->APB1ENR |= 1<<21; //I2C aktif.
	RCC->APB2ENR   |=1<<2;//A portu aktif.
	RCC->APB2ENR   |=1<<3;//B portu aktif.
	
	RCC->APB2ENR   |=1<<0;//AFIO portu aktif.
	
	GPIOA->CRL |= 0x00008020; //PA3 buton pull-down
	//GPIOA->BSRR |=1<<1;
	GPIOB->CRL = 0xFF000000; //SCL ve SDA  AF Open Drain  SCL => PB6  SDA =>PB7
	//GPIOB->CRH = 0;
	
	I2C1->CR1 |= (1 << 15);
  I2C1->CR1 &=~(1 << 15);
	I2C1->CR2    =36;
	I2C1->CCR    =180;
	I2C1->TRISE  =37;
	
	
	

	//I2C1->CR1     |=(1<<0);//Peripheral aktif. 
	I2C1->CR1     |=(1<<3);
	I2C1->CR1     |=(1<<4);
	I2C1->CR1     |=(1<<1);
	
}

void pinAyarlari()
{
	
	RCC->APB2ENR   |=1<<2;//A portu aktif.
	RCC->APB2ENR   |=1<<3;//B portu aktif.
	RCC->APB1ENR |= 1<<21; 
	
	
	
	
	//RCC->APB2ENR   |=1<<0;//AFIO aktif.	
	
  //GPIOB->CRL &= 0x00FFFFFF; //PB6 I2C1_SCL ,PB7 I2C1_SDL
	//GPIOB->CRL |= 0xFF000000;
	//GPIOA->CRL |= 0x00008020; //PA3 buton pull-down 
	GPIOA->BSRR |=1<<1;
	GPIOB->CRL = 0xFF000000; //SCL ve SDA  AF Open Drain  SCL => PB6  SDA =>PB7
	GPIOB->CRH =0;
	GPIOB->BRR |=1<<6;
	GPIOB->BRR |=1<<7;
	//GPIOB->CRH |= 0x000000FF; //SCL ve SDA  AF Open Drain  SCL => PB8  SDA =>PB9  dd
	
	
	
}
void i2c_Master_Ayarlari()
{
	
  
   I2C1->CR1 |= (1 << 15);
   I2C1->CR1 &=~(1 << 15);
	 
	
	 I2C1->CR2    = 0x0018; //0X08   36 Mhz peripheral clock.// I2C1->CR2    |=1<<5; //36 Mhz peripheral clock.I2C1->CR2    |=1<<2; //36 Mhz peripheral clock.
	 I2C1->CCR     =0x28;//100 khz clock  ???????  0X28
	 I2C1->TRISE   =0x0019;//1/8MHZ= 125 ns  => 1000ns/125ns =8  => 8+1 =9 0X09 
   I2C1->CR1     |=(1<<0);//Peripheral aktif. 
		
	 

	
	 

}
void veriGonder(uint8_t data)
{
	
  volatile uint32_t gecici;
	
  
	while(I2C1->SR2 &(1<<1));//Iletisim kontrolu.BUSY biti.
	
	I2C1->CR1  |=1<<8;//START biti.
	
	while(!(I2C1->SR1 & (1<<0))); //Start bitinin 1 olmasini bekle.
	//GPIOA->BRR |=1<<1;
	//gecici=I2C1->SR1;//Start bitini temizlemek için.
	I2C1->DR  = 0x0C << 1;//Start bitini temizlemek için  ve 7 bit adres yollandi.
	
	
	while(!(I2C1->SR1 &(1<<1)));//ADDR bitinin 1 olmasini bekle.Adres eslesme biti.
	
	gecici=I2C1->SR1;//ADDR  bitini temizlemek için.
	gecici=I2C1->SR2;//ADDR  bitini temizlemek için.
	
	
	
	I2C1->DR = memoryadress;//  
	//while (!(I2C1->SR1 & (1<<7))){} //TXE 1 olmasini bekle.

	I2C1->DR = data;//Veriyi yaz.  
	while (!(I2C1->SR1 & (1<<7))){} //TXE 1 olmasini bekle.
                       
    
  //while (!(I2C1->SR1 & (1<<2)));//BTF(Byte transfer finished)=1 olmasini bekle. 
  I2C1->CR1 |= 1<<9;//STOP bitini 1 yap.
	//while(I2C1->CR1 &(1<<9));//STOP Bitinin 0 olmasini bekle.
	
	
	

}

*/
