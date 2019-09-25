
// I2C1 Slave source code for STM32F446RETx
// I2C1 SCL PB8
// I2C1 SDA PB9

#include "stm32f10x.h"                   // Device header
#include "delay.h"

#define SLAVE_ADDR		0x41							// Slave address (OAR1), 0110000

void I2C1_init(void);
void LED_init(void);
void delayMs(int n);

void I2C1_EV_IRQHandler(void) {
	volatile uint16_t temp;		
	I2C1->CR2 &= ~(1 << 9);
	if(I2C1->SR1 & 2) 										// ADDR bitinin 1 olmasi
	{
		temp = I2C1->SR1;										// ADDR bayragini temizlemek için.
		temp = I2C1->SR2;										// ADDR bayragini temizlemek için.
	}
	while(!(I2C1->SR1 & (1 << 6)));
	
	temp = I2C1->DR;											// 
	I2C1->CR1 |= 0x200;										// Generate stop after data received
	
	if(temp == 0x12) {									
		GPIOA->BSRR = 0x00000008;						// Turn on Led PA3	
		delay_ms(20);
		GPIOA->BSRR = 0x00080000;						// Turn off Led PA3

   		
	}
	temp = I2C1->SR1;											// 
	temp = I2C1->SR2;											// 
	
	I2C1->CR1 |= 0x0001;									// Enable I2C1 module
	I2C1->CR2 |= 1 << 9;
}

int main(void) {
	delay_init();
	__disable_irq();	
	I2C1_init();
	LED_init();	
	
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	
	__enable_irq();
		
	while(1) {
		
		}
	}																			
																		

void I2C1_init(void) {	
	
	
	RCC->APB2ENR   |=1<<3;//B portu aktif.
	RCC->APB1ENR |= 1<<21; //I2C aktif.
	RCC->APB2ENR   |=1<<0;//AFIO portu aktif.
	
	
	GPIOB->CRH = 0;
	GPIOB->CRH = 0x000000FF; //SCL ve SDA  AF Open Drain  SCL => PB6  SDA =>PB7
	
	
	I2C1->CR1   =  0x8000;								// Software reset I2C1
	I2C1->CR1  &= ~0x8000;								// Out of reset
	I2C1->CR2    =36;
	I2C1->CCR    =180;
	I2C1->CR2  |= 1 << 9;								// ITEVTEN 
	I2C1->CR2  |= 1 << 10;								// ITBUFEN
	I2C1->OAR1 |= (SLAVE_ADDR << 1);			// Set Own Address Register
	I2C1->OAR1 |= 1 << 14;
  I2C1->CR1     |=(1<<0);//Peripheral aktif. 
	
		
	
	I2C1->CR1  |= 1 << 10;								// ACK Enable
}




void LED_init(void) {
	
	RCC->APB2ENR   |=1<<2;//A portu aktif.
	GPIOA->CRL =0;
	GPIOA->CRL = 0x00002000; //PA3 
}



/*
#include "stm32f10x.h"                  // Device header
#include "delay.h"


void pinAyarlari(void);
void i2c_Slave_Ayarlari(void);
void DMA_Ayarlari(void);
uint8_t veriAl(void);
void Ayar(void);

uint8_t data;

void I2C1_EV_IRQHandler(void)
{
 
	volatile uint16_t gecici;
	
  I2C1->CR2 &= ~(1 << 9);
	
	if(I2C1->SR1 &(1<<1)) //ADDR bitinin 1 olmasini bekle.Adres eslesme biti.
	{
	gecici=I2C1->SR1;//ADDR bayragini temizlemek için.	
	gecici=I2C1->SR2;//ADDR bayragini temizlemek için.
	//GPIOA->BRR  |=1<<3;
		
	}
	
	while(!(I2C1->SR1 & (1 << 6)));
	
	gecici = I2C1->DR;											// 
	I2C1->CR1 |= 0x200;										// Generate stop after data received
	
	if(gecici == 8) {									
		GPIOA->BSRR = 0x00080000;						
		
		
	}
	 if(gecici == 0) {		
    GPIOA->BSRR = 0x00000008;		
									
		
	} 
	 else{
		 GPIOA->BSRR = 0x00080000;		
	 }
	gecici = I2C1->SR1;											
	gecici = I2C1->SR2;											 
	
								
	I2C1->CR1 |= 0x200;	
  I2C1->CR1 |= 0x0001;									// Enable I2C1 module
	I2C1->CR2 |= 1 << 9;
	

}

int main()
	
{

	__disable_irq();	
		
	//pinAyarlari();	
  //i2c_Slave_Ayarlari();
	
	Ayar();
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	
	__enable_irq();
	

	
 
		
	//NVIC->ISER[0] |=0x80000000;//i2c1 interrupt AKTIF.
	//NVIC->IP[31]   =(1<<4) & 0xFF;
	
	
	

	while(1)
	{
	
	 	
		
		
		
		if(data==0)
			GPIOA->BSRR |=1<<3;
		
		else if(data==1)
			GPIOA->BRR  |=1<<3;
		
		
	
	}

}

void Ayar()
{
	RCC->APB1ENR |= 1<<21; //I2C aktif.
	RCC->APB2ENR   |=1<<2;//A portu aktif.
	RCC->APB2ENR   |=1<<3;//B portu aktif.
	RCC->APB2ENR   |=1<<0;//AFIO portu aktif.
	
	
	
	GPIOA->CRL |= 0x00002000; 
	GPIOA->BSRR |=1<<3;
	GPIOB->CRL = 0xFFF00000; //SCL ve SDA  AF Open Drain  SCL => PB6  SDA =>PB7
	//GPIOB->CRH = 0;
	
	 I2C1->CR1 |= (1 << 15);
   I2C1->CR1 &=~(1 << 15);
	 I2C1->CR2    =36;
	 I2C1->CCR    =180;
	 I2C1->CR2  |= 1 << 9;								// ITEVTEN 
	 I2C1->CR2  |= 1 << 10;								// ITBUFEN
	 I2C1->OAR1 |= (0x0C << 1);			// Set Own Address Register
	 I2C1->OAR1 |= 1 << 14;
   //I2C1->CR1     |=(1<<0);//Peripheral aktif. 
	 I2C1 ->CR1     |= 1<<10;//ACK bit.
	 I2C1->CR1      |=1<<13;
	 I2C1->CR1     |=(1<<4);
	 I2C1->CR1     |=(1<<1);
	
	
	
	
	
	
	
	
}

void pinAyarlari()
{
	
	
	RCC->APB2ENR   |=1<<2;//A portu aktif.
	RCC->APB2ENR   |=1<<3;//B portu aktif.
	
	RCC->APB2ENR   |=1<<0;//AFIO aktif.
	
	GPIOB->CRH=0;
	//GPIOB->CRH |= 0x000000FF; //SCL ve SDA  AF Open Drain  SCL => PB8  SDA =>PB9
	
	
  //GPIOB->CRL &= 0x00FFFFFF; //PB6 I2C1_SCL ,PB7 I2C1_SDL
	GPIOB->CRL |= 0xFF000000;
	GPIOB->BRR |=1<<6;
	GPIOB->BRR |=1<<7;
	GPIOA->CRL |= 0x00002000; //PA3 led.
	GPIOA->BSRR |=1<<3;//Ledi Sondur.
	//GPIOB->CRL = 0xDD000000; //SCL ve SDA  AF Open Drain  SCL => PB6  SDA =>PB7
	

}

void DMA_Ayarlari()
{
	RCC->AHBENR |=1<<0;//DMA hatti aktif.
	
	DMA1_Channel1->CCR |=1<<13;//High priority
	DMA1_Channel1->CCR &=~(1<<12);//High priority
	DMA1_Channel1->CCR &=~(1<<11);//16 bit hafiza boyutu.
	DMA1_Channel1->CCR  |=(1<<10);//16 bit hafiza boyutu.
	DMA1_Channel1->CCR &=~(1<<9);//16 bit çevresel boyutu.
	DMA1_Channel1->CCR  |=(1<<8);//16 bit çevresel boyutu.
	DMA1_Channel1->CCR |=1<<5;//Circular mode.
	DMA1_Channel1->CCR &=~(1<<4);//Çevresel okuma.
	DMA1_Channel1->CNDTR =1;//Transfer edilecek data sayisi.
	
	DMA1_Channel1->CPAR =(uint32_t) &I2C1->DR; //Buradan okunacak.
	DMA1_Channel1->CMAR =(uint32_t) &data;//Buraya yollayacak.
	
	//memory adresi artirilabilir inc modu.
  DMA1_Channel1->CCR |=(1<<0);//DMA Enable.
	



}

void i2c_Slave_Ayarlari()
{
	
	 RCC->APB1ENR |= 1<<21; 
   RCC->APB1RSTR |= 1<<21; 
	 delay_ms(1000);
   RCC->APB1RSTR &= ~(1<<21); 
	
	
	 I2C1->CR1 |= (1 << 15);
   I2C1->CR1 &=~(1 << 15);
	 I2C1->CR2    = 0x0018; //0X08   36 Mhz peripheral clock.// I2C1->CR2    |=1<<5; //36 Mhz peripheral clock.I2C1->CR2    |=1<<2; //36 Mhz peripheral clock.
	 
	 I2C1->CR2  |= 1 << 9;									// ITEVTEN 
	 I2C1->CR2  |= 1 << 10;								// ITBUFEN
	 
	 I2C1->OAR1 |= (0x41 << 1);			// Set Own Address Register
	 I2C1->OAR1 |= 1 << 14;
   I2C1->CR1     |=(1<<0);//Peripheral aktif. 
		
	 
	 I2C1 ->CR1     |= 1<<10;//ACK bit.
	
	
	
	
}

uint8_t veriAl()

{
	
	
	volatile int gecici;
	uint8_t gelenData;

	
	
	
	while(!(I2C1->SR1 &(1<<1))){}//ADDR bitinin 1 olmasini bekle.Adres eslesme biti.
	gecici=I2C1->SR1;//ADDR bayragini temizlemek için.	
	gecici=I2C1->SR2;//ADDR bayragini temizlemek için.
	gelenData=1;
		
		
	while (!(I2C1->SR1 & (1<<6))){} //RXNE 1 olmasini bekle.
	gelenData=I2C1->DR;
		
  		
	while (!(I2C1->SR1 & (1<<4))){} //STOPF 1 olmasini bekle.
	gecici=I2C1->SR1;
		
	I2C1->CR1 |=1<<9;
 

  
  return gelenData;		
	
	


}
*/




