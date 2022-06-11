#include "stm32f407xx.h"
#include "stdio.h"
#include "stdlib.h"

#define WHO_AM_I_ADDR 0x0F
#define CTRL_REG4_ADDR 0x20
#define READ_COMMAND 1<<7
#define WRITE_COMMAND 0
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B


int function_choice(int a);
void pwm();
void blinking_around();
void spi();
void accelerometer();
void init_spi();
int16_t x, y; // for x and y axis

	int main(void){
  
  RCC->CR |= RCC_CR_HSEON_Msk;  //making external clock ready
  RCC->CR |= RCC_CR_HSION;
  
  
  function_choice(2);   //first - configuration
 
  
  while((RCC->CR&0x00020000)==0){}
    
  FLASH->ACR |= 5;      //latency (delay) for MCU to write somethign in the memory - used when frequency is high
  FLASH->ACR |= 0x100;
  
  RCC->CFGR &= 0xFFFFFF0F;

  return 0;
}

int function_choice(int a)
{

  
  if(a == 1){
    //PLL=100MHz
    
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC;   // changing to the external clock (?)
      
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Msk;    //values here come from the formula in the reference manual p. 163
    RCC->PLLCFGR |= 96<<RCC_PLLCFGR_PLLN_Pos;
  
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk; 
    RCC->PLLCFGR |= 4<<RCC_PLLCFGR_PLLM_Pos;  //2

    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP_Msk; 
    RCC->PLLCFGR |= 2<<RCC_PLLCFGR_PLLP_Pos;
    
    RCC->CR |= RCC_CR_PLLON;   //second making PLL ready
    
    FLASH->ACR |= 5;
    
    uint32_t coreClock_1 = 0;
    RCC->CFGR |= RCC_CFGR_SW_1;        // switch for PLL
    SystemCoreClockUpdate();          // updating the core clock
    coreClock_1 = SystemCoreClock;    // changing the value
    SystemCoreClockUpdate(); 
    
  }else if(a == 0){
    
    RCC->CFGR |= RCC_CFGR_SW_0;   //changing the clock to the external one
    RCC->CFGR &= ~RCC_CFGR_SW_1;
    
  }else{
    
    RCC->CFGR &= ~RCC_CFGR_SW_0;   //changing the clock to the internal one
    RCC->CFGR &= ~RCC_CFGR_SW_1;
    
  }
  
  //-----------------------------------
  
  //LED config
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //enabling the clock - port D of the GPIO - LEDs
  
  //button config
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //enabling the button - port A of the GPIO - button
  GPIOA->MODER &= ~GPIO_MODER_MODE0_Msk;   
  

  int choice = 2;
  
  if(choice == 0) // blinking around option - init
  {
    //LED output settings
    GPIOD->MODER |= GPIO_MODER_MODER15_0; //Blue LED - setting pin 12 as output
    GPIOD->MODER |= GPIO_MODER_MODER14_0; //Red LED
    GPIOD->MODER |= GPIO_MODER_MODER13_0; //Orange LED
    GPIOD->MODER |= GPIO_MODER_MODER12_0; //Green LED 
    
    blinking_around();
  }
  else if(choice == 1) // pwm option - init
  {
    GPIOD->MODER |= GPIO_MODER_MODER12_1; //Green LED 
    GPIOD->MODER |= GPIO_MODER_MODER13_1; //Green LED 
    GPIOD->MODER |= GPIO_MODER_MODER14_1; //Green LED 
    GPIOD->MODER |= GPIO_MODER_MODER15_1; //Green LED 
    
    GPIOD->AFR[1] |= GPIO_AFRH_AFSEL12_1;
    GPIOD->AFR[1] |= GPIO_AFRH_AFSEL13_1;
    GPIOD->AFR[1] |= GPIO_AFRH_AFSEL14_1;
    GPIOD->AFR[1] |= GPIO_AFRH_AFSEL15_1;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN_Msk;  // turning on TIM4
    TIM4->CR1 |= TIM_CR1_CEN_Msk;
    TIM4->CCMR2 |= TIM_CCMR2_OC4CE_Msk;     //dioda 1
    TIM4->CCMR2 |= TIM_CCMR2_OC4M_2;    
    TIM4->CCMR2 |= TIM_CCMR2_OC4M_1; 

    TIM4->CCMR2 |= TIM_CCMR2_OC3CE_Msk;   
    TIM4->CCMR2 |= TIM_CCMR2_OC3M_2;        //dioda 2
    TIM4->CCMR2 |= TIM_CCMR2_OC3M_1;
    
    TIM4->CCMR1 |= TIM_CCMR1_OC2CE_Msk;   
    TIM4->CCMR1 |= TIM_CCMR1_OC2M_2;        //dioda 3
    TIM4->CCMR1 |= TIM_CCMR1_OC2M_1;
    
    TIM4->CCMR1 |= TIM_CCMR1_OC1CE_Msk;   
    TIM4->CCMR1 |= TIM_CCMR1_OC1M_2;        //dioda 4
    TIM4->CCMR1 |= TIM_CCMR1_OC1M_1;
    
    
    //TIM4->CCMR1 |= TIM_CCMR1_CC2S_0;
    TIM4->CCER |= TIM_CCER_CC4E_Msk;
    TIM4->CCER |= TIM_CCER_CC3E_Msk;
    TIM4->CCER |= TIM_CCER_CC2E_Msk;
    TIM4->CCER |= TIM_CCER_CC1E_Msk;

    TIM4->ARR |= 10000 << TIM_ARR_ARR_Pos;  // setting general duty cycle
    
    
    pwm();
  }
  
  
  else if(choice == 2) // spi/accelerometer option - init
  {
    init_spi(); 
    accelerometer();
  }
    
  
  return 1;
}


void init_spi(){
  // SPI1 Chip-Select setup (PE3)

  // enable GPIOE clock, bit 4 on AHB1ENR
  RCC->AHB1ENR |= (1 << 4);
  GPIOE->MODER &= 0xFFFFFF3F; // reset bits 6-7
  GPIOE->MODER |= 0x00000040; // set bits 6-7 to 0b01 (output)
  GPIOE->ODR |= (1 << 3);
  
  
  // enable alternative function for SPI 
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->AFR[0] = 0x55500000; // alternate function 5 for PA4-6
  
  GPIOA->MODER &= ~GPIO_MODER_MODER7_0; 
  GPIOA->MODER |= GPIO_MODER_MODER7_1; // SPI1_NSS
  GPIOA->MODER &= ~GPIO_MODER_MODER5_0;
  GPIOA->MODER |= GPIO_MODER_MODER5_1; // SPI1_SCK
  GPIOA->MODER &= ~GPIO_MODER_MODER6_0;
  GPIOA->MODER |= GPIO_MODER_MODER6_1; // SPI1_MISO
  
  
  // configure SPI
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  
  SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1; // baud rate 
  SPI1->CR1 |= SPI_CR1_DFF; // set 16-bit data frame format
  SPI1->CR1 |= SPI_CR1_MSTR; // set as master
  SPI1->CR1 |= SPI_CR1_CPOL; // CK to 1 when idle
  SPI1->CR1 |= SPI_CR1_CPHA; //  the first clock transition is the first data capture edge
  SPI1->CR1 |= SPI_CR1_SSM; // ssm enabled
  SPI1->CR1 |= SPI_CR1_SSI; // set ssi to 1
  //SPI1->CR1 |= SPI_CR1_LSBFIRST;
  //SPI1->CR2 |= SPI_CR2_FRF; // select TI Mode
  SPI1->CR1 |= SPI_CR1_SPE; // enable SPI
  
}

  uint8_t ReadData(uint8_t ACC_MEMSReg)   // alternative for the above function - working as well
{
	
	
	
    GPIOE->ODR |= GPIO_ODR_OD3;
    uint16_t data = ((uint16_t)(READ_COMMAND | ACC_MEMSReg)<<8);
    uint16_t i;
    for(i=0;i<100;i++);
    GPIOE->ODR &= ~GPIO_ODR_OD3;
    SPI1->DR = data;
    for(i=0;i<800;i++);
    GPIOE->ODR |= GPIO_ODR_OD3;
    return ACC_MEMSReg;
}

uint8_t read_data(uint8_t data)   // 8-bit unsigned integer, send data from accelerometer to microcontroller
{
    GPIOE->ODR &= ~GPIO_ODR_OD3; // chip select (low)
    SPI1->DR = 0x80 << 8 | data << 8;
    while ((SPI1->SR & SPI_SR_BSY)); // while it's busy
    GPIOE->ODR |= GPIO_ODR_OD3; // chip select (high)
    return SPI1->DR; // return what's inside data register
}


uint8_t write_data(uint8_t address, uint8_t data)   // 8-bit unsigned integer, send data from accelerometer to microcontroller
{
	  GPIOE->ODR &= ~GPIO_ODR_OD3; // chip select (low)
    SPI1->DR = address << 8 | data;
    while ((SPI1->SR & SPI_SR_BSY)); // while it's busy
    GPIOE->ODR |= GPIO_ODR_OD3; // chip select (high)
    return SPI1->DR; // return what's inside data register
}

   
  void accelerometer()
{
	  GPIOD->MODER |= GPIO_MODER_MODER12_1; //Green LED 
    GPIOD->MODER |= GPIO_MODER_MODER13_1;  
    GPIOD->MODER |= GPIO_MODER_MODER14_1;  
    GPIOD->MODER |= GPIO_MODER_MODER15_1; 
    
    GPIOD->AFR[1] |= GPIO_AFRH_AFSEL12_1;
    GPIOD->AFR[1] |= GPIO_AFRH_AFSEL13_1;
    GPIOD->AFR[1] |= GPIO_AFRH_AFSEL14_1;
    GPIOD->AFR[1] |= GPIO_AFRH_AFSEL15_1;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN_Msk;  // turning on TIM4
    TIM4->CR1 |= TIM_CR1_CEN_Msk;
    TIM4->CCMR2 |= TIM_CCMR2_OC4CE_Msk;     //dioda 1
    TIM4->CCMR2 |= TIM_CCMR2_OC4M_2;    
    TIM4->CCMR2 |= TIM_CCMR2_OC4M_1; 

    TIM4->CCMR2 |= TIM_CCMR2_OC3CE_Msk;   
    TIM4->CCMR2 |= TIM_CCMR2_OC3M_2;        //dioda 2
    TIM4->CCMR2 |= TIM_CCMR2_OC3M_1;
    
    TIM4->CCMR1 |= TIM_CCMR1_OC2CE_Msk;   
    TIM4->CCMR1 |= TIM_CCMR1_OC2M_2;        //dioda 3
    TIM4->CCMR1 |= TIM_CCMR1_OC2M_1;
    
    TIM4->CCMR1 |= TIM_CCMR1_OC1CE_Msk;   
    TIM4->CCMR1 |= TIM_CCMR1_OC1M_2;        //dioda 4
    TIM4->CCMR1 |= TIM_CCMR1_OC1M_1;
    
    
    //TIM4->CCMR1 |= TIM_CCMR1_CC2S_0;
    TIM4->CCER |= TIM_CCER_CC4E_Msk;
    TIM4->CCER |= TIM_CCER_CC3E_Msk;
    TIM4->CCER |= TIM_CCER_CC2E_Msk;
    TIM4->CCER |= TIM_CCER_CC1E_Msk;

    TIM4->ARR |= 10000 << TIM_ARR_ARR_Pos;  // setting general duty cycle
			
    read_data(WHO_AM_I_ADDR);
		
		write_data(CTRL_REG4_ADDR, 0x17);   // turning on the accelerometer
	
    while(1)
    {

        x = read_data(OUT_X_L);
        x = x | read_data(OUT_X_H)<< 8;

        y = read_data(OUT_Y_L);
        y = y | read_data(OUT_Y_H)<< 8;
			
			  TIM4->CCR1 = 500;
				TIM4->CCR2 = 500;
				TIM4->CCR3 = 500;
				TIM4->CCR4 = 500;
			
        if (x > 0)
        {
          TIM4->CCR1 = 250;
					TIM4->CCR2 = 250;
					TIM4->CCR3 = 250;
					TIM4->CCR4 = 10000;
        }

				else if (x < 0)
        {
          TIM4->CCR1 = 250;
					TIM4->CCR2 = 250;
					TIM4->CCR3 = 10000;
					TIM4->CCR4 = 250;
        }

        if (y > 0)
        {
          TIM4->CCR1 = 10000;
					TIM4->CCR2 = 250;
					TIM4->CCR3 = 250;
					TIM4->CCR4 = 250;
        }

        else if (y < 0)
        {
          TIM4->CCR1 = 250;
					TIM4->CCR2 = 10000;
					TIM4->CCR3 = 250;
					TIM4->CCR4 = 250;
        }
				
				
    }
}
  


  void pwm(void)
  {
    int flag = 0;
    while(1){  
      if(GPIOA->IDR & GPIO_IDR_ID0_Msk) 
        {  
          if(flag==0){
          flag = 1;}
          else{
          flag = 0;}
        }      
        
      if (flag == 0)
      {
        TIM4->CCR1 = 250;
        TIM4->CCR2 = 500;
        TIM4->CCR3 = 750;
        TIM4->CCR4 = 1000;
      }
      else   // changing the intensity (duty cycle) of diodes with button press
      {        
        TIM4->CCR1 = 9000;
        TIM4->CCR2 = 9500;
        TIM4->CCR3 = 975;
        TIM4->CCR4 = 10000;
      }
    }
  }


  void blinking_around(void)
  {
  
    int counter = 0;
    int flag = 0;
    
    while(counter < 100){
      
      if(GPIOA->IDR & GPIO_IDR_ID0_Msk) 
      {  
        if(flag==0){
        flag = 1;}
        else{
        flag = 0;}
      }
      
      if (flag==0){
        
        //counter_clockwise
        GPIOD->BSRR = 1<<15; // turning on a LED (setting 12 bit to 1)
        
        for(int i = 0; i < 100000; i++){} // creating a delay
          
        GPIOD->BSRR = 1<<(15+16); // turning off a LED (setting 12+16 bit to 1)
          
        for(int j = 0; j < 100000; j++){} 
          
        GPIOD->BSRR = 1<<14;
          
        for(int j = 0; j < 100000; j++){}
          
        GPIOD->BSRR = 1<<(14+16);
          
        for(int j = 0; j < 100000; j++){}
          
        GPIOD->BSRR = 1<<13;
          
        for(int j = 0; j < 100000; j++){}
          
        GPIOD->BSRR = 1<<(13+16);
          
        for(int j = 0; j < 100000; j++){}
          
        GPIOD->BSRR = 1<<12;
          
        for(int j = 0; j < 100000; j++){}
          
        GPIOD->BSRR = 1<<(12+16);
          
        for(int j = 0; j < 100000; j++){}

        counter++;
        }
      else{
      
        //clockwise
        GPIOD->BSRR = 1<<12; // turning on a LED (setting 12 bit to 1)
        
        for(int i = 0; i < 100000; i++){} // creating a delay
          
        GPIOD->BSRR = 1<<(12+16); // turning off a LED (setting 12+16 bit to 1)
          
        for(int j = 0; j < 100000; j++){} 
          
        GPIOD->BSRR = 1<<13;
          
        for(int j = 0; j < 100000; j++){}
          
        GPIOD->BSRR = 1<<(13+16);
          
        for(int j = 0; j < 100000; j++){}
          
        GPIOD->BSRR = 1<<14;
          
        for(int j = 0; j < 100000; j++){}
          
        GPIOD->BSRR = 1<<(14+16);
          
        for(int j = 0; j < 100000; j++){}
          
        GPIOD->BSRR = 1<<15;
          
        for(int j = 0; j < 100000; j++){}
          
        GPIOD->BSRR = 1<<(15+16);
          
        for(int j = 0; j < 100000; j++){}

        counter++;
          
        }

      }
  }
 
