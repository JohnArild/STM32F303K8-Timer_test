/*
 * main.c
 *
 *  Created on: Dec 11, 2018
 *      Author: John
 */

#define PA12_On *(uint32_t *)0x48000018 = 0x00001000
#define PA12_Off *(uint32_t *)0x48000018 = 0x10000000

#define PERIPH_BASE     ((uint32_t)0x40000000)
#define AHB2PERIPH_BASE (PERIPH_BASE + 0x08000000)
#define GPIOA_BASE      (AHB2PERIPH_BASE + 0x0000)
#define GPIOA_MODER     (*(uint32_t *)GPIOA_BASE)
#define Pin12_MODER_Output  ((uint32_t)0x01000000)

#include "stm32f30x.h"
void delay (int a);
  
void delay (int a)
{
    volatile int i,j;

    for (i=0 ; i < a ; i++)
    {
        j++;
    }

    return;
}

int main(){
    //RCC->CFGR |= (RCC_CFGR_SW_PLL | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLMULL16); // full speed, requiers flash latency
    RCC->CFGR |= (RCC_CFGR_SW_PLL | RCC_CFGR_PLLMULL6); // 24MHz, no latency needed
    //FLASH->ACR |= FLASH_ACR_LATENCY_1;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->CR |= RCC_CR_PLLON;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA_MODER |= Pin12_MODER_Output; // set pin PA_12 as output
    //GPIOB-> OTYPER |= GPIO_OTYPER_OT_3;
    //GPIOA->OSPEEDR = 0xFFFFFFFFU; //Speed up GPIO slew rate, not needed for 24MHz
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR11_0; //Set pullup on PA_11

    //Configure SPI
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (*(uint32_t *)0x48000020) = 0x55550000; //Sert PA_4 through PA_7 as atlernate function AF5
    //GPIOA->MODER |= GPIO_MODER_MODER4_1; // set pin PA_4 as alternate function mode (NSS)
    GPIOA->MODER |= GPIO_MODER_MODER4_0; // set pin PA_4 as output (NSS)
    GPIOA->MODER |= GPIO_MODER_MODER5_1; // set pin PA_5 as alternate function mode (SCK)
    GPIOA->MODER |= GPIO_MODER_MODER6_1; // set pin PA_6 as alternate function mode (MISO)
    GPIOA->MODER |= GPIO_MODER_MODER7_1; // set pin PA_7 as alternate function mode (MOSI)
    SPI1->CR1 |= SPI_CR1_BR_1; // Set Pclk/8 Pclk=24MHz
    SPI1->CR1 |= SPI_CR1_MSTR; // Set SPI1 as master
    SPI1->CR1 |= SPI_CR1_SSM; // Software Slave Select
    SPI1->CR2 |= SPI_CR2_DS; // Set Bit lenght to 16 bit
    SPI1->CR2 |= SPI_CR2_SSOE; // Enable SS output
    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
    
    //Configure Timer2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable Timer2 peripheral clock
    (*(uint32_t *)0x48000024) = 0x00000A00; //Sert PA_10 as atlernate function AF10 (TIM2_CH4)
    GPIOA->MODER |= GPIO_MODER_MODER10_1; // Set PA_10 as alernate mode
    TIM2->CR1 |= TIM_CR1_CEN; // Enable Timer
    TIM2->CR2 |= TIM_CR2_TI1S; // CH1,2,3 connected to TI1
    //TIM2->CR2 |= TIM_CR2_MMS_2; // Saved for late, Master mode for controlling second timer
    TIM2->SMCR |= TIM_SMCR_TS_2; // Trigger selection: TI1F
    

    PA12_Off;
    GPIOA->BSRR = GPIO_BSRR_BR_4;
    //SPI1->DR = 0xDEAD;
    //SPI1->DR = (uint8_t)0x01;
    SPI1->DR = 0x0140;
    delay(10);
    int tempe1 = SPI1->DR;
    SPI1->DR = 0x00;
    delay(10);
    tempe1 = SPI1->DR;
    GPIOA->BSRR = GPIO_BSRR_BS_4;
    PA12_On;
    PA12_Off;
    PA12_On;
    
    int inputsA = 0;
    float temperature1 = 0;
  while (1)
  {
      /* Set PC8 and PC9 */
      //GPIOA->BSRR = GPIO_BSRR_BS_12;

      //delay(500000);
      /* Reset PC8 and PC9 */

      //delay(500000);
      inputsA = GPIOA->IDR;
      if((inputsA & (1 << 11)) >> 11){
        PA12_On;
      }
      else{ 
        PA12_Off;
        GPIOA->BSRR = GPIO_BSRR_BR_4;
        SPI1->DR = 0x0;
        delay(10);
        tempe1 = SPI1->DR;
        temperature1 = tempe1 * 0.03125;
        GPIOA->BSRR = GPIO_BSRR_BS_4;
      }
  }
  //return 0;
}
