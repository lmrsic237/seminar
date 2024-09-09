#include "main.h"
#include "gpio.h"
#include <stdint.h>

volatile uint8_t count = 0;
volatile uint8_t motionDetected = 0;

void SystemClock_Config(void);
void displayDigit(uint8_t digit);
void EXTI9_5_IRQHandler(void);
void init_GPIO(void);
void init_EXTI(void);
void turnOffLED(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  init_GPIO();
  init_EXTI();

  while (1)
  {
	  if (motionDetected) {
	      HAL_Delay(1000);
	      turnOffLED();
	      motionDetected = 0;
	          }
  }

}
void init_GPIO(void) {

    __HAL_RCC_GPIOA_CLK_ENABLE();

    //Konfiguriranje pinova PA0-PA6 kao output za 7 segmenti display
    GPIOA->MODER &= ~(0x000000FF);
    GPIOA->MODER |= 0x00000055;

    // Konfiguriranje pina PA7 kao input za senzor pokreta
    GPIOA->MODER &= ~(0x00030000);
    GPIOA->PUPDR |= (0x00010000);

    // Konfiguriranje pina PA8 kao output za ledicu
    GPIOA->MODER &= ~(0x000C0000);
    GPIOA->MODER |= 0x00040000;
}
void init_EXTI(void) {

    __HAL_RCC_SYSCFG_CLK_ENABLE();

        SYSCFG->EXTICR[1] &= ~(0x0F);
        SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PA;

        EXTI->IMR |= EXTI_IMR_IM7;
        EXTI->FTSR |= EXTI_FTSR_TR7;

        NVIC_EnableIRQ(EXTI9_5_IRQn);
}
void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR7) {
        EXTI->PR |= EXTI_PR_PR7;

        count++;
        if (count > 9) {
            count = 0; //Resetiranje brojanja kada dođe do 9
        }

        displayDigit(count);

        // Uključivanje ledice kada se pokret detektira
        GPIOA->ODR |= (1 << 8); // Postavljanje ledice u HIGH state
        motionDetected = 1;
    }
}
void displayDigit(uint8_t digit) {
    GPIOA->ODR &= ~(0x0000007F);

    switch (digit) {
        case 0: GPIOA->ODR |= 0b00111111; break; // 0
        case 1: GPIOA->ODR |= 0b00000110; break; // 1
        case 2: GPIOA->ODR |= 0b01011011; break; // 2
        case 3: GPIOA->ODR |= 0b01001111; break; // 3
        case 4: GPIOA->ODR |= 0b01100110; break; // 4
        case 5: GPIOA->ODR |= 0b01101101; break; // 5
        case 6: GPIOA->ODR |= 0b01111101; break; // 6
        case 7: GPIOA->ODR |= 0b00000111; break; // 7
        case 8: GPIOA->ODR |= 0b01111111; break; // 8
        case 9: GPIOA->ODR |= 0b01101111; break; // 9
    }
}
// Funkcija za isključivanje ledice
void turnOffLED(void) {
    GPIOA->ODR &= ~(1 << 8);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

#endif



