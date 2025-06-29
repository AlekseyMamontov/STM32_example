

#include "main.h"
#include "stm32g4xx.h"
#include "INIT_STM32G473_GPIO.h"
#include <CANFD_STM32G473.h>

uint8_t led_test = 0;

int main(void)
{
	SystemClock_Config();
	GPIO_INIT();

  while (1)
  {
	if(!systick_pause){

    GPIOB->BSRR = led_test? 1 << 11 : 1 <<27;
    led_test = led_test?0:1;
    systick_pause = 500;

    }


  }


}



void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  //disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
