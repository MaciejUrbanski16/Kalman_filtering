#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "lcd_display.h"
#include "magnetometr.h"

#define LD2_PIN GPIO_PIN_5
#define LD2_GPIO_PORT GPIOA

GPIO_InitTypeDef GPIO_InitStruct;
UART_HandleTypeDef huart1, huart2;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef timer2;

void init_core_clock(void);
void GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

volatile uint8_t rec[10];

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&timer2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		//the instance of timer2 is used to triger averaging maesured data
		//if(accelerationDataReadingIndicator == READING_ACCELERATION)
		{
			//accelerationDataReadingIndicator = AVERAGING_ACCELERATION;
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
	}
}
int main(void)
{
	init_core_clock();

	HAL_Init();
	GPIO_Init();

	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__USART1_CLK_ENABLE();
	__USART2_CLK_ENABLE();

	MX_USART2_UART_Init();
	MX_USART1_UART_Init();

	MX_I2C1_Init();
	MX_TIM2_Init();
	lcdInit ();

	lcdClear();

	initHMC5883L();
	waitTillMagnetometerIsInitialized();

	float degree = 0.0;
	char magnitudeReadString[12] = "";
	//HAL_UART_Receive_IT(&huart2, &rec, 1);
    while (1)
    {
    	if(1 == checkAvalibilityOfDataInRegister())
    	{

    		degree = calculateAzimutWithDegree();
    		sprintf(magnitudeReadString, "Az:%d \r\n", (int)degree);

    		//lcdClear();
    		//lcdSendString (magnitudeReadString);
    		if(HAL_UART_Transmit(&huart2, magnitudeReadString, strlen(magnitudeReadString), 120) != HAL_OK)
    		{
    		    HAL_Delay(5000);
    		}

    	}
        // Zmieñ stan diody LD2 na ON (wysoki)
        //HAL_GPIO_WritePin(LD2_GPIO_PORT, LD2_PIN, GPIO_PIN_SET);

        // OpóŸnienie
        HAL_Delay(10);

        // Zmieñ stan diody LD2 na OFF (niski)
        //HAL_GPIO_WritePin(LD2_GPIO_PORT, LD2_PIN, GPIO_PIN_RESET);

        // OpóŸnienie
        //HAL_Delay(300);

		//lcdClear();
		//lcdSetCursor(0, 0);
//		lcdClear();
//		lcdSetCursor(0, 0);
//		lcdSendString ("Z timerka");
        //HAL_UART_Receive_IT(&huart2, &rec, 1);
    }
}

void init_core_clock(void)
{
  	SystemCoreClock = 8000000;	// 8MHz

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	__PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void GPIO_Init(void)
{
    // W³¹cz zegar dla portu GPIOA
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Inicjalizuj pin diody LD2 jako wyjœcie
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LD2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_PORT, &GPIO_InitStruct);
}


void MX_USART1_UART_Init(void)
{
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }

	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
}

void MX_USART2_UART_Init(void)
{
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

//	__HAL_UART_ENABLE_IT(&huart2,  UART_IT_RXNE);
//	HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void MX_I2C1_Init(void)
{

	GPIO_InitTypeDef gpio_I2C1_SDA_SCL;
	gpio_I2C1_SDA_SCL.Pin = /*GPIO_PIN_4 |*/GPIO_PIN_8 | GPIO_PIN_9;
	gpio_I2C1_SDA_SCL.Mode = GPIO_MODE_AF_OD;
			// SCL, SDA
	gpio_I2C1_SDA_SCL.Pull = GPIO_NOPULL;
	gpio_I2C1_SDA_SCL.Alternate = GPIO_AF4_I2C1;
	gpio_I2C1_SDA_SCL.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &gpio_I2C1_SDA_SCL);

	__HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);

}

static void MX_TIM2_Init(void)
{
	__HAL_RCC_TIM2_CLK_ENABLE();

	const uint16_t durationBetweenSendingTwoMeasurementsInMs = 9000;

	timer2.Instance = TIM2;
	timer2.Init.Period = durationBetweenSendingTwoMeasurementsInMs - 1;
	timer2.Init.Prescaler = 8000 - 1;
	timer2.Init.ClockDivision = 0;
	timer2.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer2.Init.RepetitionCounter = 0;
	timer2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(&timer2);

	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_TIM_Base_Start_IT(&timer2);
}
