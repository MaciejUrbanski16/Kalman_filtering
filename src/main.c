#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "lcd_display.h"
#include "magnetometr.h"
#include "icm20948.h"

#define LD2_PIN GPIO_PIN_5
#define LD2_GPIO_PORT GPIOA

GPIO_InitTypeDef GPIO_InitStruct;
UART_HandleTypeDef huart1, huart2, huart6;
I2C_HandleTypeDef hi2c1, hi2c3;
TIM_HandleTypeDef timer2, timer3, timer4;

float azs = 1.0f;
float accelScale = 0.0f;
const float accelRawScaling = 32767.5f;
const float G = 9.807f;

float gyroScale = 0.0f;
const float gyroRawScaling = 32767.5f; // =(2^16-1)/2 16 bit representation of gyro value to cover +/- range
const float _d2r = 3.14159265359f/180.0f; //degrees to radian/sec conversion

void init_core_clock(void);
void GPIO_Init(void);
void initMagnetometr(void);
void ak9916_magn_write_reg(uint8_t reg, uint8_t data);
void ak9916_magn_read_reg(uint8_t onset_reg, uint8_t len);

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART6_UART_Init(void);

static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

volatile uint8_t rec[10];

volatile uint8_t magnReadFlag = 0;
volatile uint8_t accReadFlag = 0;
volatile uint8_t gyroReadFlag = 0;

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&timer2);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&timer3);
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&timer4);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		magnReadFlag = 1;
		gyroReadFlag = 1;
	}
	else if(htim->Instance == TIM3)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		accReadFlag = 1;
	}
	else if(htim->Instance == TIM4)
	{
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//gyroReadFlag = 1;
	}
}
int main(void)
{
	init_core_clock();

	HAL_Init();
	GPIO_Init();

	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	//__USART1_CLK_ENABLE();
	__USART2_CLK_ENABLE();
	__USART6_CLK_ENABLE();

	MX_USART6_UART_Init(); // !! this impacts husart2
	MX_USART2_UART_Init();
	//MX_USART1_UART_Init();


	MX_I2C1_Init();
	MX_I2C3_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	//MX_TIM4_Init();

	float degree = 0.0;
	char accReadString[64] = "";
	char gyroReadString[64] = "";
	char magnReadString[32] = "";

	HAL_Delay(100);
	//initAccGyro();

    uint8_t whoAmIReg = 0x00; // Adres rejestru akcelerometru
    uint8_t whoAmIValue[1];

    HAL_Delay(100);

	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)ICM20948_ADDRESS << 1, 10, 200) != HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(500);
	}
	setUserBank(ub_0);
	uint8_t val = 0xc1;
	HAL_Delay(200);
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_PWR_MGMT_1, 1, 0xc1, 1, 200) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(200);
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_PWR_MGMT_1, 1, 0x01, 1, 200) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    HAL_Delay(200);
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_PWR_MGMT_2, 1, 0x00, 1, 200) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(100);
    setUserBank(ub_2);
    HAL_Delay(200);
    //output data rate start time alignment
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_ODR_ALIGN_EN, 1, 0x01, 1, 200) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
//    HAL_Delay(100);
//    //gyroscope configuration, gyroscope range set and enable digital filtering
//    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_GYRO_CONFIG, 1, 0x00, 1, 100) != HAL_OK)
//    {
//    	HAL_Delay(5000);
//    }

    HAL_Delay(200);
    //gyroscope configuration, sample rate divider = 0
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_GYRO_SMPLRT_DIV, 1, 0x00, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    HAL_Delay(200);
    //gyroscope configuration, gyroscope range set and enable digital filtering
    uint8_t dps_250_lpf = 0b00000001;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_GYRO_CONFIG_1, 1, dps_250_lpf, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    gyroScale = 250.0f/gyroRawScaling * _d2r;

    HAL_Delay(200);
    //accelerometr configuration, sample rate divider = 0
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_ACCEL_SMPLRT_DIV_1, 1, 0x00, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(200);
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_ACCEL_SMPLRT_DIV_2, 1, 0x00, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    HAL_Delay(200);
    uint8_t accel_config_2g_lpf = 0b00000001;
    //accelerometr configuration, accelerometr range set and enable digital filtering
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_ACCEL_CONFIG, 1, accel_config_2g_lpf, 1, 200) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    accelScale = G * 2.0f/accelRawScaling; // setting the accel scale to 2G

    HAL_Delay(300);
    initMagnetometr();

    initHMC5883L();

    //read data (8 registers for magn data starting from reg 0x11 (HXL))
    HAL_Delay(200);
    //ak9916_magn_read_reg(MAG_HXL, 8);

    setUserBank(ub_0);

    int16_t rawXAcc = 0;
    int16_t rawYAcc = 0;
    int16_t rawZAcc = 0;

    int16_t rawXGyro = 0;
    int16_t rawYGyro = 0;
    int16_t rawZGyro = 0;

    int16_t degreeMagn = 0;

    while (1)
    {
    	if(accReadFlag == 1)
    	{


        	HAL_Delay(5);
        	AccelData accelData = readAccData();
        	HAL_Delay(5);
        	GyroData gyroData = readGyroData();

			const int16_t xAccDecimal = (int16_t)(accelData.xAcc * 10000);
			const int16_t yAccDecimal = (int16_t)(accelData.yAcc * 10000);
			const int16_t zAccDecimal = (int16_t)(accelData.zAcc * 10000);
			float  _az = (((float)accelData.zAcc * accelScale))*azs;
	//    	sprintf(accReadString, "ACCELEROMETR whoAmI: %u -> xAcc:%d.%d yAcc:%d.%d, zAcc:%d.%d\r\n",
	//    			accelData.devAddr, (int16_t)accelData.xAcc, xAccDecimal,  (int16_t)accelData.yAcc, yAccDecimal, (int16_t)accelData.zAcc, zAccDecimal);
			sprintf(accReadString, "ACCELEROMETR whoAmI: %u -> zAcc:%d.%d rawZ:%d\r\n",
					accelData.devAddr, (int16_t)_az, zAccDecimal, accelData.zAcc);
//	    	if(HAL_UART_Transmit(&huart2, accReadString, strlen(accReadString), 20) != HAL_OK)
//	    	{
//	    		HAL_Delay(5000);
//	    	}


	    	float _gx = ((float)gyroData.xGyro * gyroScale);
	    	const int16_t xGyroDecimal = (int16_t)(_gx * 10000);
	    	float _gy = ((float)gyroData.yGyro * gyroScale);
	    	const int16_t yGyroDecimal = (int16_t)(_gy * 10000);
	    	float _gz = ((float)gyroData.zGyro * gyroScale);
	    	const int16_t zGyroDecimal = (int16_t)(_gz * 10000);

	    	//degree = calculateAzimutWithDegree();

	    	sprintf(gyroReadString, "ACC %d %d %d GYR %d %d %d MAGN %d\r\n",
	    			accelData.xAcc, accelData.yAcc, accelData.zAcc, gyroData.xGyro, gyroData.yGyro, gyroData.zGyro, (int16_t)(degree));
//	    	if(HAL_UART_Transmit(&huart6, gyroReadString, strlen(gyroReadString), 120) != HAL_OK)
//	    	{
//	    	  	HAL_Delay(5000);
//	    	}

	    	if(HAL_UART_Transmit(&huart2, gyroReadString, strlen(gyroReadString), 120) != HAL_OK)
	    	{
	    	  	HAL_Delay(5000);
	    	}
	    	accReadFlag = 0;
    	}
    	if(magnReadFlag == 1)
    	{
			//HAL_UART_Receive_IT(&huart2, &rec, 1);
			magnReadFlag = 0;
    	}
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

void ak9916_magn_write_reg(uint8_t reg, uint8_t data)
{
	setUserBank(ub_3);
	uint8_t tempData;
	tempData = AK09916_ADDRESS;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_ADDR, 1, AK09916_ADDRESS, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_REG, 1, reg, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_DO, 1, data, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    //enable single data write to the register
    tempData = 0x80|0x01;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_CTRL, 1, 0x80|0x01, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);
}

void ak9916_magn_read_reg(uint8_t onset_reg, uint8_t len)
{
	setUserBank(ub_3);
	uint8_t tempData;
	tempData = 0x80|AK09916_ADDRESS;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_ADDR, 1, (0x80|AK09916_ADDRESS) << 1, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_REG, 1, onset_reg, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    //enable single data write to the output register(s)
    tempData = 0x80|len;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_SLV0_CTRL, 1, 0x80|len, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);
    setUserBank(ub_0);
}

void initMagnetometr(void)
{
	setUserBank(ub_0);
	uint8_t tempData;
	HAL_Delay(50);
    if(HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDRESS << 1, B0_USER_CTRL, 1, &tempData, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    //reset I2C master module
	HAL_Delay(50);
	tempData |= 0x02;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_USER_CTRL, 1, tempData, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);
    //enable I2C master module
    tempData |= 0x20;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_USER_CTRL, 1, tempData, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }

    //I2C master clock: 7 (400kHz)
    HAL_Delay(50);
    setUserBank(ub_3);

    tempData = 0x07;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_MST_CTRL, 1, tempData, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);
    //LP CONFIG
    setUserBank(ub_0);
    tempData = 0x40;
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_LP_CONFIG, 1, tempData, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);

    setUserBank(ub_3);
    tempData = 0x03;
    //I2C_MST_ODR_CONFIG: 1.1kHz/(2^3) = 136Hz
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B3_I2C_MST_ODR_CONFIG, 1, tempData, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
    HAL_Delay(50);
    setUserBank(ub_0);

    //magnetometr reset
    ak9916_magn_write_reg(MAG_CNTL3, 0x01);
    HAL_Delay(100);
    //continouos mode 4: 100Hz
    ak9916_magn_write_reg(MAG_CNTL2, 0x08);
    setUserBank(ub_0);

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

	__HAL_UART_ENABLE_IT(&huart2,  UART_IT_RXNE);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void MX_USART6_UART_Init(void)
{
	//USART6_TX
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    //USART6_RX
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    huart6.Instance = USART6;
    huart6.Init.BaudRate = 9600;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart6);

	//__HAL_UART_ENABLE_IT(&huart6,  UART_IT_RXNE);
	//HAL_NVIC_EnableIRQ(USART6_IRQn);
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

static void MX_I2C3_Init(void)
{

	GPIO_InitTypeDef gpio_I2C3_SDA;
	gpio_I2C3_SDA.Pin = /*GPIO_PIN_4 |*/GPIO_PIN_4;
	gpio_I2C3_SDA.Mode = GPIO_MODE_AF_OD;
			//SDA
	gpio_I2C3_SDA.Pull = GPIO_NOPULL;
	gpio_I2C3_SDA.Alternate = GPIO_AF4_I2C3;
	gpio_I2C3_SDA.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &gpio_I2C3_SDA);

	GPIO_InitTypeDef gpio_I2C3_SCL;
	gpio_I2C3_SCL.Pin = /*GPIO_PIN_4 |*/GPIO_PIN_8;
	gpio_I2C3_SCL.Mode = GPIO_MODE_AF_OD;
			// SCL
	gpio_I2C3_SCL.Pull = GPIO_NOPULL;
	gpio_I2C3_SCL.Alternate = GPIO_AF4_I2C3;
	gpio_I2C3_SCL.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio_I2C3_SCL);

	__HAL_RCC_I2C3_CLK_ENABLE();

    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c3);

}


static void MX_TIM2_Init(void)
{
	//magnetometr timer
	__HAL_RCC_TIM2_CLK_ENABLE();

	const uint16_t durationBetweenSendingTwoMeasurementsInMs = 2000;

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

static void MX_TIM3_Init(void)
{
	//accelerometr timer
	__HAL_RCC_TIM3_CLK_ENABLE();

	const uint16_t durationBetweenSendingTwoMeasurementsInMs = 1200;

	timer3.Instance = TIM3;
	timer3.Init.Period = durationBetweenSendingTwoMeasurementsInMs - 1;
	timer3.Init.Prescaler = 8000 - 1;
	timer3.Init.ClockDivision = 0;
	timer3.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer3.Init.RepetitionCounter = 0;
	timer3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(&timer3);

	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_TIM_Base_Start_IT(&timer3);
}

static void MX_TIM4_Init(void)
{
	//gyroscope timer
	__HAL_RCC_TIM4_CLK_ENABLE();

	const uint16_t durationBetweenSendingTwoMeasurementsInMs = 24000;

	timer4.Instance = TIM4;
	timer4.Init.Period = durationBetweenSendingTwoMeasurementsInMs - 1;
	timer4.Init.Prescaler = 8000 - 1;
	timer4.Init.ClockDivision = 0;
	timer4.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer4.Init.RepetitionCounter = 0;
	timer4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(&timer4);

	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	HAL_TIM_Base_Start_IT(&timer4);
}
