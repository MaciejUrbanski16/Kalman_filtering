#include "icm20948.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
const float g = 9.81;
const float rawGrawity = 16532.0;



void ICM20948_ReadData(uint8_t reg, uint8_t* data, uint16_t size)
{
    HAL_I2C_Master_Transmit(&hi2c1, ICM20948_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, ICM20948_ADDRESS, data, size, HAL_MAX_DELAY);
}

void initAccMagn()
{
//    uint8_t whoAmIReg = 0x00; // Adres rejestru akcelerometru
//    uint8_t whoAmIValue[1];
//
//    HAL_Delay(100);
//
//	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)ICM20948_ADDRESS << 1, 10, 200) != HAL_OK)
//	{
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		HAL_Delay(500);
//	}
//	setUserBank(ub_0);
//	uint8_t val = 0xc1;
//	HAL_Delay(200);
//    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_PWR_MGMT_1, 1, 0xc1, 1, 200) != HAL_OK)
//    {
//    	HAL_Delay(5000);
//    }
//    HAL_Delay(200);
//    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_PWR_MGMT_1, 1, 0x01, 1, 200) != HAL_OK)
//    {
//    	HAL_Delay(5000);
//    }
//
//    HAL_Delay(200);
//    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B0_PWR_MGMT_2, 1, 0x00, 1, 200) != HAL_OK)
//    {
//    	HAL_Delay(5000);
//    }
//    HAL_Delay(100);
//    setUserBank(ub_2);
//    HAL_Delay(200);
//    //output data rate start time alignment
//    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_ODR_ALIGN_EN, 1, 0x01, 1, 200) != HAL_OK)
//    {
//    	HAL_Delay(5000);
//    }
////    HAL_Delay(100);
////    //gyroscope configuration, gyroscope range set and enable digital filtering
////    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_GYRO_CONFIG, 1, 0x00, 1, 100) != HAL_OK)
////    {
////    	HAL_Delay(5000);
////    }
//
//    HAL_Delay(200);
//    //gyroscope configuration, sample rate divider = 0
//    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_GYRO_SMPLRT_DIV, 1, 0x00, 1, 100) != HAL_OK)
//    {
//    	HAL_Delay(5000);
//    }
//
//    HAL_Delay(200);
//    //gyroscope configuration, gyroscope range set and enable digital filtering
//    uint8_t dps_250_lpf = 0b00000001;
//    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_GYRO_CONFIG_1, 1, dps_250_lpf, 1, 100) != HAL_OK)
//    {
//    	HAL_Delay(5000);
//    }
//    gyroScale = 250.0f/gyroRawScaling * _d2r;
//
//    HAL_Delay(200);
//    //accelerometr configuration, sample rate divider = 0
//    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_ACCEL_SMPLRT_DIV_1, 1, 0x00, 1, 100) != HAL_OK)
//    {
//    	HAL_Delay(5000);
//    }
//    HAL_Delay(200);
//    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_ACCEL_SMPLRT_DIV_2, 1, 0x00, 1, 100) != HAL_OK)
//    {
//    	HAL_Delay(5000);
//    }
//
//    HAL_Delay(200);
//    uint8_t accel_config_2g_lpf = 0b00000001;
//    //accelerometr configuration, accelerometr range set and enable digital filtering
//    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, B2_ACCEL_CONFIG, 1, accel_config_2g_lpf, 1, 200) != HAL_OK)
//    {
//    	HAL_Delay(5000);
//    }
//    accelScale = G * 2.0f/accelRawScaling; // setting the accel scale to 2G
}

AccelData readAccData()
{
	AccelData accelData;
    uint8_t whoAmIReg = 0x00; // Adres rejestru akcelerometru
    uint8_t* whoAmIValue[1];
    //whoAmIValue[0] = 0;
    HAL_I2C_Master_Transmit(&hi2c1, ICM20948_ADDRESS << 1, &whoAmIReg, 1, HAL_MAX_DELAY);
    HAL_Delay(5);
    HAL_I2C_Master_Receive(&hi2c1, ICM20948_ADDRESS << 1, &whoAmIValue, 1, HAL_MAX_DELAY);
    //HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDRESS, 0x00, 1, &whoAmIValue, 1, 100);
    HAL_Delay(5);

    accelData.devAddr = *whoAmIValue[0];

    uint8_t raw_data[2];
    uint8_t xOut = B0_ACCEL_ZOUT_H;

    if(HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDRESS << 1, B0_ACCEL_XOUT_H, 1, raw_data, 2, 20) != HAL_OK)
    {
    	HAL_Delay(5000);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
    HAL_Delay(5);
    accelData.xAcc = (raw_data[0] << 8) | raw_data[1];
    //accelData.xAcc = ((float)rawXAcceleration * g)/(float)rawGrawity;

    if(HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDRESS << 1, B0_ACCEL_YOUT_H, 1, raw_data, 2, 20) != HAL_OK)
    {
    	HAL_Delay(5000);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    HAL_Delay(5);

    accelData.yAcc = (raw_data[0] << 8) | raw_data[1];
    //accelData.yAcc = ((float)rawYAcceleration * g)/(float)rawGrawity;

    if(HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDRESS << 1, B0_ACCEL_ZOUT_H, 1, raw_data, 2, 20) != HAL_OK)
    {
    	HAL_Delay(5000);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }

    HAL_Delay(5);
    char rawAccData[64];
    accelData.zAcc = (raw_data[0] << 8) | raw_data[1];
//    sprintf(rawAccData, "RAW x:%d y:%d z:%d \r\n", rawXAcceleration, rawYAcceleration, rawZAcceleration);
//	if(HAL_UART_Transmit(&huart2, rawAccData, strlen(rawAccData), 20) != HAL_OK)
//	{
//		//HAL_Delay(5000);
//	}
    //accelData.zAcc = ((float)(rawZAcceleration) * g)/(float)rawGrawity;


    return accelData;
}

GyroData readGyroData()
{
	GyroData gyroData;

    int8_t rawX_data[2];
    int8_t rawY_data[2];
    int8_t rawZ_data[2];

    uint8_t rawLowGyroX[1];
    uint8_t rawHighGyroX[1];

    if(HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDRESS << 1, B0_GYRO_XOUT_H, 1, rawX_data, 2, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }

    HAL_Delay(30);
    gyroData.xGyro = (rawX_data[0] << 8) + rawX_data[1];// | rawLowGyroX[0];

    if(HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDRESS << 1, B0_GYRO_YOUT_H, 1, rawY_data, 2, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    HAL_Delay(30);

    gyroData.yGyro = (rawY_data[0] << 8) + rawY_data[1];

    if(HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDRESS << 1, B0_GYRO_ZOUT_H, 1, rawZ_data, 2, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }


    gyroData.zGyro = (rawZ_data[0] << 8) + rawZ_data[1];

	return gyroData;
}

MagnData readMagnData()
{
    //initMagnetometr();

    //read data (8 registers for magn data starting from reg 0x11 (HXL))
    //ak9916_magn_read_reg(MAG_HXL, 8);

	setUserBank(ub_0);

	MagnData magnData;
	uint8_t rawData[6];
    if(HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDRESS << 1, B0_EXT_SLV_SENS_DATA_00, 1, rawData, 6, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
    HAL_Delay(50);

    magnData.xMagn = rawData[1] << 8 | rawData[0];
    magnData.yMagn = rawData[3] << 8 | rawData[2];
    magnData.zMagn = rawData[5] << 8 | rawData[4];

    return magnData;
}

void setUserBank(userbank ub)
{
    HAL_Delay(10);
    if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDRESS << 1, REG_BANK_SEL, 1, (uint8_t*)ub, 1, 100) != HAL_OK)
    {
    	HAL_Delay(5000);
    }
}
