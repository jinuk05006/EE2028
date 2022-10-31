//include all libraries we are using
#include "main.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"
#include "stdio.h"
#include "string.h"
//functions we are going to call
extern void initialise_monitor_handles(void);  // for semi-hosting support (printf)
static void MX_GPIO_Init(void);
static void UART1_Init(void);
static void GatheredData(void);
static void EXPLORER_MODE(void);
static void BATTLE_MODE(void);
static void WARNING_MODE(void);
static void single_press(void);
static void check_ths(float hum_data
		,float rms_mag_data
		,float rms_gyro_data
		,float temp_data
		,float bruh
		,float pressure_data);

void SystemClock_Config(void);
//global variables
#define exploring 0
#define warning 1
#define battle 2

int sec_counter;
int mode = 0;
int previous_mode;
int battery = 10;
int flag_pb = 0;
int flag_sp = 0;
int flag_dp = 0;
char message_print[32];
char big_message_print[128];

uint32_t t1,t2,texp,tbat;

UART_HandleTypeDef huart1;

//Our beloved callback function
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == BUTTON_EXTI13_Pin)
    {
      if(flag_pb == 1){
        flag_pb = 2;
        t2 = uwTick;
      }
      if(flag_pb == 0){
        flag_pb = 1;
        t1 = uwTick;
      }

      while(1){
        if(flag_pb == 2 && t2 -t1<= 1000){
          flag_dp = 1;
          flag_pb = 0;
          flag_sp = 0;
          break;
        }
       if (flag_pb == 2 && t2 - t1> 1000){
          flag_sp = 1;
          flag_pb = 0;
          flag_dp = 0;
          break;
        }
        break;
      }
    }



}


int main(void)
{
//initialise everything here
	initialise_monitor_handles(); // for semi-hosting support (printf)
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	MX_GPIO_Init();
	UART1_Init();
	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();

	//infinite loop using conditionals to funnel the guy through different modes
	while(1){
		if(mode == exploring && flag_pb == 0 && flag_sp == 0 && flag_dp == 0){
			EXPLORER_MODE();
		}
		if(mode == battle && flag_dp == 1 ){
			BATTLE_MODE();
		}
		if (mode == exploring && flag_dp == 1){
			EXPLORER_MODE();
		}
		if(mode == battle && flag_pb == 0 && flag_sp == 0 && flag_dp == 0){
			BATTLE_MODE();
		}

		if (mode == warning && flag_dp == 1){
			WARNING_MODE();
		}
	}
}

static void EXPLORER_MODE(void)
{
	//initialize all the stuff here
	mode = exploring;
	previous_mode = mode;
	flag_dp = 0;
	flag_sp = 0;
	sprintf(message_print,"I'm exploring\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	int exp_counter=0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,1);

	//while loop with two break conditions
	while(1){
		texp = uwTick;
		if(exp_counter%10==0){
			//this line gives the flag_dp ==1 and mode == warning
			GatheredData();
		}
		if(flag_dp == 1 && mode == warning){
			mode = warning;
			break;
		}
		//this line is to change mode with double press
		if(flag_dp == 1 && mode == exploring){
			mode = battle;
			break;
		}
		while(1){
			if((uwTick- texp)==100){
				exp_counter++;
				break;
			}
		}
	}
}

static void BATTLE_MODE(void)
{
	//Initializing stuff
	mode = battle;
	previous_mode = battle;
	flag_dp = 0;
	flag_sp = 0;
	sec_counter = 0;

	sprintf(message_print,"I am in battle\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

	//while not double pressed
	while(flag_dp == 0){
		single_press(); //check for single press

		if(sec_counter%100 == 0 && sec_counter != 0){
			GatheredData();
			}
		if(sec_counter%50 == 0){
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			}

		if(sec_counter%500 == 0){
			if(battery-2 >= 0){
				battery = battery - 2;
				sprintf(big_message_print,"bang bang bang!\r\nCurrent Battery level: %d/10\r\n",battery);
				HAL_UART_Transmit(&huart1, (uint8_t*)big_message_print, strlen(big_message_print),0xFFFF);
			}else{
				sprintf(message_print,"Out of battery!\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
		}
		if(flag_dp == 1 && mode == warning){
			mode = warning;
			break;
		}
		if(flag_dp ==1 && mode == battle){
			mode = exploring;
			break;
		}
		sec_counter++;
		tbat= uwTick;
		while(1){
			if(uwTick-tbat==10){
				break;
			}
		}
	}
}
static void WARNING_MODE(void)
{
	mode = previous_mode;
	flag_sp = 0;
	flag_dp = 0;
	sec_counter = 0;
	while(flag_sp == 0){
		mode = warning;
		single_press();
		uint32_t t1=uwTick;
		if(sec_counter%100==0){
			sprintf(message_print,"SOS!\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
		}
		if(sec_counter%16==0){
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		}
		while(1){
			if((uwTick-t1)==10){
				sec_counter++;
				break;
		  	  	  }
				}
		}
	mode = previous_mode;
	flag_sp = 0;
}

static void single_press(void){
	while(flag_pb == 1){
		if(uwTick -t1>1000){
        flag_pb = 0;
        flag_sp = 1;
        if(mode == battle){
          if(battery<10){
          battery++;
          sprintf(big_message_print,"CHARGING!\r\nCurrent Battery Level: %d/10\r\n",battery);
          HAL_UART_Transmit(&huart1, (uint8_t*)big_message_print, strlen(big_message_print),0xFFFF);
          }else{
            sprintf(message_print,"Battery is full!\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
          }
        }
      }
  }
}


static void GatheredData(void){
	float hum_data;
	hum_data = BSP_HSENSOR_ReadHumidity();

	float mag_data[3];
	float rms_mag_data;
	int16_t mag_data_i16[3] = { 0 };
	BSP_MAGNETO_GetXYZ(mag_data_i16);
	mag_data[0] = (float)mag_data_i16[0]/100.0f;
	mag_data[1] = (float)mag_data_i16[1]/100.0f;
	mag_data[2] = (float)mag_data_i16[2]/100.0f;
	rms_mag_data = (sqrt((mag_data[0]*mag_data[0])+(mag_data[1]*mag_data[1])+(mag_data[2]*mag_data[2])));

	float gyro_data[3];
	float rms_gyro_data;
	float gyro_data_i16[3] = { 0 };

	BSP_GYRO_GetXYZ(gyro_data_i16);
	gyro_data[0] = (float)gyro_data_i16[0]/100.0f;
	gyro_data[1] = (float)gyro_data_i16[1]/100.0f;
	gyro_data[2] = (float)gyro_data_i16[2]/100.0f;
	rms_gyro_data = (sqrt((gyro_data[0]*gyro_data[0])+(gyro_data[1]*gyro_data[1])+(gyro_data[2]*gyro_data[2])));

	float temp_data;
	temp_data = BSP_TSENSOR_ReadTemp();

	float accel_data[3];
	int16_t accel_data_i16[3] = { 0 };      // array to store the x, y and z readings.
	BSP_ACCELERO_AccGetXYZ(accel_data_i16);    // read accelerometer
		// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
	accel_data[0] = (float)accel_data_i16[0] / 100.0f;
	accel_data[1] = (float)accel_data_i16[1] / 100.0f;
	accel_data[2] = (float)accel_data_i16[2] / 100.0f;
	float bruh = accel_data[2];

	float pressure_data;
	pressure_data = BSP_PSENSOR_ReadPressure()/10.0f;

	check_ths(hum_data,rms_mag_data, rms_gyro_data, temp_data,bruh,pressure_data);

	//for battle mode
	if(mode == battle){
		sprintf(big_message_print,"T: %f deg cel, P:%f kPa, H:%f %%, A:%f M s^-2, G:%f dps, M:%f gauss\r\n", temp_data, pressure_data,hum_data,accel_data[2],rms_gyro_data,rms_mag_data);
		HAL_UART_Transmit(&huart1, (uint8_t*)big_message_print, strlen(big_message_print),0xFFFF);
	}
	//for exploration mode
	else if(mode == exploring){
		sprintf(big_message_print,"G:%f dps, M:%f gauss, P:%f kPa, H:%f %% \r\n",rms_gyro_data,rms_mag_data,pressure_data,hum_data);
		HAL_UART_Transmit(&huart1, (uint8_t*)big_message_print, strlen(big_message_print),0xFFFF);
	}

}

static void check_ths(float hum_data,float rms_mag_data,float rms_gyro_data,float temp_data,float bruh,float pressure_data){
	int flag = 0;

	if(hum_data<= 10.0f){
		flag++;
	}
	if(rms_mag_data>=35.0f){
		flag++;
	}
	if(rms_gyro_data>300.0f){
		flag++;
	}
	if(temp_data>=35.0f || temp_data<= 10.0f){
		flag++;
	}
	if(bruh<= 0.0f || bruh>= 20.0){//accelerometer
		flag++;
	}
	if(pressure_data>=110.0f || pressure_data<=70.0f){
		flag++;
	}
	if(flag>=2){
		mode = warning;
		flag_dp = 1;
	}
}

static void MX_GPIO_Init(void)

{
	__HAL_RCC_GPIOB_CLK_ENABLE(); //Enable AHB2 Bus for GPIOB
	__HAL_RCC_GPIOC_CLK_ENABLE(); //ENable AHB2 Bus for GPIOC

	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET); // Reset the LED2_Pin as 0

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Configuration of LED2_Pin (GPIO-B Pin-14) as GPIO output
	GPIO_InitStruct.Pin = LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



	// Configuration of BUTTON_EXTI13_Pin (G{IO-C Pin-13)as AF
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


	// Configuration of HTS221_DRDY_EXTI15 Pin
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


	// Configuration of LSM6DSL_INT1_EXTI11 Pin
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


	// Configuration of LSM6DSL_DRDY_EXTI11 pin
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Enable NVIC EXTI line 15 - 10
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);

	// Enable NVIC EXTI line 9 - 5
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn,0,1);


}

static void UART1_Init(void)
{
    /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


/* Configuring UART1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }

}
