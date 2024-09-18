/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc_ex.h"
#include <stdio.h>

#define KP 0.5f
#define KI 1.0f
#define KD 0.5f

#define dt 10 //ms, compute speed  period

//sum error max value
#define SUM_ERR_MAX 10000
//pwm max value
#define PWM_MAX 10000
#define PKG_HEADh 0xFF
#define PKG_HEADl 0x55//head 0xFF55
#define PKG_TAIL 0xC3
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//PID struct
typedef struct _PID
{
  int   SetPoint;
  int   Actual;
  int   Err;
  int   LastErr;
  int   SumErr;
	int   Output;
}PID;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//actual speed
short speedL=0;
short speedR=0;
//target speed
short speedL_set=0;
short speedR_set=0;
uint16_t speedL_abs=0;
uint16_t speedR_abs=0;


PID pidL;
PID pidR;
uint8_t sw=0;
uint16_t gray[4]={0};
uint8_t RxBuffer[15];  // 用于接收数据的缓冲区

short encoder_countL=0;
short encoder_countR=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//compute PID output
int PID_Compute(PID *pid)
{
  pid->Err = pid->SetPoint - pid->Actual;
  pid->SumErr += pid->Err;
  //sum error limiting
  if(pid->SumErr > SUM_ERR_MAX)
  {
    pid->SumErr = SUM_ERR_MAX;
  }
  else if(pid->SumErr < -SUM_ERR_MAX)
  {
    pid->SumErr = -SUM_ERR_MAX;
  }
  pid->Output = (0.375 * pid->Err + 0.0075 * pid->SumErr + 0.1* (pid->Err - pid->LastErr) );
  //amplitude limiting
  if(pid->Output > PWM_MAX)
  {
    pid->Output = PWM_MAX;
  }
  else if(pid->Output < -PWM_MAX)
  {
    pid->Output = -PWM_MAX;
  }
  pid->LastErr = pid->Err;
  return pid->Output;
}

// control speed and dir of L and R motor
void Motor_Open_Control(void)
{
	//speed abs value
  speedL_abs = abs(speedL_set);
  speedR_abs = abs(speedR_set);
  //set pwm
  htim1.Instance->CCR1 = speedL_abs;
  htim1.Instance->CCR2 = speedR_abs;
	
	//dir control
  if(speedL_set < 0)
  {
    // 0 1
    HAL_GPIO_WritePin(GPIOB, motor1_dir1_Pin, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOB, motor1_dir2_Pin, GPIO_PIN_RESET);
  }
  else if(speedL_set > 0)
  {
    //1 0 
    HAL_GPIO_WritePin(GPIOB, motor1_dir1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, motor1_dir2_Pin, GPIO_PIN_SET);
  }
	else
  { 
    //0 0
    HAL_GPIO_WritePin(GPIOB, motor1_dir1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, motor1_dir2_Pin, GPIO_PIN_RESET);
  }
	
	
  if(speedR_set < 0)
  {
    // 0 1
    HAL_GPIO_WritePin(GPIOB, motor2_dir1_Pin, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOB, motor2_dir2_Pin, GPIO_PIN_SET);
  }
  else if(speedR_set > 0)
  {
    //1 0 
    HAL_GPIO_WritePin(GPIOB, motor2_dir1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, motor2_dir2_Pin, GPIO_PIN_RESET);
  }
	else
  { 
    //0 0
    HAL_GPIO_WritePin(GPIOB, motor2_dir1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, motor2_dir2_Pin, GPIO_PIN_RESET);
  }

}
int Motor_Control(void)
{
		
  //HAL_Delay(10);

  //speed control
  //compute PID output
  pidL.SetPoint = speedL_set;
  pidL.Actual = speedL;
  pidR.SetPoint = speedR_set;
  pidR.Actual = speedR;
  pidL.Output = PID_Compute(&pidL);
  pidR.Output = PID_Compute(&pidR);

  //speed abs value
  speedL_abs = abs(pidL.Output);
  speedR_abs = abs(pidR.Output);
  //set pwm
  htim1.Instance->CCR1 = speedL_abs;
  htim1.Instance->CCR2 = speedR_abs;

  //dir control
	if(speedL_set==0 ){
			HAL_GPIO_WritePin(GPIOB, motor1_dir1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, motor1_dir2_Pin, GPIO_PIN_RESET);
			pidL.SumErr = 0;
		}
	else{
  if(pidL.Output < 0)
  {
    // 0 1
    HAL_GPIO_WritePin(GPIOB, motor1_dir1_Pin, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOB, motor1_dir2_Pin, GPIO_PIN_RESET);
  }
  else if(pidL.Output > 0)
  {
    //1 0 
    HAL_GPIO_WritePin(GPIOB, motor1_dir1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, motor1_dir2_Pin, GPIO_PIN_SET);
  }
	else
  { 
    //0 0
    HAL_GPIO_WritePin(GPIOB, motor1_dir1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, motor1_dir2_Pin, GPIO_PIN_RESET);
  }
}
	if(speedR_set==0)
	{
			HAL_GPIO_WritePin(GPIOB, motor2_dir1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, motor2_dir2_Pin, GPIO_PIN_RESET);
			pidR.SumErr = 0;
	}
	else{
  if(pidR.Output < 0)
  {
    // 0 1
    HAL_GPIO_WritePin(GPIOB, motor2_dir1_Pin, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOB, motor2_dir2_Pin, GPIO_PIN_SET);
  }
  else if(pidR.Output > 0)
  {
    //1 0 
    HAL_GPIO_WritePin(GPIOB, motor2_dir1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, motor2_dir2_Pin, GPIO_PIN_RESET);
  }
	else
  { 
    //0 0
    HAL_GPIO_WritePin(GPIOB, motor2_dir1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, motor2_dir2_Pin, GPIO_PIN_RESET);
	}  
	}
	return 0;
}

//send pkg=[PKG_HEADh,PKG_HEADl,sw,gray[0],gray[1],gray[2],gray[3],PKG_TAIL] 8bit*8 trough UART3
uint8_t tx_pkg(void)
{
	uint8_t pkg[8]={PKG_HEADh,PKG_HEADl,sw,gray[0],gray[1],gray[2],gray[3],PKG_TAIL};
	HAL_UART_Transmit(&huart3, pkg, 8,10);
	return 0;
}

//send motor speed and tar speed data to uart5
//PKG_HEADh,PKG_HEADl,speedL,speedR,speedL_set,speedR_set,PKG_TAIL
void tx_speed(void)
{
  uint8_t pkg[7]={PKG_HEADh,PKG_HEADl,speedL,speedR,speedL_set,speedR_set,PKG_TAIL};
  HAL_UART_Transmit(&huart5, pkg, 7,10);
}

//print all sensor data and motor speed data to uart5
void print_data(void)
{
	printf("sw:%d\n" ,sw);
  printf("start_sw1:%d  start_sw2:%d\n",sw & 0x10 ? 1 : 0,sw & 0x20 ? 1 : 0);
  for(int i=0;i<4;i++)
  {
    printf("ir_sw[%d]:%d  ",i+1,sw & (1<<i) ? 1 : 0);
  }
	printf("\n");
  for(int i=0;i<4;i++)
  {
    printf("gray[%d]:%d  ",i,gray[i]);
  }
	
  printf("\nspeedL:%d  speedR:%d  speedL_set:%d  speedR_set:%d\n",speedL,speedR,speedL_set,speedR_set);
}

//update sensor data here: gray[4](uint16 adc value) and sw(8bit switch value bit[0:3]:ir_sw1~4, bit[4:5]:start_sw1~2, bit[6:7]:0)
void update_sensor(void)
{
  //gray sensor data has been updated in DMA
  //update switch data
  sw =  HAL_GPIO_ReadPin(GPIOA, ir_sw1_Pin)      | HAL_GPIO_ReadPin(GPIOA, ir_sw2_Pin) << 1    | HAL_GPIO_ReadPin(GPIOA, ir_sw3_Pin) << 2 |
        HAL_GPIO_ReadPin(GPIOA, ir_sw4_Pin) << 3 | HAL_GPIO_ReadPin(GPIOB, start_sw1_Pin) << 4 | HAL_GPIO_ReadPin(GPIOB, start_sw2_Pin) << 5;
}

//recieve speed data here
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {

    //get latest pkg from RxBuffer
    //revPkg= PKG_HEADh(8bit), PKG_HEADl(8bit), speedL_set(16bit short), speedR_set(16bit short), PKG_TAIL(8bit),length=7*8=40bit
    for(int i=0;i<7;i++)
    {
      if(RxBuffer[i] == PKG_HEADh && RxBuffer[i+1] == PKG_HEADl&& RxBuffer[i+6] == PKG_TAIL)
      {
        //get speedL_set
        speedL_set = -((RxBuffer[i+2] << 8) | RxBuffer[i+3]);
        //get speedR_set
        speedR_set = (RxBuffer[i+4] << 8) | RxBuffer[i+5];
				Motor_Control();
				update_sensor();
				tx_pkg();
        break;
      }
    }
    // restart receive
    HAL_UART_Receive_IT(&huart3, RxBuffer, 7);
  }
}

//update actual speed here
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM9)
    {
				encoder_countL =(short)( __HAL_TIM_GET_COUNTER(&htim4));
        encoder_countR =(short)( __HAL_TIM_GET_COUNTER(&htim8));
				//compute speed, round/min , reduction ratio:60 , 16pulse/round
        speedL = encoder_countL ;
        speedR = encoder_countR ;
				__HAL_TIM_SET_COUNTER(&htim4, 0);
        __HAL_TIM_SET_COUNTER(&htim8, 0);
//				printf("speedL:%d \n",speedL);
//        printf("speedR:%d \n",speedR);
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart3, RxBuffer, 7);
	HAL_TIM_Base_Start_IT(&htim9);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOB, motor1_dir1_Pin, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOB, motor1_dir2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, motor2_dir1_Pin, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOB, motor2_dir2_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)gray,4);
  //uint8_t msg[]="114514";
	//	uint8_t tx_msg[]="i am stm32f4\n";
	//	uint8_t rx_msg[64];
	while (1)
	 {
		 /*
			speedL_set=30;
		 speedR_set = 30;
			Motor_Control();
		 HAL_Delay(5000);
		 speedL_set=0;
		 speedR_set = 0;
		 Motor_Control();
		 HAL_Delay(5000);
		 speedL_set=-30;
		 speedR_set = -30;
			Motor_Control();
		 HAL_Delay(5000);
		 */
		 
		 HAL_Delay(50);
			
		  //print_data();
		 //HAL_UART_Transmit(&huart3,msg,sizeof(msg),100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1,0xffff);
    return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
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
