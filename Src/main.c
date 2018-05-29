/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/


/***Note: CAN in stm32 has three transmit mailboxes and two receive FIFOs***/

 /*Handlers for transmission*/

CAN_HandleTypeDef hcan;
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;



CAN_HandleTypeDef hcanR;  /*Handler for receiving*/



TIM_HandleTypeDef htim3;		/*Handler for timer*/


CAN_FilterConfTypeDef sFilterConfig;         /*Handler for filter (which messages to receive))*/


#define GPIOA_ODR		*(volatile uint32_t*) ((0x4001080C))      /*PortA output data register*/
#define CAN_RDL0R		*(volatile uint32_t*) ((0x400065B8))		/*CAN receive FIFO0 mailbox data low register */
#define CAN_RDH0R		*(volatile uint32_t*) ((0x400065BC))		/*CAN receive FIFO0 mailbox data high register */
#define CAN_MCR          *(volatile uint32_t*) ((0x40016402))		/*CAN master control register*/
#define CAN_IER 		*(volatile uint32_t*) ((0x40006414))		/*CAN interrupt enable register */

CanTxMsgTypeDef        TxMessage;
CanRxMsgTypeDef		   RxMsg;

CanTxMsgTypeDef        TxMessage1;


CanTxMsgTypeDef        TxMessage2;

#define get_bit(reg,bit_no)   ((reg)&(1<<(bit_no)))>>(bit_no)          /*Macro for get bit*/


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


typedef float  float32;
typedef unsigned short  uint16;
typedef unsigned char  uint8;



uint8 flag=0;
uint8 flagR=0;
uint8 flag2=0;





#define byte_01_2				0
#define byte_01_3             	1
#define byte_01_4               2
#define byte_01_5               3
#define byte_01_6				4


#define byte_02_0				5
#define byte_02_1               6
#define byte_02_2				7
#define byte_02_3             	8
#define byte_02_4               9
#define byte_02_5              10
#define byte_02_6				0
#define byte_02_7				11


#define byte_05_0					 0x00
#define byte_05_1             		 0x7D
#define byte_05_2					 0x08
#define byte_05_3                	 0x05
#define byte_05_4                    0x00
#define byte_05_5                    0x00
#define byte_05_6				     0x00
#define byte_05_7				     0x00




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint8 a;
	uint8 i;
	uint8 Received_Data[8];
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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

  MX_TIM3_Init();
  MX_CAN_Init();


  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);

 HAL_CAN_Receive_IT(&hcanR,CAN_FIFO1);        /*CAN receive interrupt function*/


  while (1)
  {


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */



	  	  	  	  	  	 if(flag==1)
	  			        {

	  			        			HAL_CAN_Transmit(&hcan,20);
	  			        			HAL_CAN_Transmit(&hcan1,20);
	  			        			flag=0;

	  			        }

	  			        if(flag2==1)
	  			        {
	  			        	HAL_CAN_Transmit(&hcan2,20);
	  			        	flag2=0;
	  			        }
	  			        if (flagR==1)
	  			        {
	  			        	for(i=0;i<8;i++)
	  			        	{
	  			        		Received_Data[i]=hcanR.pRx1Msg->Data[i];
	  			        	}
	  			        	a=Received_Data[0];
	  			        		flagR=0;
	  			        }

	  			        	/*switch case on the camera status to map to four LEDs*/
	  			        switch(a)
	  			        {
	  			        case 0:
	  			        	GPIOA_ODR=0;
	  			        	break;
	  			        case 1:
	  			        	GPIOA_ODR=1;
	  			        	break;
	  			        case 2:
	  			        	GPIOA_ODR=2;
	  			        	break;
	  			        case 3:
	  			        	GPIOA_ODR=3;
	  			        	break;
	  			        case 4:
	  			        	GPIOA_ODR=4;
	  			        	break;
	  			        case 5:
	  			        	GPIOA_ODR=5;
	  			        	break;
	  			        case 6:
	  			        	GPIOA_ODR=6;
	  			        	break;
	  			        case 7:
	  			        	GPIOA_ODR=7;
	  			        	break;
	  			        case 8:
	  			        	GPIOA_ODR=8;
	  			        	break;
	  			        case 9:
	  			        	GPIOA_ODR=9;
	  			        	break;
	  			        case 10:
	  			        	GPIOA_ODR=10;
	  			        	break;
	  			        case 11:
	  			        	GPIOA_ODR=11;
	  			        	break;
	  			        case 12:
	  			        	GPIOA_ODR=12;
	  			        	break;
	  			        case 13:
	  			        	GPIOA_ODR=13;
	  			        	break;
	  			        case 14:
	  			        	GPIOA_ODR=14;
	  			        	break;
	  			        case 15:
	  			        	GPIOA_ODR=15;
	  			        	break;
	  			        default:
	  			        	/*Misra*/
	  			        	break;
	  			        }


  }


  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{





	hcan.pTxMsg = &TxMessage;


	hcan1.pTxMsg = &TxMessage1;


		hcan2.pTxMsg = &TxMessage2;

		/*hcan handler configuration*/

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;	/*CAN working in normal mode*/
  hcan.Init.SJW = CAN_BS1_1TQ;      /*CAN_synchronisation_jump_width*/
  hcan.Init.BS1 = CAN_BS1_12TQ;		/*CAN_time_quantum_in_bit_segment_1*/
  hcan.Init.BS2 = CAN_BS2_3TQ;		/*CAN_time_quantum_in_bit_segment_2*/
  hcan.Init.TTCM = DISABLE;			/*Disable the time triggered communication mode*/
  hcan.Init.ABOM = DISABLE;			/*Disable the automatic bus-off management*/
  hcan.Init.AWUM = DISABLE;			/*Disable the automatic wake-up mode*/
  hcan.Init.NART = DISABLE;			/*Disable the non-automatic retransmission mode*/
  hcan.Init.RFLM = DISABLE;			/*Disable the receive FIFO Locked mode( Once a receive FIFO is full the next incoming
										message will overwrite the previous one.)*/
  hcan.Init.TXFP = DISABLE;			/* Disable the transmit FIFO priority*/

  /*hcan1 handler configuration*/

  hcan1.Instance = CAN1;
   hcan1.Init.Prescaler = 3;
   hcan1.Init.Mode = CAN_MODE_NORMAL;    /*CAN working in normal mode*/
   hcan1.Init.SJW = CAN_BS1_1TQ;         /*CAN_synchronisation_jump_width*/
   hcan1.Init.BS1 = CAN_BS1_12TQ;        /*CAN_time_quantum_in_bit_segment_1*/
   hcan1.Init.BS2 = CAN_BS2_3TQ;         /*CAN_time_quantum_in_bit_segment_2*/
   hcan1.Init.TTCM = DISABLE;            /*Disable the time triggered communication mode*/
   hcan1.Init.ABOM = DISABLE;            /*Disable the automatic bus-off management*/
   hcan1.Init.AWUM = DISABLE;            /*Disable the automatic wake-up mode*/
   hcan1.Init.NART = DISABLE;            /*Disable the non-automatic retransmission mode*/
   hcan1.Init.RFLM = DISABLE;            /*Disable the receive FIFO Locked mode( Once a receive FIFO is full the next incoming message will overwrite the previous one.)*/
   hcan1.Init.TXFP = DISABLE;             /* Disable the transmit FIFO priority*/

   /*hcan2 handler*/

   hcan2.Instance = CAN1;
    hcan2.Init.Prescaler = 3;
    hcan2.Init.Mode = CAN_MODE_NORMAL;   /*CAN working in normal mode*/
    hcan2.Init.SJW = CAN_BS1_1TQ;        /*CAN_synchronisation_jump_width*/
    hcan2.Init.BS1 = CAN_BS1_12TQ;       /*CAN_time_quantum_in_bit_segment_1*/
    hcan2.Init.BS2 = CAN_BS2_3TQ;        /*CAN_time_quantum_in_bit_segment_2*/
    hcan2.Init.TTCM = DISABLE;           /*Disable the time triggered communication mode*/
    hcan2.Init.ABOM = DISABLE;           /*Disable the automatic bus-off management*/
    hcan2.Init.AWUM = DISABLE;           /*Disable the automatic wake-up mode*/
    hcan2.Init.NART = DISABLE;           /*Disable the non-automatic retransmission mode*/
    hcan2.Init.RFLM = DISABLE;           /*Disable the receive FIFO Locked mode( Once a receive FIFO is full the next incoming message will overwrite the previous one.)*/
    hcan2.Init.TXFP = DISABLE;            /* Disable the transmit FIFO priority*/


    /*Message transmit 1*/

  hcan.pTxMsg->StdId=0x10;			/*standard ID*/
  hcan.pTxMsg->IDE=CAN_ID_STD;		/*ID is '0' which means it runs in standard format*/
  hcan.pTxMsg->RTR=CAN_RTR_DATA;	/*'RTR' is '0' which means data frame*/
  hcan.pTxMsg->DLC=8;				/*'8' byte data'*/
  hcan.pTxMsg->Data[2]=byte_01_2;
  hcan.pTxMsg->Data[3]=byte_01_3;
  hcan.pTxMsg->Data[4]=byte_01_4;
  hcan.pTxMsg->Data[5]=byte_01_5;
  hcan.pTxMsg->Data[6]=byte_01_6;
  hcan.pTxMsg->Data[7]=0;


  /*Message transmit 2*/


	hcan1.pTxMsg->StdId=0x11;                 /*standard ID*/
	hcan1.pTxMsg->IDE=CAN_ID_STD;              /*ID is '0' which means it runs in standard format*/
	hcan1.pTxMsg->RTR=CAN_RTR_DATA;            /*'RTR' is '0' which means data frame*/
	hcan1.pTxMsg->DLC=8;                       /*'8' byte data'*/
    hcan1.pTxMsg->Data[0]=byte_02_0;
    hcan1.pTxMsg->Data[1]=byte_02_1;
    hcan1.pTxMsg->Data[2]=byte_02_2;
    hcan1.pTxMsg->Data[3]=byte_02_3;
    hcan1.pTxMsg->Data[4]=byte_02_4;
    hcan1.pTxMsg->Data[5]=byte_02_5;
    hcan1.pTxMsg->Data[6]=byte_02_6;
    hcan1.pTxMsg->Data[7]=byte_02_7;


    /*Message transmit 3*/

      hcan2.pTxMsg->StdId=0x5;             /*standard ID*/
      hcan2.pTxMsg->IDE=CAN_ID_STD;          /*ID is '0' which means it runs in standard format*/
      hcan2.pTxMsg->RTR=CAN_RTR_DATA;        /*'RTR' is '0' which means data frame*/
      hcan2.pTxMsg->DLC=8;                   /*'8' byte data'*/
      hcan2.pTxMsg->Data[0]=byte_05_0;
      hcan2.pTxMsg->Data[1]=byte_05_1;
      hcan2.pTxMsg->Data[2]=byte_05_2;
      hcan2.pTxMsg->Data[3]=byte_05_3;
      hcan2.pTxMsg->Data[4]=byte_05_4;
      hcan2.pTxMsg->Data[5]=byte_05_5;
      hcan2.pTxMsg->Data[6]=byte_05_6;
      hcan2.pTxMsg->Data[7]=byte_05_7;



      HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);   /*Setting the priority of CAN receive to '0' and sub priority to '0'*/


  /*if HAL_CAN_Init returns NOT_OK then it is error*/
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }



      hcanR.pRx1Msg=&RxMsg;

      /*configure the filter of CAN message receiving*/

	  sFilterConfig.FilterNumber = 0;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterIdHigh = 0xffff;
	  sFilterConfig.FilterIdLow = 0xffff;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = 1;
	  sFilterConfig.FilterActivation = 1;
	  sFilterConfig.BankNumber = 14;

	  /*Message to be received*/

	  hcanR.pRx1Msg->StdId=0x50;             /*standard ID*/
	  hcanR.pRx1Msg->IDE=CAN_ID_STD;          /*ID is '0' which means it runs in standard format*/
	  hcanR.pRx1Msg->RTR=CAN_RTR_DATA;        /*'RTR' is '0' which means data frame*/
	  hcanR.pRx1Msg->DLC=8;                   /*'8' byte data'*/

	  hcanR.Instance = CAN1;
	  hcanR.Init.Prescaler = 3;
	  hcanR.Init.Mode = CAN_MODE_NORMAL;    /*CAN working in normal mode*/
	  hcanR.Init.SJW = CAN_BS1_1TQ;         /*CAN_synchronisation_jump_width*/
	  hcanR.Init.BS1 = CAN_BS1_12TQ;        /*CAN_time_quantum_in_bit_segment_1*/
	  hcanR.Init.BS2 = CAN_BS2_3TQ;         /*CAN_time_quantum_in_bit_segment_2*/
	  hcanR.Init.TTCM = DISABLE;            /*Disable the time triggered communication mode*/
	  hcanR.Init.ABOM = ENABLE;             /*Disable the automatic bus-off management*/
	  hcanR.Init.AWUM = ENABLE;             /*Disable the automatic wake-up mode*/
	  hcanR.Init.NART = DISABLE;            /*Disable the non-automatic retransmission mode*/
	  hcanR.Init.RFLM = DISABLE;            /*Disable the receive FIFO Locked mode( Once a receive FIFO is full the next incoming message will overwrite the previous one.)*/
	  hcanR.Init.TXFP = DISABLE;             /* Disable the transmit FIFO priority*/




	  hcanR.pRx1Msg->FIFONumber=CAN_FIFO1;            /*Message received in FIFO1*/

	  /*if HAL_CAN_ConfigFilter returns NOT_OK then it is error*/

	  if (HAL_CAN_ConfigFilter(&hcanR, &sFilterConfig) != HAL_OK)
	  {
	    /* Filter configuration Error */
	    Error_Handler();
	  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  /*Configuration to run timer each 1ms*/

   htim3.Instance = TIM3;                          /*using timer3*/
   htim3.Init.Prescaler = 1;
   htim3.Init.CounterMode = TIM_COUNTERMODE_UP;   /*counts up*/
   htim3.Init.Period = 24000;
   htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /*TIM_ClockDivision*/
   htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

   /*if HAL_TIM_Base_Init returns NOT_OK then it is error*/

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitTypeDef GPIO_InitStruct1;
  GPIO_InitTypeDef GPIO_InitStruct2;
  GPIO_InitTypeDef GPIO_InitStruct3;
  GPIO_InitTypeDef GPIO_InitStruct4;


  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();   /*Clock enable for PORTC*/
  __HAL_RCC_GPIOD_CLK_ENABLE();  /*Clock enable for PORTD*/
  __HAL_RCC_GPIOA_CLK_ENABLE();  /*Clock enable for PORTA*/

  /*Configure GPIO pin Output Level */


  /*Configure GPIO pin : PC13 */

  /* Init PIN_13 in PORTC*/

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Init PIN_0 in PORTA*/

   GPIO_InitStruct1.Pin = GPIO_PIN_0;
   GPIO_InitStruct1.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct1.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct1);

   /* Init PIN_1 in PORTA*/

   GPIO_InitStruct2.Pin = GPIO_PIN_1;
    GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct2);


    /* Init PIN_2 in PORTA*/

    GPIO_InitStruct3.Pin = GPIO_PIN_2;
     GPIO_InitStruct3.Mode = GPIO_MODE_OUTPUT_PP;
     GPIO_InitStruct3.Speed = GPIO_SPEED_FREQ_LOW;
     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct3);

     /* Init PIN_3 in PORTA*/

     GPIO_InitStruct4.Pin = GPIO_PIN_3;
      GPIO_InitStruct4.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct4.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct4);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
