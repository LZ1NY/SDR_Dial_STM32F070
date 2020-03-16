/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int  count = 0, count_prev = 0;
unsigned int radio_frequency = 432750000, radio_frequency_prev;
unsigned int sdr_frequency   = 739750000, sdr_frequency_prev;
int tstamp;
bool pressed =  false, RIT_RX = true, RIT_TX = true;
int len;
char tx_buf[20];
char hexbuffer[20];
char textbuf[20];
int pos32=0, i_temp;
// bool data_set_ready = false;
char  rx_buf[15];
int rx_len;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_radio(int deltata){
        char radio_str[11];
        volatile unsigned char cmd[10];
        volatile int MHZ, KHZ, HZ;
        radio_frequency_prev =  radio_frequency;
        radio_frequency += deltata;
        MHZ = (int)(radio_frequency/1000000); KHZ = (int)((radio_frequency-MHZ*1000000)/1000); HZ = (int)(radio_frequency-MHZ*1000000-KHZ*1000);
               // if ((radio_frequency >= 432000000) && (radio_frequency <= 432500000)){ warn = false;}
        //else {warn = true;} 
        sprintf(radio_str," %3d%03d%03d" , MHZ, KHZ, HZ );
        
        cmd[0] = (radio_str[1]-0x30)*16 + radio_str[2]-0x30;
        cmd[1] = (radio_str[3]-0x30)*16 + radio_str[4]-0x30;
        cmd[2] = (radio_str[5]-0x30)*16 + radio_str[6]-0x30;
        cmd[3] = (radio_str[7]-0x30)*16 + radio_str[8]-0x30; 
        cmd[4] =  0x01;
        HAL_UART_Transmit(&huart2, &cmd[0], 5, 10);
        //CDC_Transmit_FS(&cmd[0],5);
}
//------------------------------------------------------
void set_sdr(int deltata){
        int len;
        sdr_frequency_prev = sdr_frequency;
        sdr_frequency += deltata;

        len = sprintf(tx_buf, "F %d\r\n", sdr_frequency);
        CDC_Transmit_FS(tx_buf,len);
         
       
}
//------------------------------------------------------
void check4jump(){
       // int len;
       // len = sprintf(tx_buf, "f\r\n");
       // CDC_Transmit_FS(tx_buf,len);
          if( rx_len >0 ) {    
                int len, len1;
                HAL_Delay(4);
                //HAL_UART_Transmit(&huart2, &rx_buf[0], rx_len, 10); 
                rx_buf[rx_len] =  '\0';
                len  = sprintf(tx_buf,"F %s", rx_buf);
                len1 = sprintf(&tx_buf[len], "\r\n");
                HAL_UART_Transmit(&huart2, &tx_buf[0], len1+len, 10);
                int frq = atoi(rx_buf);
                if ( abs(frq - sdr_frequency_prev) > 1000  ) 
                                { sdr_frequency =  frq; sdr_frequency_prev = frq;
                                set_radio(frq - sdr_frequency);         
                rx_len = 0;     }
                                }
}
//------------------------------------------------------
void setup_wrong_command(){
                len = sprintf(tx_buf,"Wrong command! Enter new command or 'q' to exit!\r\n", rx_buf[0] );
                CDC_Transmit_FS(tx_buf,len);
                HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 0 );
                rx_len = 0;
}
//-------------------------------------------------------
void setup_hello(){
        int len = sprintf(tx_buf,"LZ1NY External Dial\r\nCurrent Setup:\r\n" );
        CDC_Transmit_FS(tx_buf,len);
        HAL_Delay(1);
        len = sprintf(tx_buf,"F1 %d\r\n", sdr_frequency );
        CDC_Transmit_FS(tx_buf,len);
        HAL_Delay(1);
        len = sprintf(tx_buf,"F2 %d\r\n", radio_frequency );
        CDC_Transmit_FS(tx_buf,len);}


void setup(){
        char tmp[12];
        setup_hello();
        while(true) {
                if(rx_len>0) {
                        if(rx_buf[0] == 'q') {      
                                HAL_Delay(1); 
                                len = sprintf(tx_buf,"Exit Setup Mode \r\n" );
                                CDC_Transmit_FS(tx_buf,len);
                                HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 0 );
                                rx_len = 0;
                                return;       }
                        else if(rx_buf[0] ==  'f') {   
                                HAL_Delay(2);   
                                        if (rx_buf[1] ==  '1'){
                                                strncpy(&tmp, &rx_buf[3],9);
                                                sdr_frequency = atoi(tmp)       ;
                                                ;
                                                setup_hello();
                                                                }
                                        else if (rx_buf[1] ==  '2'){
                                                strncpy(&tmp, &rx_buf[3],9);
                                                radio_frequency = atoi(tmp)       ;
                                                ;
                                                setup_hello();
                                                
                                                                        }
                                        else { setup_wrong_command();}

                                len = sprintf(tx_buf,"Set frequency command\r\n" );
                                CDC_Transmit_FS(tx_buf,len);
                                rx_len = 0;
                                    }
                        else  {   setup_wrong_command();   }

                                }

                else {        
                        HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 1 );
                        HAL_Delay(300);    
                        //len = sprintf(tx_buf,"Waiting for input..." );
                        //CDC_Transmit_FS(tx_buf,len);
                                                }
                                }  // end WHILE loop

}

//-------------------------------------------------------


/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
    /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  // Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

        set_radio(0);
        set_sdr(0);



  //  if(HAL_GPIO_ReadPin(KNOB_A5_GPIO_Port, KNOB_A5_Pin) == GPIO_PIN_RESET ) { setup(); }    



  /* USER CODE END 2 */
   /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
        int delta, k;
        //check4jump();
        count = __HAL_TIM_GET_COUNTER(&htim3);
        delta = count - count_prev;
        if (delta != 0){                 k = 1;
                /*
                if ( abs(delta) > 5 ) { k = 2;}
                if ( abs(delta) > 10 ) { k = 3;}  
                if ( abs(delta )> 20 ) { k = 4;}        
                */
               k = 3;
                delta = k*STEP*delta;
                count_prev =  count; 

                if( RIT_TX ){   set_radio(delta);        }            
                if( RIT_RX ){   set_sdr(delta);          }


                }

        if(HAL_GPIO_ReadPin(KNOB_A5_GPIO_Port, KNOB_A5_Pin) == GPIO_PIN_RESET ){    
                if ( pressed == 0)              {
                        pressed = 1;
                        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                        RIT_TX = !RIT_TX;
                        tstamp = HAL_GetTick(); }        
                else {   
                        if( HAL_GetTick() - tstamp > 1100 )
                                { setup(); 
                                pressed = 0; 
                                }
                
                
                
                                  }  
                                                                             }
              
       
	HAL_Delay(300);

        //check4jump();
              
       


//---------------------------------------
/*if(VCP_retrieveInputData( hexbuffer,&pos32 )!=0){
        int len; // you could do data processing here.by demo, i just send it back to PC
        len = sprintf(textbuf, "i=%d data=%s \r\n" , i_temp, hexbuffer);
        //export it to char buffer first. 
        //SendTextMsgToUSB(textbuf); 
       
        CDC_Transmit_FS(textbuf,len);
        //send the text to PC via cdc 
        hexbuffer[0]=  '\0'        ;        }
                      
   i_temp++;
cmd

*/

  }// end while(1)
  /* USER CODE END 3 */
}  // end main()

//-------------------------------------------------------------------------------------------------
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
HAL_TIM_Encoder_Start_IT(&htim3, &sMasterConfig);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA4 KNOB_A5_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|KNOB_A5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
