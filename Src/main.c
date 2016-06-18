/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "usb_device.h"
#include "NRF_Utilities.c"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/          
          
uint8_t PowerUpTx[2] = {0x20, 0x02};
//uint8_t ConfigBuffer[2] = {0x00, 0x00};
uint8_t PowerUpRx[2] = {0x20, 0x03};
uint8_t RFSetup[2] = {0x26, 0x26};
uint8_t Nop[2] = {0xFF, 0xFF};
uint8_t RxPayloadSize[2] = {0x31, 0x20};
uint8_t ClearStatus[2] = {0x27, 0x70};
uint8_t SetNoAutoretransmit[2] = {0x24, 0xF0};
uint8_t FlushTx = 0xE1;
uint8_t FlushRx = 0xE2;
uint8_t LoadTxBuffer[34];
uint8_t ReadRxBuffer[34];
uint8_t RXIncrementor;
uint8_t RXERROR;

uint8_t rData[35];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SetupReceiver();
void SetupTransmitter();
uint8_t Tx_Transmit(uint8_t *pData, uint16_t size);
uint8_t Rx_Transmit(uint8_t *pData, uint16_t size);
void TransmitPayload(uint8_t *payload, uint16_t size);
void ReceivePayload();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  for(uint8_t i = 0; i < 35; i++){//Initialize rData
    rData[i] = 0;
  }
  
  LoadTxBuffer[0] = 0xA0; //Load Buffer Command
  
  ReadRxBuffer[0] = 0x61;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(SPI2_CE_GPIO_Port, SPI2_CE_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  SetupTransmitter();
  SetupReceiver();
  HAL_Delay(100);
  uint8_t count[32];
  for(uint8_t i = 0; i < 32; i++){count[i] = 0x16;}
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
   
  /* USER CODE BEGIN 3 */
    //Load TX Buffer
    //Transmit Packet
    TransmitPayload(&count[0], 32);
    //Wait for IRQ
    while(Rx_Transmit(&Nop[0], 1)!=0x40);
    //Read Packet
    ReceivePayload();
    //HAL_Delay(50);
    count[5]++;
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  HAL_SPI_Init(&hspi1);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  HAL_SPI_Init(&hspi2);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_7B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> TSC_G1_IO3
     PA3   ------> TSC_G1_IO4
     PA6   ------> TSC_G2_IO3
     PA7   ------> TSC_G2_IO4
     PB0   ------> TSC_G3_IO2
     PB1   ------> TSC_G3_IO3
     PB10   ------> I2C2_SCL
     PB11   ------> I2C2_SDA
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin 
                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin|SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CE_GPIO_Port, SPI2_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CE_GPIO_Port, SPI1_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin 
                           LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin 
                          |LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_IRQ_Pin */
  GPIO_InitStruct.Pin = SPI2_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI2_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_Pin SPI1_CE_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin|SPI1_CE_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CE_Pin */
  GPIO_InitStruct.Pin = SPI2_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI2_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_IRQ_Pin */
  GPIO_InitStruct.Pin = SPI1_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_IRQ_GPIO_Port, &GPIO_InitStruct);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PB6);

}

/* USER CODE BEGIN 4 */
uint8_t Rx_Transmit(uint8_t *pData, uint16_t size){
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, pData, &rData[0], size, 10);
  while(!__HAL_SPI_GET_FLAG(&hspi2,SPI_FLAG_TXE));
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin, GPIO_PIN_SET);
  
  return(rData[0]);
}

uint8_t Tx_Transmit(uint8_t *pData, uint16_t size){
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, pData, &rData[0], size, 10);
  while(!__HAL_SPI_GET_FLAG(&hspi1,SPI_FLAG_TXE));
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
  
  return(rData[0]);
}

void SetupReceiver(){
   uint8_t ConfigBuffer[4] = {0x00, 0x00, 0x00, 0x00};
  //int a = W_REGISTER | CONFIG;
  ConfigBuffer[0] = 0x20;//W_REGISTER | CONFIG;//Set device to RX
  ConfigBuffer[1] = 0x09;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x21;//W_REGISTER | EN_AA;//Enable autoack only on pipe 0
  ConfigBuffer[1] = 0x01;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x22;//W_REGISTER + EN_RXADDR;//Enable RX pipe 0
  ConfigBuffer[1] = 0x01;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x23;//W_REGISTER + SETUP_AW;//Setup address width 3 bytes
  ConfigBuffer[1] = 0x01;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x24;//W_REGISTER + SETUP_RETR;//Disable Retransmission
  ConfigBuffer[1] = 0x00;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x25;//W_REGISTER + RF_CH;//Setup RF Channel/Same as RX
  ConfigBuffer[1] = 0x02;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x26;//W_REGISTER + RF_SETUP;//Setup RF(250kbps, 0dbm)
  ConfigBuffer[1] = 0x26;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x27;//W_REGISTER + STATUS;//Clear Interrupts
  ConfigBuffer[1] = 0xF0;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x2A;//W_REGISTER + RX_ADDR_P0;//Set RX Address
  ConfigBuffer[1] = 0xE7;
  ConfigBuffer[2] = 0xE7;
  ConfigBuffer[3] = 0xE7;
  Rx_Transmit(&ConfigBuffer[0], 4);
  
  ConfigBuffer[0] = 0x30;//W_REGISTER + TX_ADDR;//Set TX Address
  ConfigBuffer[1] = 0xE7;
  ConfigBuffer[2] = 0xE7;
  ConfigBuffer[3] = 0xE7;
  Rx_Transmit(&ConfigBuffer[0], 4);
  
  ConfigBuffer[0] = 0x31;//W_REGISTER + RX_PW_P0;//Set Payload length 32 Bytes
  ConfigBuffer[1] = 0x20;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x3C;//W_REGISTER + DYNPD;//Disable dynamic payload
  ConfigBuffer[1] = 0x00;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x3D;//W_REGISTER + FEATURE;//Disable features
  ConfigBuffer[1] = 0x00;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x20;//W_REGISTER + CONFIG;//Power Up device
  ConfigBuffer[1] = 0x0B;
  Rx_Transmit(&ConfigBuffer[0], 2);
  
  Rx_Transmit(&FlushTx, 1);
  Rx_Transmit(&FlushRx, 1);
}

void SetupTransmitter(){
  uint8_t ConfigBuffer[4] = {0x00, 0x00, 0x00, 0x00};
  //int a = W_REGISTER | CONFIG;
  ConfigBuffer[0] = 0x20;//W_REGISTER | CONFIG;//Set device to TX
  ConfigBuffer[1] = 0x08;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x21;//W_REGISTER | EN_AA;//Enable autoack only on pipe 0
  ConfigBuffer[1] = 0x01;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x22;//W_REGISTER + EN_RXADDR;//Enable RX pipe 0
  ConfigBuffer[1] = 0x01;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x23;//W_REGISTER + SETUP_AW;//Setup address width 3 bytes
  ConfigBuffer[1] = 0x01;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x24;//W_REGISTER + SETUP_RETR;//Disable Retransmission
  ConfigBuffer[1] = 0x00;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x25;//W_REGISTER + RF_CH;//Setup RF Channel/Same as RX
  ConfigBuffer[1] = 0x02;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x26;//W_REGISTER + RF_SETUP;//Setup RF(250kbps, 0dbm)
  ConfigBuffer[1] = 0x26;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x27;//W_REGISTER + STATUS;//Clear Interrupts
  ConfigBuffer[1] = 0xF0;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x2A;//W_REGISTER + RX_ADDR_P0;//Set RX Address
  ConfigBuffer[1] = 0xE7;
  ConfigBuffer[2] = 0xE7;
  ConfigBuffer[3] = 0xE7;
  Tx_Transmit(&ConfigBuffer[0], 4);
  
  ConfigBuffer[0] = 0x30;//W_REGISTER + TX_ADDR;//Set TX Address
  ConfigBuffer[1] = 0xE7;
  ConfigBuffer[2] = 0xE7;
  ConfigBuffer[3] = 0xE7;
  Tx_Transmit(&ConfigBuffer[0], 4);
  
  ConfigBuffer[0] = 0x31;//W_REGISTER + RX_PW_P0;//Set Payload length 32 Bytes
  ConfigBuffer[1] = 0x20;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x3C;//W_REGISTER + DYNPD;//Disable dynamic payload
  ConfigBuffer[1] = 0x00;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x3D;//W_REGISTER + FEATURE;//Disable features
  ConfigBuffer[1] = 0x00;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = 0x20;//W_REGISTER + CONFIG;//Power Up device
  ConfigBuffer[1] = 0x0A;
  Tx_Transmit(&ConfigBuffer[0], 2);
  
  Tx_Transmit(&FlushTx, 1);
  Tx_Transmit(&FlushRx, 1);
}

void TransmitPayload(uint8_t *payload, uint16_t size){
  Tx_Transmit(&FlushTx, 1);
  Tx_Transmit(&ClearStatus[0], 2);
  for(uint8_t i = 1; i < size + 1; i++){
    LoadTxBuffer[i] = payload[i-1];
  }
  
  Tx_Transmit(&LoadTxBuffer[0], 33);
  
  HAL_GPIO_WritePin(GPIOB, SPI1_CE_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, SPI1_CE_Pin, GPIO_PIN_RESET);
  uint8_t cont = 0;
  /*
  while(!cont){
    if((rData[0]&&0x20)==0x20)
      cont = 1;
    Tx_Transmit(&Nop[0], 1);  
  }*/  
  
  return;
}

void ReceivePayload(){
  Rx_Transmit(&ReadRxBuffer[0], 33);
  if(rData[6] != RXIncrementor+1)
    RXERROR++;
  RXIncrementor = rData[6];
  
  return;
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
