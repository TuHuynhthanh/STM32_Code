
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ff.h"
#include <string.h>
#include "USB_HID_KEYS.h"
#include "usbd_hid.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS FatFs;
uint8_t fm_inited = 0;
char *txt;
char file_buff[100];

uint8_t rValue;
uint8_t rBuffer[30];
uint8_t rIndex=0;
uint8_t rFlag=0;
uint8_t buff[8]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void create_files_list();
void USB_Write(uint16_t NUMBER);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t getScancode(uint8_t Num)
{uint8_t Co;
	switch (Num)
	{
		case 0:	Co=KEY_0;break;
		case 1:	Co=KEY_1;break;
		case 2:	Co=KEY_2;break;
		case 3:	Co=KEY_3;break;
		case 4:	Co=KEY_4;break;
		case 5:	Co=KEY_5;break;
		case 6:	Co=KEY_6;break;
		case 7:	Co=KEY_7;break;
		case 8:	Co=KEY_8;break;
		case 9:	Co=KEY_9;break;
	}
	return Co;
	
}
void USB_Write(uint16_t NUMBER)
{
	uint8_t i=0,Ngan,Tram,Chuc,Donvi;
	Ngan=NUMBER/1000;
	NUMBER=NUMBER%1000;
	Tram=NUMBER/100;
	NUMBER=NUMBER%100;
	Chuc=NUMBER/10;
	Donvi=NUMBER%10;
	
	
	buff[0]=1;
	buff[1]=0;
	buff[2]=0;
	buff[3]=getScancode(Ngan);
	buff[4]=getScancode(Tram);
	buff[5]=getScancode(Chuc);
	buff[6]=getScancode(Donvi);
	buff[7]=KEY_ENTER;
	USBD_HID_SendReport(&hUsbDeviceFS,buff,8);
	HAL_Delay(100);
	buff[3]=0;
	buff[4]=0;
	buff[5]=0;
	buff[6]=0;
	buff[7]=0;
	USBD_HID_SendReport(&hUsbDeviceFS,buff,8);
}

void fm_init()
{
	if (f_mount(&FatFs, "", 1) == FR_OK) {
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
		fm_inited = 1;
		printf("SDcard has been inited\n\r");
	}
	else
	{
		fm_inited = 0;
		return;
	}
	//create_files_list();
	//f_mount(0, "", 1);
}

uint32_t count_files(char *path)
{
	uint32_t count = 0;
	DIR dir;
	FILINFO fno;
	FRESULT res;
	static char lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
	fno.lfname = lfn;
  fno.lfsize = sizeof(lfn);
	
	res = f_opendir(&dir, path);                       /* Open the directory */
	if (res == FR_OK) {
		for (;;) {
			res = f_readdir(&dir, &fno);                   /* Read a directory item */
      if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
      if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
      if (!(fno.fattrib & AM_DIR)) {                 /* It is a file. */
				count ++;
      }
    }
    f_closedir(&dir);
	}
	return count;
}

void create_files_list()
{	
	printf("cre lst\n");
	char path[] = "/";
	uint32_t count = count_files(path);
	printf("count: %d\n\r", count);
	if(count == 0)
		return;
	
	uint32_t ci = 0;
	

	FRESULT res;
  FILINFO fno;
  DIR dir;
  int i;
  char *fn;   /* This function assumes non-Unicode configuration */
  static char lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
  fno.lfname = lfn;
  fno.lfsize = sizeof(lfn);
	printf("cre lst\n\r");
	res = f_opendir(&dir, path);											 /* Open the directory */
	if (res == FR_OK) {
		i = strlen(path);
		for (;;) {
			res = f_readdir(&dir, &fno);									 /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0) break;	/* Break on error or end of dir */
			if (fno.fname[0] == '.') continue;						 /* Ignore dot entry */
			fn = *fno.lfname ? fno.lfname : fno.fname;
			if (fno.fattrib & AM_DIR) {										/* It is a directory */
				//sprintf(&path[i], "/%s", fn);
				//path[i] = 0;
				printf("fldr\n\r");
				if (res != FR_OK) break;
			} else {																			 /* It is a file. */
				printf("kek \n\r");
				ci++;
			}
		}
		f_closedir(&dir);
	}
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart3,&rValue,1);
//		fm_init();
//		FIL fil; //file object
//		char line[100];
//		UINT NW;
//		FRESULT fr;
//		FILINFO fno;
//		
//		if(f_stat("httu.csv",&fno)==FR_OK) printf("Co file nay\n\r");
//		if(f_stat("httu.csv",&fno)==FR_NO_FILE) {printf("Khong co file nay\n\r"); break;}
//			HAL_Delay(500);
		printf("Test case\n\r");
		
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		float fValue=0;
		char *j;
		uint16_t TL;
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(rFlag==1) {
			fValue=strtof((char *)&rBuffer[8],NULL);
			j=strstr((char *)rBuffer,"k");
			
			if(j!=0)	TL=fValue*1000;
			else	TL=fValue;
			printf("Trong luong=%d g \r\n",TL);
			rFlag=0;
			
			USB_Write(TL);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart3.Instance)
	{
		if(rValue!=13) //not enter
		{
			rBuffer[rIndex++]=rValue;
		}
		else
		{
			rIndex=NULL;
			rFlag=1;
		}
		HAL_UART_Receive_IT(&huart3,&rValue,1);
	}
}
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
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
