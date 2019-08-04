/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "sdcard.h"
#include "buffer.h"
#include "midi.h"
#include "string.h"

#define MAX_LEDS 1000
#define PREAMBLE_SIZE 100
#define LEDBUF_SIZE MAX_LEDS*12+PREAMBLE_SIZE
#define BYTES_PER_LED 3

#define RED   0x0000ff00
#define BLUE  0x00ff0000
#define GREEN 0x000000ff

#define CMD1 "AT+CWMODE=1\r\n"
#define	CMD2 "AT+CWQAP\r\n"
#define CMD3 "AT+CWJAP=\"%s\",\"%s\"\r\n"
#define CMD4 "AT+CIPSTART=\"UDP\",\"192.168.1.1\",21928,%d,2\r\n"
#define CMD_BUF_SIZE 64
#define INITS 4

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef struct {
	uint32_t start;
	uint32_t length;
} table_element;

typedef struct {
	uint32_t leds_per_frame;
	uint32_t frames_per_second;
	uint32_t bytes_per_led;
	uint32_t port;
	uint32_t channel;
	char ssid[16];
	char password[16];
} settings_struct;

extern volatile uint32_t timer;

uint8_t settings_buf[SD_SECTOR_SIZE];
settings_struct *settings = (settings_struct *)settings_buf;

uint8_t table_buf[SD_SECTOR_SIZE*2];
table_element *table = (table_element *)table_buf;

uint32_t buf[MAX_LEDS];
uint8_t ledbuf[LEDBUF_SIZE];

uint8_t patterns[4] = {0x88,0x8c,0xc8,0xcc};
uint32_t bmasks[8] = {0xffffffff,0x7f7f7f7f,0x3f3f3f3f,0x1f1f1f1f,
											0x0f0f0f0f,0x07070707,0x03030303,0x01010101};

char cmd_buf[2][CMD_BUF_SIZE+1];

char *inits[INITS] = {
	CMD1,
	CMD2,
	cmd_buf[0],
	cmd_buf[1],
};

uint8_t flash_present = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void fill_buf(uint32_t start_sector, uint32_t frame) {
	uint8_t brightness = ((127 - midi_get_velocity()) >> 4);
	for(uint16_t current_led=0;current_led<settings->leds_per_frame;current_led++) {
		buf[current_led] = get_item(start_sector, frame * settings->leds_per_frame + current_led, settings->bytes_per_led);
		buf[current_led] = (buf[current_led]>>brightness) & bmasks[brightness];
	}
}

void process_buf() {
	uint32_t c; //current color

#ifdef __NEO	
	uint32_t x;
	uint16_t pp = PREAMBLE_SIZE; //neopixel buffer pointer
	for(uint16_t current_led=0;current_led<settings->leds_per_frame;current_led++) {
		x = buf[current_led];
		c = 0; c|=(x&0x03030303); x>>=2; //reverse bits
		c<<=2; c|=(x&0x03030303); x>>=2;
		c<<=2; c|=(x&0x03030303); x>>=2;
		c<<=2; c|=(x&0x03030303);
		for(uint16_t current_byte=0;current_byte<BYTES_PER_LED;current_byte++) 
			for(uint8_t i=0;i<4;i++){
				ledbuf[pp++] = patterns[(c&0x00000003)];
				c>>=2;
			}
	}
#endif

#ifdef __PWM
	c = buf[0];
	htim3.Instance->CCR1 = 256 - (c & 0x000000ff);
	c>>=8;
	htim3.Instance->CCR2 = 256 - (c & 0x000000ff);
	c>>=8;
	htim4.Instance->CCR2 = 256 - (c & 0x000000ff);
	c>>=8;
	htim4.Instance->CCR1 = 256 - (c & 0x000000ff);

#endif 	
}

void clean_buf() {
#ifdef __NEO	
	for(uint16_t i=PREAMBLE_SIZE;i<LEDBUF_SIZE;i++) {
		ledbuf[i] = 0x88;
	}
#endif

#ifdef __PWM
		htim3.Instance->CCR1 = 256;
		htim3.Instance->CCR2 = 256;
		htim4.Instance->CCR1 = 256;
		htim4.Instance->CCR2 = 256;
#endif	
}

void signal(uint32_t color) {
	buf[0] = color;
	process_buf();
}

uint8_t get_note() {
	return midi_get_note();
}

void init_wifi() {
	HAL_GPIO_WritePin(ESP_RST_GPIO_Port,ESP_RST_Pin,GPIO_PIN_RESET); //reset ESP8266
  HAL_Delay(100);
	HAL_GPIO_WritePin(ESP_RST_GPIO_Port,ESP_RST_Pin,GPIO_PIN_SET);
	HAL_Delay(2000);

  snprintf(cmd_buf[0],CMD_BUF_SIZE,CMD3,settings->ssid,settings->password);
	snprintf(cmd_buf[1],CMD_BUF_SIZE,CMD4,settings->port);
	for(uint8_t i=0;i<INITS;i++) {
		HAL_UART_Transmit(&huart1, (uint8_t *)inits[i], strlen(inits[i]), 1);
		if (i==2)
			HAL_Delay(20000);
		HAL_Delay(200);
	}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
uint32_t start_sector = 0;
uint32_t frame_counter = 0;
uint8_t note=UINT8_MAX;	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
	settings->bytes_per_led = 3;
	settings->leds_per_frame = 1;

	signal(RED);

#ifdef __NEO
	HAL_SPI_Transmit_DMA(&hspi2,ledbuf,LEDBUF_SIZE);
#endif

#ifdef __PWM
  htim3.Instance->CCR1 = 256;
	htim3.Instance->CCR2 = 256;
	htim4.Instance->CCR1 = 256;
	htim4.Instance->CCR2 = 256;
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
#endif

	while (SD_init()!=0);

	SD_ReadSector(0,settings_buf);
	SD_ReadSector(1,table_buf);
	SD_ReadSector(2,table_buf+SD_SECTOR_SIZE);
	
	signal(BLUE);
	
	init_wifi();
	start_midi(settings->channel & 0x0000000f);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		while (timer>0);
		if(note!=get_note()) {
			note = get_note();
			if (note<128)
				start_sector = table[note].start;
			else
				start_sector = 0;
			frame_counter = 0;	
		}
		
		if(start_sector==0)
			clean_buf();
		else {
			timer = 1000 / settings->frames_per_second;
			frame_counter%=(table[note].length);
			fill_buf(start_sector,frame_counter++);
			process_buf();
		}
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
