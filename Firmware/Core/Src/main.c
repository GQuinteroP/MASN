/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "arm_math.h"
#include "stdio.h"
#include "PondFilter.h"
#include "ThirdOctave.h"
#include "Custom_DSP.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "MAX-M10S.h"
#include "time.h"
#include "string.h"
#include "m95p32.h"
#include "SIMCOM7022.h"
#include "EEPROM.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_hal_sai.h"
#include "usb_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_lpuart_rx;
DMA_HandleTypeDef hdma_lpuart_tx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* Definitions for main_task */
osThreadId_t main_taskHandle;
const osThreadAttr_t main_task_attributes = {
  .name = "main_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for signal_processi */
osThreadId_t signal_processiHandle;
const osThreadAttr_t signal_processi_attributes = {
  .name = "signal_processi",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for georeferencing_ */
osThreadId_t georeferencing_Handle;
const osThreadAttr_t georeferencing__attributes = {
  .name = "georeferencing_",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for usb_cmd_queue */
osMessageQueueId_t usb_cmd_queueHandle;
const osMessageQueueAttr_t usb_cmd_queue_attributes = {
  .name = "usb_cmd_queue"
};
/* Definitions for leqs_buffer */
osMessageQueueId_t leqs_bufferHandle;
const osMessageQueueAttr_t leqs_buffer_attributes = {
  .name = "leqs_buffer"
};
/* Definitions for timeout_LPWA */
osTimerId_t timeout_LPWAHandle;
const osTimerAttr_t timeout_LPWA_attributes = {
  .name = "timeout_LPWA"
};
/* Definitions for mutex_buffer */
osMutexId_t mutex_bufferHandle;
const osMutexAttr_t mutex_buffer_attributes = {
  .name = "mutex_buffer"
};
/* Definitions for mutex_lpwa */
osMutexId_t mutex_lpwaHandle;
const osMutexAttr_t mutex_lpwa_attributes = {
  .name = "mutex_lpwa"
};
/* Definitions for sem_125 */
osSemaphoreId_t sem_125Handle;
const osSemaphoreAttr_t sem_125_attributes = {
  .name = "sem_125"
};
/* Definitions for sem_georef */
osSemaphoreId_t sem_georefHandle;
const osSemaphoreAttr_t sem_georef_attributes = {
  .name = "sem_georef"
};
/* Definitions for sem_start_stop */
osSemaphoreId_t sem_start_stopHandle;
const osSemaphoreAttr_t sem_start_stop_attributes = {
  .name = "sem_start_stop"
};
/* Definitions for sem_mem_write */
osSemaphoreId_t sem_mem_writeHandle;
const osSemaphoreAttr_t sem_mem_write_attributes = {
  .name = "sem_mem_write"
};
/* Definitions for sem_mem_read */
osSemaphoreId_t sem_mem_readHandle;
const osSemaphoreAttr_t sem_mem_read_attributes = {
  .name = "sem_mem_read"
};
/* Definitions for sem_lwpa */
osSemaphoreId_t sem_lwpaHandle;
const osSemaphoreAttr_t sem_lwpa_attributes = {
  .name = "sem_lwpa"
};
/* Definitions for sem_send_data */
osSemaphoreId_t sem_send_dataHandle;
const osSemaphoreAttr_t sem_send_data_attributes = {
  .name = "sem_send_data"
};
/* Definitions for sem_usb */
osSemaphoreId_t sem_usbHandle;
const osSemaphoreAttr_t sem_usb_attributes = {
  .name = "sem_usb"
};
/* Definitions for sem_gnss */
osSemaphoreId_t sem_gnssHandle;
const osSemaphoreAttr_t sem_gnss_attributes = {
  .name = "sem_gnss"
};
/* Definitions for sem_init */
osSemaphoreId_t sem_initHandle;
const osSemaphoreAttr_t sem_init_attributes = {
  .name = "sem_init"
};
/* Definitions for sem_process_LPWA */
osSemaphoreId_t sem_process_LPWAHandle;
const osSemaphoreAttr_t sem_process_LPWA_attributes = {
  .name = "sem_process_LPWA"
};
/* Definitions for sem125_counting */
osSemaphoreId_t sem125_countingHandle;
const osSemaphoreAttr_t sem125_counting_attributes = {
  .name = "sem125_counting"
};
/* USER CODE BEGIN PV */

/* Definitions for usb_task */
osThreadId_t usb_taskHandle;
const osThreadAttr_t usb_task_attributes = { .name = "usb_task", .stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityAboveNormal,};

/* Definitions for IoT_task */
osThreadId_t IoT_taskHandle;
const osThreadAttr_t IoT_task_attributes = { .name = "IoT_task", .stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityNormal,};

/* Definitions for offline_task */
osThreadId_t offline_taskHandle;
const osThreadAttr_t offline_task_attributes = { .name = "offline_task", .stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityBelowNormal,};

/* Definitions for gnss_task */
osThreadId_t gnss_taskHandle;
const osThreadAttr_t gnss_task_attributes = { .name = "gnss_task", .stack_size = 2048 * 4,
		.priority = (osPriority_t) osPriorityNormal,};

const osThreadAttr_t data_send_task_attributes = { .name = "task_data_send", .stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityNormal + 1,};

/*const osThreadAttr_t data_store_task_attributes = { .name = "task_data_store", .stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityNormal,};*/

/* RTOS resources for DMA-based debug */
osMutexId_t debug_mutexHandle;
const osMutexAttr_t debug_mutex_attributes = {
  .name = "debug_mutex"
};

osSemaphoreId_t debug_dma_semHandle;
const osSemaphoreAttr_t debug_dma_sem_attributes = {
  .name = "debug_dma_sem"
};

/* Static buffer to prevent memory corruption during DMA transfer */
static char debug_buffer[256];

RTC_TimeTypeDef rtc_time;
RTC_DateTypeDef rtc_date;
struct tm time_ref;

//Config values
uint32_t BankNumber = 0;
uint32_t Address = 0, PAGEError = 0;
__IO uint32_t MemoryProgramStatus = 0;
__IO uint64_t data64 = 0;
static FLASH_EraseInitTypeDef EraseInitStruct;
const uint64_t Data64_To_Prog[FLASH_ROW_SIZE] = {
  0x1234567890123456, 0x1111111111111111, 0x2222222222222222, 0x3333333333333333,
  0x4444444444444444, 0x5555555555555555, 0x6666666666666666, 0x7777777777777777,
  0x8888888888888888, 0x9999999999999999, 0xAAAAAAAAAAAAAAAA, 0xBBBBBBBBBBBBBBBB,
  0xCCCCCCCCCCCCCCCC, 0xDDDDDDDDDDDDDDDD, 0xEEEEEEEEEEEEEEEE, 0xFFFFFFFFFFFFFFFF,
  0x0011001100110011, 0x2233223322332233, 0x4455445544554455, 0x6677667766776677,
  0x8899889988998899, 0xAABBAABBAABBAABB, 0xCCDDCCDDCCDDCCDD, 0xEEFFEEFFEEFFEEFF,
  0x2200220022002200, 0x3311331133113311, 0x6644664466446644, 0x7755775577557755,
  0xAA88AA88AA88AA88, 0xBB99BB99BB99BB99, 0xEECCEECCEECCEECC, 0xFFDDFFDDFFDDFFDD};

//USB
uint8_t usb_cmd[usb_buff_len];
extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t USB_plugged = 0;

uint8_t LPWGNS_config = false;
uint8_t buffer_rx[2];
uint8_t buffer_lpwa_old[buffer_lwpa_config_size] = {0};	//TODO: OLD?
uint16_t buffer_lpwa_len = 0;
uint16_t write_lpwa_index = 0;
uint16_t read_lpwa_index = 0;

uint8_t gps_enabled = false;		//Use gps coordinates
uint8_t lpwa_enabled = false;		//Use LPWA
uint8_t lpwa_flag_rx = false;		//Flag - awaiting for message

float32_t cal = 0; //21.8
uint16_t LeqTime = 1;  	//Leq time period - '0' value indicates fast Leq (Up to 1 hour - 3600 seconds)
uint16_t RecTime = 0;    	//Recording time - '0' value indicates push button control recording
uint8_t octave = true;  //'True' or 'false' calculate octave bands

//Variables
time_t endTime;     //End time of recording
uint8_t start=false;   //Flag to keep track of measuring status

//uint8_t	sampling_buffer[fast_125ms*4]; // fast_125ms samples of 32 bits (4 bytes) - 24 bits data
uint8_t	sampling_buffer[fast_125ms*4] __attribute__((section(".ram2_data")));

//float32_t working_buffer[fast_125ms + 96] = {0} ;
float32_t working_buffer[fast_125ms + 96] __attribute__((section(".ram2_data")));

//float32_t acc_dec_buffer[fast_125ms + 96] = {0};
float32_t acc_dec_buffer[fast_125ms + 96] __attribute__((section(".ram2_data")));

float32_t aux_low_freqs[4][250];
uint8_t acc_dec_ctr = 0;
time_t seconds =  0;

#ifdef NO_MIC
	float32_t sine_working_buffer[fast_125ms + 96] = {0};
#endif

uint8_t	sai_half = 0;

int8_t sync_val = 0;	// Value used to sync the 125 ms since the acquisition seems not multiple of 8 TODO: To be removed
int32_t sync_rtc_time = -1;

/*enum STATE
{
	WAIT = 0,
	GEOREF,
	CODING,
	DOUT
} task_state;*/

//Measurement data
uint8_t leqs_buffer_acq1[block_len];	//Auxiliary buffer to store 1 block of data
uint8_t leqs_buffer_acq2[block_len];	//Auxiliary buffer to store 1 block of data

int16_t n_meas = 0;				//Counter of measurements so that a block is filled until block_length
uint16_t block_ctr = 0;				//Counter of number of blocks
uint8_t *buff_addr_acq = &leqs_buffer_acq1[0];			//Address of active buffer to store samples
uint8_t *buff_addr_snd = &leqs_buffer_acq2[0];			//Address of past samples to be saved in leqs_buffer_snd

int16_t	eeprom_data_len = 0;
int16_t eeprom_read_idx = 0;
int16_t eeprom_write_idx = 0;
uint8_t eeprom_aux_buffer[M95P32_PAGESIZE];

osThreadId_t data_send_taskHandle;
osThreadId_t data_store_taskHandle;
osThreadId_t data_rcv_taskHandle;

//DSP
arm_biquad_cascade_df2T_instance_f32 iirS;
arm_biquad_cascade_df2T_instance_f32 iirDec8;
arm_rfft_fast_instance_f32 fft_inst;
int32_t initIRR = 5*NUM_SECTIONS;

float32_t iir_state[4*iir_order];
float32_t iir_dec_state8[4*iir_order_dec_o06_f08];
float32_t Pxx = 0.0;    //Calculate power
float32_t Pxf = 0.0;   //Power in dB

//float32_t outFFT[fast_125ms + 96];  //Out var for FFT calculations
float32_t outFFT[fast_125ms + 96] __attribute__((section(".ram2_data")));

//Leqs
uint32_t auxSave=0;  //Seconds cpo
uint8_t LeqFastCount=0;   //Number of fast samples taken
uint8_t LeqMinutesCount=0;  // Count of number of minutes
float32_t LeqFastSamples[NLeqSlow];   //Save 8 Leq samples to get slow Leq
float32_t LeqSeconds[60];  //60 seconds values
float32_t LeqFastTOB[NFilters][NLeqSlow];   //Third octave band levels for fast sampling
float32_t LeqSecondsTOB[NFilters][60];  //60 seconds values
float32_t LeqAuxTOB[NFilters];    //Leq value computed for third octave bands to be saved
float32_t LeqMinutes[60];  // 60 minutes values
float32_t LeqMinutesTOB[NFilters][60];  // 60 minutes values
uint8_t LeqSecondsCount=0;  //Count of number of obtained seconds

#ifdef IIR_DEC
	uint8_t LeqFastTOBCount=0;     //Number of fast samples taken for third Octave Band
#endif

//GNSS
MAX_M10S GPS;
uint8_t buffer_gps1[gnss_buff_len];
uint8_t *buffer_gps_tmp1 = buffer_gps1;
uint8_t buffer_gps2[gnss_buff_len];
uint8_t *buffer_gps_tmp2 = buffer_gps2;

uint8_t gnss_rx_buffer[gnss_buff_len];
uint16_t    old_pos = 0;
uint16_t    parse_idx = 0;

struct BUFF_GPS
{
	uint8_t *buffer_gps;
	uint8_t valid;
};

uint16_t gps_counter = 0;
uint16_t gps_str_len = 0;

struct BUFF_GPS GNSS;

//LPWA
const uint8_t	server_IP[15] =	"127.0.0.1";
const uint16_t	server_port = 5435;


uint8_t buffer_lpwa[250];		//Buffer for storing characters received (Higher than queue to avoid overflow)
uint8_t n_res = 0;
uint8_t coap_custom_res_flag = 0;
uint8_t coap_custom_res[50];
extern struct LWPA_def LPWA;

uint16_t counter_lpwa =0;		//Received characters counter
uint16_t last_count_lpwa =0;	//Received characters counter (last count)

uint16_t counter_n_lpwa =0;		//Received \r's counter
uint16_t n_res_lpwa = 0;		//Number of expected responses (to count for \r's)
uint8_t IMEI[6] = {0}; // 6 dígitos + terminador nulo para evitar basura en memoria

HAL_StatusTypeDef res_uart = HAL_OK;

struct BUFF_G
{
	uint16_t bufferLength;	//Actual buffer length
	uint16_t writeIndex;	 //	Increase writeIndex position to prepare for next write
	uint16_t readIndex;
	uint8_t	ncols;
	uint8_t nrows;
	uint8_t  **bufferMain;
};
struct BUFF_G BUFF_main, BUFF_usb, BUFF_LPWA;

uint8_t leqs_buffer_main[main_buffer_size][block_len + 1];	//Main buffer to store "buffer_size" of data
uint8_t leqs_buffer_LWPA[max_blocks_LPWA][block_len];	//Auxiliary buffer to store "n_blocks_LPWA" of data that will be sent through LPWA
//uint8_t conf_vector[max_blocks_LPWA]={0};				//Auxiliary buffer to control which confirmations have been received
uint8_t USB_buff_msgs[usb_max_n_msg][usb_buff_len];		//Buffer for the USB communications
uint8_t LPWA_buff[max_blocks_LPWA][LPWA_rx_len];		//Buffer for rx (uC->LPWA)

osStatus_t MAIN_Queue_put(struct BUFF_G *queue, uint8_t *msg_in, uint8_t msg_len);
osStatus_t MAIN_Queue_get(struct BUFF_G *queue, uint8_t *msg_out);
void MAIN_Queue_init(struct BUFF_G *queue, uint8_t ** buff, uint8_t ncols, uint8_t nrows);

//DEBUG
uint32_t benchmark_ctr = 0;
uint32_t benchmark_time = 0;
uint32_t tcount=1;
uint32_t prevTime = 0;
uint8_t tmp_leq[200];
#ifdef debug_GNSS
	uint8_t tmp_gps[200];
#endif
float32_t delta = (2*M_PI*8000)/32000;
float32_t out_counter = 0;

#ifdef IIR_DEBUG
	uint8_t LeqFastTOBCount=0;     //Number of fast samples taken for third Octave Band
	arm_biquad_cascade_df2T_instance_f32 iirDec;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SAI1_Init(void);
void task_main(void *argument);
void task_signal_processing(void *argument);
void task_georeferencing(void *argument);
void rst_timeout_LPWA(void *argument);

/* USER CODE BEGIN PFP */
osStatus_t LPWA_enable();
osStatus_t LPWA_disable();
void task_gnss(void *argument);
static void GPIO_ConfigAN(void);
void debug(const char *fmt, ...);
void stopSampling();
void restart_UART_DMA();
void buffer_show();
void loadValues();
void store_config_64(uint64_t config_data);
void green_led(uint8_t status);
void red_led(uint8_t status);
void load_config(float32_t *cal, uint8_t *gps_enabled, uint8_t *lpwa_enabled,
			uint8_t *octave, uint16_t *RecTime, uint16_t	*LeqTime);
void SystemClock_Config_LOW(void);
void SystemClock_Config_LOW_IIR(void);
void SystemClock_Config_HIGH (void);
size_t encode_byte_stuffing(const uint8_t *src, size_t len, uint8_t *dst);

void flush_main_buffer_to_eeprom(uint32_t total_blocks);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void debug(const char *fmt, ...)
{
    va_list args;
    int len;

    if (osKernelGetState() == osKernelRunning)
    {
        /* Acquire mutex to protect the shared static buffer and UART peripheral */
        if (osMutexAcquire(debug_mutexHandle, osWaitForever) == osOK)
        {
            va_start(args, fmt);
            vsnprintf(debug_buffer, sizeof(debug_buffer), fmt, args);
            va_end(args);

            len = strlen(debug_buffer);

            /* Transmit in blocking mode with a strict 5 ms timeout */
            /* Prevents RTOS deadlock if hardware fails */
            HAL_UART_Transmit(&huart1, (uint8_t*)debug_buffer, len, 5);

            osMutexRelease(debug_mutexHandle);
        }
    }
    else
    {
        /* Fallback for pre-RTOS initialization (Blocking mode) */
        char local_buffer[256];
        va_start(args, fmt);
        vsnprintf(local_buffer, sizeof(local_buffer), fmt, args);
        va_end(args);

        len = strlen(local_buffer);
        HAL_UART_Transmit(&huart1, (uint8_t*)local_buffer, len, 1000);
    }
}

/* UART Tx Complete Callback to unblock the debug function */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

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
  uint32_t fpscr = __get_FPSCR();
    fpscr |= (1UL << 24); // Flush-to-Zero (FZ)
    __set_FPSCR(fpscr);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	#ifdef LOW_FREQ
	  MX_GPIO_Init();
	  //MX_USART1_UART_Init();

	  HAL_Delay(100);
	  SET_BIT(PWR->CR2, PWR_PVM_1);
	  HAL_Delay(500);
	  if (!HAL_IS_BIT_SET(PWR->SR2, PWR_SR2_PVMO1)) //Detect USBVDD
	  {
		  debug("\r\n USB ON");
		  SystemClock_Config_HIGH();
		  MX_GPIO_Init();
		  MX_DMA_Init();
		  MX_LPUART1_UART_Init();
		  MX_RTC_Init();
		  MX_SAI1_Init();
		  MX_USART1_UART_Init();
		  MX_SPI1_Init();
		  MX_USART3_UART_Init();
		  MX_USB_DEVICE_Init();
	  }else
	  {
		  debug("\r\n USB OFF");
		  CLEAR_BIT(PWR->CR2, PWR_PVM_1);
		  SystemClock_Config_LOW();	//16 MHz
		  MX_GPIO_Init();
		  MX_DMA_Init();
		  MX_LPUART1_UART_Init();
		  MX_RTC_Init();
		  MX_SAI1_Init();
		  MX_USART1_UART_Init();
		  MX_SPI1_Init();
		  MX_USART3_UART_Init();
	  }
	#else
	  SystemClock_Config_HIGH();
	  MX_GPIO_Init();
	  MX_DMA_Init();
	  MX_LPUART1_UART_Init();
	  MX_RTC_Init();
	  MX_SAI1_Init();
	  MX_USART1_UART_Init();
	  MX_SPI1_Init();
	  MX_USART3_UART_Init();
	  HAL_Delay(100);
	  SET_BIT(PWR->CR2, PWR_PVM_1);
	  HAL_Delay(500);
	  if (!HAL_IS_BIT_SET(PWR->SR2, PWR_SR2_PVMO1)) //Detect USBVDD
		  MX_USB_DEVICE_Init();
	#endif

  //GPIO_ConfigAN();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of mutex_buffer */
  mutex_bufferHandle = osMutexNew(&mutex_buffer_attributes);

  /* creation of mutex_lpwa */
  mutex_lpwaHandle = osMutexNew(&mutex_lpwa_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  debug_mutexHandle = osMutexNew(&debug_mutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sem_125 */
  sem_125Handle = osSemaphoreNew(1, 0, &sem_125_attributes);

  /* creation of sem_georef */
  sem_georefHandle = osSemaphoreNew(1, 0, &sem_georef_attributes);

  /* creation of sem_start_stop */
  sem_start_stopHandle = osSemaphoreNew(1, 0, &sem_start_stop_attributes);

  /* creation of sem_mem_write */
  sem_mem_writeHandle = osSemaphoreNew(1, 0, &sem_mem_write_attributes);

  /* creation of sem_mem_read */
  sem_mem_readHandle = osSemaphoreNew(1, 0, &sem_mem_read_attributes);

  /* creation of sem_lwpa */
  sem_lwpaHandle = osSemaphoreNew(1, 0, &sem_lwpa_attributes);

  /* creation of sem_send_data */
  sem_send_dataHandle = osSemaphoreNew(1, 0, &sem_send_data_attributes);

  /* creation of sem_usb */
  sem_usbHandle = osSemaphoreNew(1, 0, &sem_usb_attributes);

  /* creation of sem_gnss */
  sem_gnssHandle = osSemaphoreNew(1, 0, &sem_gnss_attributes);

  /* creation of sem_init */
  sem_initHandle = osSemaphoreNew(1, 0, &sem_init_attributes);

  /* creation of sem_process_LPWA */
  sem_process_LPWAHandle = osSemaphoreNew(1, 1, &sem_process_LPWA_attributes);

  /* creation of sem125_counting */
  sem125_countingHandle = osSemaphoreNew(10, 0, &sem125_counting_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  debug_dma_semHandle = osSemaphoreNew(1, 0, &debug_dma_sem_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of timeout_LPWA */
  timeout_LPWAHandle = osTimerNew(rst_timeout_LPWA, osTimerOnce, NULL, &timeout_LPWA_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of usb_cmd_queue */
  usb_cmd_queueHandle = osMessageQueueNew (30, sizeof(uint8_t), &usb_cmd_queue_attributes);

  /* creation of leqs_buffer */
  leqs_bufferHandle = osMessageQueueNew (40, sizeof(uint8_t), &leqs_buffer_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of main_task */
  main_taskHandle = osThreadNew(task_main, NULL, &main_task_attributes);

  /* creation of signal_processi */
  signal_processiHandle = osThreadNew(task_signal_processing, NULL, &signal_processi_attributes);

  /* creation of georeferencing_ */
  georeferencing_Handle = osThreadNew(task_georeferencing, NULL, &georeferencing__attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  gnss_taskHandle = osThreadNew(task_gnss, NULL, &gnss_task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 19;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV19;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_AUTOBAUDRATE_INIT;
  hlpuart1.AdvancedInit.AutoBaudRateEnable = UART_ADVFEATURE_AUTOBAUDRATE_ENABLE;
  hlpuart1.AdvancedInit.AutoBaudRateMode = UART_ADVFEATURE_AUTOBAUDRATE_ONSTARTBIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 400000;
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  time_ref.tm_hour = 0x00;
  time_ref.tm_min = 0x00;
  time_ref.tm_sec = 0x00;
  time_ref.tm_mday = 0x01;
  time_ref.tm_mon = 0x01;
  time_ref.tm_year = 0x22;

  /*RTC_TimeTypeDef time;
  RTC_DateTypeDef date;
  time.Hours = 0x11;
  time.Minutes = 0x55;
  time.Seconds = 00;
  date.WeekDay = RTC_WEEKDAY_TUESDAY;
  date.Year = 0x23;
  date.Month = RTC_MONTH_JUNE;
  date.Date	= 0x07;
  HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BCD);
  HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BCD);*/

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_32K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_MONOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */
  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPS_NRST_Pin|STATUS_P_Pin|STATUS_N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|LPWA_NRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPS_NRST_Pin STATUS_P_Pin STATUS_N_Pin */
  GPIO_InitStruct.Pin = GPS_NRST_Pin|STATUS_P_Pin|STATUS_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB_REC_Pin */
  GPIO_InitStruct.Pin = PB_REC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PB_REC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LPWA_NRST_Pin */
  GPIO_InitStruct.Pin = LPWA_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LPWA_NRST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void GPIO_ConfigAN(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure all GPIO as analog to reduce current consumption on non used IOs */
  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  /* GPIOF and GPIOG not defined */
  __HAL_RCC_GPIOH_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  /* GPIOF and GPIOG not defined */
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  /* GPIOF and GPIOG not defined */
  __HAL_RCC_GPIOH_CLK_DISABLE();
}

void red_led(uint8_t status)
{
	HAL_GPIO_WritePin(STATUS_P_GPIO_Port, STATUS_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STATUS_N_GPIO_Port, STATUS_N_Pin, status);
}

void green_led(uint8_t status)
{
	HAL_GPIO_WritePin(STATUS_P_GPIO_Port, STATUS_P_Pin, status);
	HAL_GPIO_WritePin(STATUS_N_GPIO_Port, STATUS_N_Pin, GPIO_PIN_RESET);
}

int8_t buffer_lpwa_write(const uint8_t c)
{

	if (buffer_lpwa_len == buffer_lwpa_config_size)
	{
		//debug("\r\nBuffer lpwa is full!");
		return -1;
	}

	buffer_lpwa_old[write_lpwa_index] = c;
	buffer_lpwa_len++;	 //	Increase buffer size after writing
	write_lpwa_index++;	 //	Increase writeIndex position to prepare for next write

	// If at last index in buffer, set writeIndex back to 0
	if (write_lpwa_index == buffer_lwpa_config_size)
		write_lpwa_index = 0;

	return 0;
}


/*void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	benchmark_ctr++;
}*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	if(CHECK_BIT(GPIO_Pin, 13))
		osSemaphoreRelease(sem_start_stopHandle);

	#ifdef DEBUG
		debug("\r\n*[HAL_GPIO_EXTI_Callback]\r\n");
	#endif
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	debug("Alarm!\r\n");
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	sai_half = 0;
	//osSemaphoreRelease(sem_125Handle);
	osSemaphoreRelease(sem125_countingHandle);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	sai_half = 1;
	//osSemaphoreRelease(sem_125Handle);
	osSemaphoreRelease(sem125_countingHandle);
	#ifdef LOW_POWER	//Already commented out
		////------ Power saving ---////
		/*if((USB_plugged == 0) & (LPWA.status !=2))
		{
			HAL_SuspendTick();
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		}*/
	#endif
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == LPUART1)
    {
        uint16_t new_data_len = 0;
        uint16_t first_part_len = 0;

        /* 1. Calculate incoming data length */
        if (Size > old_pos) {
            new_data_len = Size - old_pos;
        } else if (Size < old_pos) {
            new_data_len = gnss_buff_len - old_pos + Size;
        }

        /* 2. Validate bounds and transfer data */
        if (new_data_len > 0)
        {
            /* Clamp the transfer to avoid hard faults if logic slips */
            if ((parse_idx + new_data_len) >= gnss_buff_len) {
                new_data_len = gnss_buff_len - parse_idx - 1;
            }

            /* 3. Execute memory transfer safely */
            if (Size > old_pos) {
                memcpy(&buffer_gps_tmp1[parse_idx], &gnss_rx_buffer[old_pos], new_data_len);
            } else {
                first_part_len = gnss_buff_len - old_pos;
                if(first_part_len > new_data_len) first_part_len = new_data_len;
                memcpy(&buffer_gps_tmp1[parse_idx], &gnss_rx_buffer[old_pos], first_part_len);

                if (new_data_len > first_part_len) {
                    memcpy(&buffer_gps_tmp1[parse_idx + first_part_len], gnss_rx_buffer, new_data_len - first_part_len);
                }
            }

            parse_idx += new_data_len;
            old_pos = Size;

            /* Ensure null termination */
            buffer_gps_tmp1[parse_idx] = '\0';


            if ((parse_idx > 0 && buffer_gps_tmp1[parse_idx - 1] == '\n') ||
                (parse_idx >= (gnss_buff_len - 50)))
            {
                /* Swap pointers for double buffering */
                GNSS.buffer_gps = buffer_gps_tmp1;
                buffer_gps_tmp1 = buffer_gps_tmp2;
                buffer_gps_tmp2 = GNSS.buffer_gps;

                /* Clear assembler buffer for next fragments */
                memset(buffer_gps_tmp1, 0, gnss_buff_len);
                parse_idx = 0;

                GNSS.valid = true;
                osSemaphoreRelease(sem_gnssHandle);
            }
        }
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//TODO: MUst be improved so that less processes are performed here
	if(LPWGNS_config == true)
	{
		if(huart == &huart3)	//uC < - LPWA
		{
			buffer_lpwa_write(buffer_rx[0]);	//TODO: Just use buffer_rx without array
			HAL_UART_Receive_DMA(&huart3, buffer_rx, 1);
		}
	}
	else
	{
		/*if(huart == &hlpuart1)
		{
			if(start==true)
				HAL_UART_Receive_DMA(huart, &buffer_gps_tmp1[gps_counter], 1);

			if(buffer_gps_tmp1[gps_counter-1]=='$')
			{
				buffer_gps_tmp1[gps_counter-1] = 0;
				gps_counter=0;
				GNSS.buffer_gps = buffer_gps_tmp1;
				buffer_gps_tmp1 = buffer_gps_tmp2;
				buffer_gps_tmp2 = GNSS.buffer_gps;
				GNSS.valid = true;
				osSemaphoreRelease(sem_gnssHandle);
			}
			else
			{
				gps_counter++;
				if (gps_counter >= gnss_buff_len)
				{
					gps_counter = 0;
				}
			}

		}else */if(huart == &LPWA_UART)
		{
			if(counter_lpwa==0)
				osSemaphoreRelease(sem_process_LPWAHandle);
			HAL_UART_Receive_DMA(&LPWA_UART, &buffer_lpwa[counter_lpwa], 1);
			//debug("%c", buffer_lpwa[counter_lpwa]);
			counter_lpwa++;

		}
	}


}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	osSemaphoreRelease(sem_mem_writeHandle);
	////debug("Tx!\r\n");
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	osSemaphoreRelease(sem_mem_readHandle);
	////debug("Rx!\r\n");
}

time_t rtc_read(void)
{
	struct tm timeinfo;
	time_t	t;
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

	timeinfo.tm_mon  = rtc_date.Month -1 ;
	timeinfo.tm_mday = rtc_date.Date;
	uint16_t y = rtc_date.Year;
	timeinfo.tm_year =  (uint16_t)(y+2000-1900);
	timeinfo.tm_hour = rtc_time.Hours;
	timeinfo.tm_min  = rtc_time.Minutes;
	timeinfo.tm_sec  = rtc_time.Seconds;

	t = mktime(&timeinfo);

	return t;
}

void restart_UART_DMA()
{
	HAL_UART_Abort(&LPWA_UART);
	HAL_UART_DMAStop(&LPWA_UART);
	HAL_UART_DeInit(&LPWA_UART);

	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	MX_USART3_UART_Init();
	vTaskDelay(100);
	#ifdef DEBUG
		debug("\r\n[restart_UART_DMA] DONE!");
	#endif
}

int8_t buffer_lpwa_read(uint8_t *c)
{
	if (buffer_lpwa_len == 0)
	{
		//debug("\r\nBuffer lpwa is empty!");
		return -1;
	}

	*c = buffer_lpwa_old[read_lpwa_index];

	buffer_lpwa_len--;	 //	Decrease buffer size after reading
	read_lpwa_index++;	 //	Increase readIndex position to prepare for next read

	// If at last index in buffer, set readIndex back to 0
	if (read_lpwa_index == buffer_lwpa_config_size)
		read_lpwa_index = 0;

	return 0;
}

void MAIN_Queue_init(struct BUFF_G *queue, uint8_t ** buff, uint8_t ncols, uint8_t nrows)
{
	queue->bufferMain = buff;
	queue->ncols = ncols;
	queue->nrows = nrows;
}

osStatus_t MAIN_Queue_put(struct BUFF_G *queue, uint8_t *msg_in, uint8_t msg_len)
{
	if (queue->bufferLength == queue->nrows)
	{
		#ifdef DEBUG
			debug("\r\n*[Queue_put] Buffer is full!");
		#endif
		return osError;
	}

	uint8_t (*arr)[queue->ncols] = (uint8_t (*)[queue->ncols]) (uint8_t*)((uint32_t) queue->bufferMain + (queue->writeIndex)*(queue->ncols));
	arr[0][0] = msg_len;		//First byte is used for string length
	memcpy(&arr[0][1], msg_in, msg_len);

	queue->bufferLength++;	 //	Increase buffer size after writing

	#ifdef DEBUG
		//debug("\r\n*[Queue_put] bufferLength[%d] msg_len:%d - [%s]", queue->bufferLength, msg_len, (char *) &arr[queue->writeIndex][1]);
	#endif

	queue->writeIndex++;	 //	Increase writeIndex position to prepare for next write

	// If at last index in buffer, set writeIndex back to 0
	if (queue->writeIndex == queue->nrows)
		queue->writeIndex = 0;

	return osOK;
}

osStatus_t MAIN_Queue_get(struct BUFF_G *queue, uint8_t *msg_out)
{
	if (queue->bufferLength == 0)
	{
		#ifdef DEBUG
			//debug("\r\nBuffer is empty!");
		#endif
		return osError;
	}

	uint8_t (*arr)[(queue->ncols)*(queue->nrows)] = (uint8_t (*)[(queue->ncols)*(queue->nrows)]) (uint8_t*)((uint32_t) queue->bufferMain + (queue->readIndex)*(queue->ncols));
	memcpy(msg_out, &arr[0][1], arr[0][0]);//First byte is used for string length

	#ifdef DEBUG
		//debug("\r\n*[Queue_get] bufferLength[%d] msg_len:%d - [%s]", queue->bufferLength, queue->bufferMain[queue->readIndex][0], msg_out);
		/*debug("\r\n*[MAIN_Queue_get] bufferLength[%d] msg_len:%d \r\nOut: [", queue->bufferLength, arr[0][0]);
		for(int jj=0;jj<arr[0][0];jj++)
			debug("%02x", msg_out[jj]);
		debug("]\r\n");*/
	#endif

	memset(arr, 0, queue->ncols);

	queue->bufferLength--;	 //	Decrease buffer size after reading
	queue->readIndex++;	 //	Increase readIndex position to prepare for next read

	// If at last index in buffer, set readIndex back to 0
	if (queue->readIndex == queue->nrows)
		queue->readIndex = 0;

	return osOK;
}

osStatus_t eeprom_init_queue()
{
	//Continue after last measurement
	eeprom_data_len = get_eeprom_data_len();
	if(eeprom_data_len>0)
	{
		eeprom_read_idx = get_eeprom_read_idx();
		eeprom_write_idx = eeprom_read_idx + eeprom_data_len;
	}else
	{
		eeprom_read_idx = 0;
		eeprom_write_idx = 0;
	}

	#ifdef debug_EEPROM
		debug("\r\n*[eeprom_init_queue] EEPROM len:%d - r_idx:%d - w_idx: %d\r\n",
				eeprom_data_len, eeprom_read_idx, eeprom_write_idx);
	#endif

	return osOK;
}

osStatus_t eeprom_write(uint8_t *data)
{
	if (eeprom_data_len == eeprom_size)
	{
		#ifdef debug_EEPROM
			debug("\r\nEEPROM is full!");
		#endif
		return osError;
	}
	WRITE_ENABLE();
	if(Page_Write(data, eeprom_write_idx*M95P32_PAGESIZE, block_len*2)!=M95_OK)
	{
		#ifdef debug_EEPROM
			debug("\r\neeprom_write ERROR!");
		#endif
		return osError;
	}
	//vTaskDelay(2);
	WRITE_DISABLE();

	eeprom_data_len++;	 //	Increase buffer size after writing
	eeprom_write_idx++;	 //	Increase writeIndex position to prepare for next write

	// If at last index in buffer, set writeIndex back to 0
	if (eeprom_write_idx == eeprom_size)
		eeprom_write_idx = 0;
	#ifdef debug_EEPROM
		debug("\r\n*[eeprom_write] EEPROM r_idx:%d - w_idx:%d - len: %d\r\n",
				eeprom_read_idx, eeprom_write_idx, eeprom_data_len);
	#endif
	return osOK;
}

osStatus_t eeprom_read(uint8_t *data_out, uint8_t clear)
{
	if (eeprom_data_len == 0)
	{
		#ifdef debug_EEPROM
			debug("\r\n*[eeprom_read] EEPROM is empty!");
		#endif
		return osErrorResource;
	}

	if(Single_Read(data_out, eeprom_read_idx*M95P32_PAGESIZE, block_len*2)!=M95_OK)
	{
		#ifdef debug_EEPROM
			debug("\r\n*[eeprom_read] eeprom_read ERROR!");
		#endif
		return osError;
	}

	//TODO:Data clear after it is sent
	if(clear==1)
	{
		uint8_t clr = 0xFF;
		WRITE_ENABLE();
		if(Page_Write(&clr, eeprom_read_idx*M95P32_PAGESIZE, 1)!=M95_OK)
		{
			#ifdef debug_EEPROM
				debug("\r\n*[eeprom_read] eeprom_write ERROR!");
			#endif
			return osError;
		}
		//vTaskDelay(2);
		WRITE_DISABLE();
	}

	eeprom_data_len--;	 //	Decrease buffer size after reading
	eeprom_read_idx++;	 //	Increase readIndex position to prepare for next read

	// If at last index in buffer, set readIndex back to 0
	if (eeprom_read_idx == eeprom_size)
		eeprom_read_idx = 0;

	#ifdef debug_EEPROM
		debug("\r\n*[eeprom_read] EEPROM r_idx:%d - w_idx:%d - len: %d\r\n",
				eeprom_read_idx, eeprom_write_idx, eeprom_data_len);
	#endif
	return osOK;
}

/*void task_data_store(uint8_t *argument)
{
	uint8_t	 curr_size = (uint8_t) ((uint8_t) *argument)/2;
	uint8_t tmp_buffer[block_len*2];

	#ifdef DEBUG
		debug("\r\n*[task_data_store] curr_size: %d!", curr_size);
	#endif

	for(int ii=0; ii<curr_size; ii++)
	{
		memcpy(&tmp_buffer[0], &leqs_buffer_LWPA[ii*2][0], block_len);
		memcpy(&tmp_buffer[block_len], &leqs_buffer_LWPA[(ii*2) + 1][0], block_len);
		eeprom_write(tmp_buffer);
	}
	#ifdef LOW_POWER
		//-- Power saving --//
		if((USB_plugged == 0) & (LPWA.status !=2))
		{
			HAL_SuspendTick();
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		}
	#endif

	#ifdef DEBUG
		debug("\r\n*[task_data_store] TERMINATED!");
	#endif

	osThreadExit();
	vTaskDelete(data_store_taskHandle);
}*/

void task_data_rcv(void *argument)
{
	HAL_StatusTypeDef res_uart;
	uint8_t timeout_ctr = 0;

	vTaskDelay(2000);
	if((res_uart=HAL_UART_Receive_DMA(&LPWA_UART, buffer_lpwa, 1))!=HAL_OK)
	{
		//TODO: Error handling
		debug("\r\n*[task_data_rcv] Error rx[%d]", res_uart);
	}
	else
	{
		#ifdef debug_LPWA
			debug("\r\n*[task_data_rcv] initiating!");
		#endif

		while((LPWA.status !=1) && (LPWA.status !=3))
		{
			osSemaphoreAcquire(sem_process_LPWAHandle, osWaitForever);
			vTaskDelay(10);	//10 - To wait for the rest of the string
			timeout_ctr=0;
			while( (buffer_lpwa[counter_lpwa]!='\r') && (buffer_lpwa[counter_lpwa+1]!='\n') && (timeout_ctr<10))
			{
				vTaskDelay(1);
				timeout_ctr++;
			}

			if(lpwa_flag_rx)
			{
				MAIN_Queue_put(&BUFF_LPWA, &buffer_lpwa, counter_lpwa);
				#ifdef debug_LPWA
					/*debug("\r\n*[task_data_rcv] Data rx[%d]: ", counter_lpwa);
					int ii=0;
					while(buffer_lpwa[ii]!='\0')
					{
						debug("%c", buffer_lpwa[ii]);
						ii++;
					}*/
					//debug("\r\n");
				#endif
			}
			#ifdef debug_LPWA
				else
					debug("Unsolicited code: %s\r\n", buffer_lpwa);
			#endif
			memset(buffer_lpwa, 0, sizeof buffer_lpwa);
			counter_lpwa = 0;
		}
	}
	#ifdef debug_LPWA
		debug("\r\n*[task_data_rcv] TERMINATED!\r\n");
	#endif
	restart_UART_DMA();
	osThreadExit();
	vTaskDelete(data_rcv_taskHandle);
}

void task_data_send(void *argument)
{
	// TO EXTRACT MISSING SECONDS IN POSTGRESQL
	/*
		WITH secuencias AS (
			SELECT
				datetime,
				LAG(datetime) OVER (ORDER BY datetime ASC) AS prev_datetime
			FROM public.noise
		)
		SELECT
			generate_series(
				prev_datetime + INTERVAL '1 second',
				datetime - INTERVAL '1 second',
				INTERVAL '1 second'
			) AS segundo_faltante
		FROM secuencias
		WHERE EXTRACT(EPOCH FROM (datetime - prev_datetime)) > 1;
	*/
	uint8_t n_blocks = (uint8_t)((uint32_t)argument);
	static uint8_t escaped_payload[260];
	size_t final_len = 0;

	#ifdef debug_LPWA
		debug("\r\n[task_data_send] STARTED!");
		/*for(int curr_block=0; curr_block < n_blocks; curr_block++)
		{
			debug("\r\n\r\n");
			for(int ii=0; ii<block_len;ii++)
				debug("%02x,", leqs_buffer_LWPA[curr_block][ii]);
			debug("\r\n\r\n");
		}*/
	#endif

	lpwa_flag_rx = true;
	if(S7022_AT(300) != osOK)
		goto task_delete;
	if(S7022_CEREG(500, 10)!= osOK)
		goto task_delete;
	if(S7022_COAPSTART(300)!= osOK)
		goto task_delete;
	if(S7022_COAPOPEN(300)!= osOK)
		goto task_delete;

	for(int curr_block=0; curr_block < n_blocks; curr_block++)
	{
		final_len = encode_byte_stuffing(leqs_buffer_LWPA[curr_block], block_len, escaped_payload);
		//debug("\r\n**** [task_data_send_testing] - Final len: %d", final_len);

		//const uint8_t *server_resp = "0";
		//const uint8_t *server_resp = "1";
		const uint8_t *server_resp = (curr_block == n_blocks - 1) ? (const uint8_t *)"1" : (const uint8_t *)"0";
		if(S7022_COAPHEAD(curr_block, server_resp, 300) ==osOK)
		{
			vTaskDelay(20);
			if(S7022_COAPOPTION(300) ==osOK)
			{
				vTaskDelay(20);
				//if(S7022_COAPSENDTX(leqs_buffer_LWPA[curr_block], block_len, 200) == osOK)
				if(final_len<255)
				{
					S7022_COAPSENDTX(escaped_payload, final_len, 5000);
					vTaskDelay(200);
				}
				#ifdef debug_LPWA
					else
						debug("\r\n[task_data_send] final_len>255!");
				#endif
			}
		}
	}
	#ifdef debug_LPWA
		debug("\r\n[task_data_send] Packets confirmation started!");
	#endif
	bitmask_ready = 0;
	received_bitmask = 0;
	vTaskDelay(1000);

	#ifdef wait_ack
		//TODO: Dynamic confirmation (multiple timeouts)
		uint8_t timeuout_counter = 0;
		while(timeuout_counter<90) // 9 s max
		{
			vTaskDelay(100);
			S7022_Confirm_Rx(leqs_buffer_LWPA);

			if(bitmask_ready == 1)
			{
				for(uint16_t i = 0; i < n_blocks; i++)
				{
					// Check if bit 'i' is 0 (packet lost or corrupted)
					if((received_bitmask & (1ULL << i)) == 0)
					{
						if(osMutexAcquire(mutex_bufferHandle, 100) == osOK)
						{
							MAIN_Queue_put(&BUFF_main, &leqs_buffer_LWPA[i][0], block_len);
							osMutexRelease(mutex_bufferHandle);
						}
						#ifdef debug_LPWA
							debug("\r\n[task_data_send] Corrupted packet: %d!", i);
						#endif
					}
				}
				break;
			}
			timeuout_counter++;
		}


		if(S7022_COAPCLOSE(300)!= osOK)
			goto task_delete;
		if(S7022_COAPSTOP(5000)!= osOK)
			goto task_delete;

	#else
		vTaskDelay(500);
		if(S7022_COAPCLOSE(300)!= osOK)
			goto task_delete;
		if(S7022_COAPSTOP(2000)!= osOK)
			goto task_delete;
	#endif


	task_delete:
		if((bitmask_ready == 0 && received_bitmask == 0) || (timeuout_counter >= 90))
		{
			// Timeout occurred: No response received from server. Re-enqueue all blocks from this burst
			for(uint16_t i = 0; i < n_blocks; i++)
			{
				if(osMutexAcquire(mutex_bufferHandle, 100) == osOK)
				{
					MAIN_Queue_put(&BUFF_main, &leqs_buffer_LWPA[i][0], block_len);
					osMutexRelease(mutex_bufferHandle);
				}
			}
			//Probably server down
			debug("\r\n[task_data_send] Timeout reached!");
			S7022_error(&LPWA.errors.S7022_SERVER_RESP);
		}
		/*if(S7022_QCPMUCFG(1, 300)==osOK)
		#ifdef debug_LPWA
				debug("\r\n PSM Activated!");
			else
				debug("\r\n PSM error!");
		#else
			continue;
		#endif*/
		lpwa_flag_rx = false;

		if((n_blocks_LPWA > n_blocks_LPWA_limit) && (LPWA.status != 1))
					LPWA_disable();

		for(uint16_t curr_block=0; curr_block<n_blocks; curr_block++)
			memset(leqs_buffer_LWPA[curr_block], 0, block_len);

		//memset(conf_vector, 0, n_blocks);
		#ifdef debug_LPWA
			debug("\r\n*[task_data_send] TERMINATED!");
		#endif
		if(LPWA.status == 2)
			LPWA.status = 0;

		osThreadExit(); //
		vTaskDelete(data_send_taskHandle);

}

size_t encode_byte_stuffing(const uint8_t *src, size_t len, uint8_t *dst)
{
    size_t dst_idx = 0;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t b = src[i];

        if (b == 0x03 || b == 0x1A || b == 0x1B || b == 0x1C || b == 0x7D)
        {
            dst[dst_idx++] = 0x7D;
            dst[dst_idx++] = b ^ 0x20;
        }
        else
        {
            dst[dst_idx++] = b;
        }
    }
    return dst_idx;
}

osStatus_t LPWA_enable()
{
	osStatus ret = osOK;
	uint8_t lctr = 0;
	if(lpwa_enabled)
	{
		LPWA.status = 4;	//Starting

		data_rcv_taskHandle = osThreadNew(task_data_rcv, NULL, &usb_task_attributes);	//Launch thread to start receiving data from LPWA
		HAL_GPIO_WritePin(RST_port, RST_pin, GPIO_PIN_SET);		//Enable LPWA hardware
		vTaskDelay(3500);

		lpwa_flag_rx = true;	//Enable LPWA data queuing

		ret = S7022_AT(300);
		while((ret!= osOK) & (lctr<3))
		{
			ret = S7022_AT(300);
			lctr++;
		}
		lpwa_flag_rx = false;
		if(ret != osOK)
		{
			#ifdef debug_LPWA
				debug("\r\n[LPWA_enable] ENABLE ERROR!\r\n");
			#endif
			HAL_GPIO_WritePin(RST_port, RST_pin, GPIO_PIN_RESET);		//Disable LPWA hardware
			restart_UART_DMA();
			LPWA.status = 3;
			lpwa_flag_rx = false;	//Disable LPWA data queuing
			vTaskDelay(500);
			return osError;
		}
	}

	#ifdef debug_LPWA
		debug("\r\n[LPWA_enable] ENABLED!\r\n");
	#endif

	vTaskDelay(300);
	LPWA.status = 0;

	return osOK;
}

osStatus_t LPWA_disable()
{
	if(lpwa_enabled)
	{
		//LPWA.status = 4;	//Starting
		HAL_GPIO_WritePin(RST_port, RST_pin, GPIO_PIN_RESET);		//Disable LPWA hardware
		vTaskDelay(50);
		LPWA.status = 3;
		lpwa_flag_rx = false;	//Disable LPWA data queuing
		osSemaphoreRelease(sem_process_LPWAHandle);
	}

	#ifdef debug_LPWA
		debug("\r\n[LPWA_disable] DISABLED!\r\n");
	#endif

	return osOK;
}

void task_IoT(void *argument)
{
  /* USER CODE BEGIN task_IoT */
	uint8_t eeprom_buffer[block_len*2];
	int16_t send_buffer_length = 0;
	vTaskDelay(10);

	#ifdef debug_LPWA
		debug("\r\n[task_IoT](%s:%d)\r\n!", server_IP, server_port);
	#endif

	//TODO: Should be changed when increasing update period
	/*if(LPWA_enable()!=osOK)
	{
		#ifdef debug_LPWA
			debug("\r\n[task_IoT] Error initiating LPWA\r\n");
		#endif
	}*/

	osSemaphoreRelease(sem_initHandle);

	//**** TEST_BED *****//
	/*uint8_t test_blocks = 1; // Or whatever N you want to test
	fill_controlled_data(test_blocks);
	vTaskDelay(10000);
	data_send_taskHandle = osThreadNew((void *) task_data_send_testing, (void *) &test_blocks, &data_send_task_attributes);
	while(1){};*/

	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(sem_send_dataHandle, osWaitForever);
		if(osMutexAcquire(mutex_bufferHandle,100)==osOK)
			MAIN_Queue_put(&BUFF_main, buff_addr_snd, block_len);
		#ifdef debug_LPWA
			else
				debug("\r\n*[task_IoT] Error obtaining mutex_buffer (MAIN_Queue_put(&BUFF_main, buff_addr_snd, block_len))\r\n");
		#endif

		send_buffer_length = BUFF_main.bufferLength;
		osMutexRelease(mutex_bufferHandle);

		#ifdef debug_LPWA
			debug("\r\n[task_IoT] readIndex:%d - writeIndex:%d - bufferLength: %d - LPWA.status:%d\r\n",
					BUFF_main.readIndex, BUFF_main.writeIndex, send_buffer_length, LPWA.status);
		#endif

		if(send_buffer_length<n_blocks_full_buffer)
		{
			if(send_buffer_length>=n_blocks_LPWA)
			{
				if(LPWA.status == 3)// LPWA OFF
				{
					if(LPWA_enable()!=osOK)
					{
						LPWA.status = 3;
					}
				}

				if(LPWA.status == 0)
				{
					LPWA.status = 2;
					if(osMutexAcquire(mutex_bufferHandle,100)==osOK)
					{
						if (send_buffer_length>max_blocks_LPWA)
							send_buffer_length = max_blocks_LPWA;

						for(int ii=0;ii<send_buffer_length;ii++)
							MAIN_Queue_get(&BUFF_main, &leqs_buffer_LWPA[ii][0]);

						osMutexRelease(mutex_bufferHandle);
						data_send_taskHandle = osThreadNew((void *) task_data_send, (void *) (uint32_t) send_buffer_length, &data_send_task_attributes);
						if (data_send_taskHandle == NULL)
						{
							LPWA_disable();
							if(osMutexAcquire(mutex_bufferHandle, 100) == osOK)
							{
								for(int ii=0;ii<send_buffer_length;ii++)
									MAIN_Queue_put(&BUFF_main, &leqs_buffer_LWPA[ii][0], block_len);
								osMutexRelease(mutex_bufferHandle);
							}
							#ifdef debug_LPWA
								debug("\r\n[task_IoT] Error launching task_data_send\r\n");
							#endif
						}
					}
				}
			}
			else if( (eeprom_data_len > 0) && (send_buffer_length < (n_blocks_LPWA - 6)) && ( (LPWA.status == 0) | (LPWA.status == 3)))
			{
				uint8_t blocks_to_read = (eeprom_data_len >= n_blocks_EEPROM) ? n_blocks_EEPROM : eeprom_data_len;
				for(int jj=0; jj < blocks_to_read; jj++)
				{
					if (eeprom_read(eeprom_buffer, 1) == osOK)
					{
						if(osMutexAcquire(mutex_bufferHandle, 100) == osOK)
						{
							MAIN_Queue_put(&BUFF_main, &eeprom_buffer[0], block_len);
							MAIN_Queue_put(&BUFF_main, &eeprom_buffer[block_len], block_len);
							osMutexRelease(mutex_bufferHandle);
						}
					}
				}
			}
			#ifdef debug_LPWA
			/*else
				debug("\r\n[task_IoT] task_data_send not launched[%d]!\r\n", LPWA.status);*/
			#endif
		}else
		{
			/*if(osMutexAcquire(mutex_bufferHandle,100)==osOK)
			{
				for(int ii=0;ii<send_buffer_length;ii++)
					MAIN_Queue_get(&BUFF_main, &leqs_buffer_LWPA[ii][0]);
				osMutexRelease(mutex_bufferHandle);
			}
			#ifdef debug_LPWA
			else
				debug("\r\n*[task_IoT] Error obtaining mutex_buffer! (data_store_taskHandle)\r\n");
			#endif*/

			flush_main_buffer_to_eeprom(send_buffer_length);

			#ifdef LOW_POWER
				if((USB_plugged == 0) && (LPWA.status != 2))
				{
					HAL_SuspendTick();
					HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
				}
			#endif
		}
	}
  /* USER CODE END task_IoT */
}

void flush_main_buffer_to_eeprom(uint32_t total_blocks)
{

    total_blocks &= ~1;
    uint8_t curr_size = (uint8_t) (total_blocks / 2);
    uint8_t tmp_buffer[block_len * 2];

	#ifdef debug_EEPROM
    	debug("\r\n[flush_main_buff] started!");
	#endif

    for(int ii = 0; ii < curr_size; ii++)
    {
        if(osMutexAcquire(mutex_bufferHandle, 100) == osOK)
        {
            MAIN_Queue_get(&BUFF_main, &tmp_buffer[0]);
            MAIN_Queue_get(&BUFF_main, &tmp_buffer[block_len]);
            osMutexRelease(mutex_bufferHandle);

            eeprom_write(tmp_buffer);
        }
    }
	#ifdef debug_EEPROM
    	debug("\r\n[flush_main_buff] finished!");
	#endif
}

void task_offline(void *argument)
{
	/* USER CODE BEGIN task_offline */
	#ifdef debug_offline
		debug("\r\n[Offline TASK] Offline TASK!");
	#endif
	int16_t send_buffer_length = 0;
	osSemaphoreRelease(sem_initHandle);
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(sem_send_dataHandle, osWaitForever);
		if(osMutexAcquire(mutex_bufferHandle,100)==osOK)
		{
			#ifdef debug_offline
				//for(int ii=0; ii<block_len;ii++)
					//debug("%02x,", leqs_buffer_acq1[ii]);
			#endif
			MAIN_Queue_put(&BUFF_main, buff_addr_snd, block_len);
		}
		#ifdef debug_offline
			else
				debug("\r\n*[Offline TASK] Error obtaining mutex_buffer (MAIN_Queue_put(&BUFF_main, buff_addr_snd, block_len))");
		#endif

		send_buffer_length = BUFF_main.bufferLength;
		osMutexRelease(mutex_bufferHandle);
		#ifdef debug_offline
			debug("\r\n*[Offline TASK] BUFF_main r_idx:%d - w_idx:%d - len: %d\r\n", BUFF_main.readIndex, BUFF_main.writeIndex, BUFF_main.bufferLength);
		#endif
		if(send_buffer_length>=n_blocks_EEPROM)
		{
			/*if(osMutexAcquire(mutex_bufferHandle,100)==osOK)
			{
				for(int ii=0;ii<send_buffer_length;ii++)
					MAIN_Queue_get(&BUFF_main, &leqs_buffer_LWPA[ii][0]);
				osMutexRelease(mutex_bufferHandle);
				data_store_taskHandle = osThreadNew((void *)task_data_store, (void *) &send_buffer_length, &data_store_task_attributes);
			}
			#ifdef debug_offline
			else
				debug("\r\n*[Offline TASK] Error obtaining mutex_buffer! (send_buffer_length>=n_blocks_EEPROM)");
			#endif*/
			flush_main_buffer_to_eeprom(send_buffer_length);
		}else
		{
			#ifdef LOW_POWER
				//-- Power saving --//
				if((USB_plugged == 0) & (LPWA.status !=2))
				{
					HAL_SuspendTick();
					HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
				}
			#endif
		}

	}
/* USER CODE END task_offline */
}

void task_usb(void *argument)
{
	/* USER CODE BEGIN task_usb */
	uint8_t tmp, end_c = 00;
	uint16_t data_len = 0, tout = 100;
	uint16_t rd_idx = 0;
	uint64_t p;
	uint64_t config_val = 0;
	HAL_StatusTypeDef hal_stat=HAL_OK;
	RTC_TimeTypeDef time, rtc_time;
	RTC_DateTypeDef date, rtc_date;
	uint8_t currTimeBuff[50];

	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(sem_usbHandle, osWaitForever);

		#ifdef debug_USB
			debug("\r\n*[task_usb]Queue out [%d]: ", BUFF_usb.bufferLength);
		#endif

		while(BUFF_usb.bufferLength>0)
		{
			MAIN_Queue_get(&BUFF_usb, usb_cmd);
			#ifdef debug_USB
				for(int ii=0;ii<usb_cmd[0];ii++)
					debug("%x", usb_cmd[ii+1]);
			#endif
		}

		switch(usb_cmd[1])
		{
			case 0xFF:	//Validate PC connection sending the random bytes back
				CDC_Transmit_FS(&usb_cmd[1], usb_cmd[0]);
				#ifdef debug_USB
					debug("\r\nConnected!");
				#endif
			break;

			case 0x01:	//Get EEPROM data length
				data_len = get_eeprom_data_len();
				eeprom_init_queue();
				#ifdef debug_USB
					debug("\r\nData_len: %d", data_len);
				#endif
				CDC_Transmit_FS((uint8_t*)&data_len, 2);
			break;

			case 0x02: //Read EEPROM data
				/*memcpy(&data_len, &usb_cmd[2], 2);
				data_len+=rd_idx;
				Single_Read(eeprom_aux_buffer, data_len*M95P32_PAGESIZE, block_len*2);*/

				memcpy(&data_len, &usb_cmd[2], 2);
				eeprom_read(eeprom_aux_buffer, 0);
				#ifdef debug_USB
					debug("\r\nSel data: %d/%d", data_len, eeprom_data_len);
					/*debug("\r\nRead: ");
					for(int ii=0;ii<block_len*2;ii++)
						debug("%02x", eeprom_aux_buffer[ii]);*/
				#endif
				CDC_Transmit_FS(eeprom_aux_buffer, block_len*2);
			break;

			case 0x03:	//Enter LPWA configuration
				memcpy(&tout, &usb_cmd[2], 2);

				#ifdef debug_USB
					debug("\r\nIoT Config! (Tout=%d)", tout);
				#endif

				LPWGNS_config = true;
				if((hal_stat=HAL_UART_Receive_DMA(&huart3, buffer_rx, 1))!=HAL_OK)
					debug("\r\nIoT Error receiving [%d]", hal_stat);
				while(1)
				{
					#ifdef debug_USB
						debug("\r\nIoT-CMD:");
						for(uint8_t ii=0; ii<(usb_cmd[0] - 2);ii++)
							debug("%c", usb_cmd[3+ii]);
						debug("\r\n");
					#endif

					HAL_UART_Transmit_DMA(&LPWA_UART, &usb_cmd[3], usb_cmd[0] - 2);
					vTaskDelay(tout);
					while(buffer_lpwa_len)
					{
						buffer_lpwa_read(&tmp);
						//HAL_UART_Transmit(&huart1, &test1, 1,100);
						CDC_Transmit_FS(&tmp, 1);
						debug("[%c]", tmp);
					}
					CDC_Transmit_FS(&end_c, 1);
					break;
				}
				LPWGNS_config = false;
				restart_UART_DMA();

				#ifdef debug_USB
					debug("\r\nIoT Configuration EXIT!");
				#endif
			break;

			case 0x04:	//Get configuration value from flash memory
				p = *(uint64_t *)ADDR_FLASH_PAGE_252;
				#ifdef debug_USB
					debug("\r\nConfig. value:%lx - %lx",(uint32_t)(p>>32), (uint32_t)(p&0xFFFFFFFF));
				#endif
				CDC_Transmit_FS((uint8_t*)&p, 8);
			break;

			case 0x05:	//Receive 64bit configuration data and store it in flash

				memcpy(&config_val, &usb_cmd[2], 8);

				#ifdef debug_USB
					debug("\r\nConfig_val: %d", config_val);
				#endif
				store_config_64(config_val);
				vTaskDelay(100);
				load_config(&cal, &gps_enabled, &lpwa_enabled, &octave, &RecTime, &LeqTime);
				#ifdef debug_USB
					debug("\r\nCal: %f, gps:%d, lwpa:%d, octave:%d, Rec:%d, Leq:%d", cal, gps_enabled, lpwa_enabled, octave, RecTime, LeqTime);
				#endif
			break;

			case 0x06:
				time.Hours = usb_cmd[2];
				time.Minutes = usb_cmd[3];
				time.Seconds = usb_cmd[4];
				date.WeekDay = usb_cmd[5];
				date.Year = usb_cmd[6];
				date.Month = usb_cmd[7];
				date.Date = usb_cmd[8];
				HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BCD);
				HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BCD);

				#ifdef debug_USB
					vTaskDelay(100);
					HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
					HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
					sprintf((char *)currTimeBuff, "%02d/%02d/%02d %02d:%02d:%02d %04lu/%04lu", rtc_date.Date, rtc_date.Month, rtc_date.Year,
								rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, rtc_time.SubSeconds, rtc_time.SecondFraction);
					debug("\r\n%s", currTimeBuff);
					debug("\r\nDatetime Configuration EXIT!");
				#endif
			break;

			case 0x07:
				//TODO: Return data to validate erase ok
				WRITE_ENABLE();
				Chip_Erase();
				WRITE_DISABLE();
				#ifdef debug_USB
					debug("\r\nChip_Erase");
				#endif
			break;

			case 0x08:
				S7022_RST();
				#ifdef debug_USB
					debug("\r\nLPWA Restart!");
				#endif
			break;

			default:
				debug("\r\nUnknown command!");
			break;
		}
	}
	/* USER CODE END task_usb */
}

void task_gnss(void *argument)
{
	#ifdef	debug_GNSS
		debug("\r\n[task_gnss]Task GNSS");
	#endif
	for(;;)
	{

		if (osSemaphoreAcquire(sem_gnssHandle, 15000) == osOK)
		{
			if(GNSS.valid)
			{
				M10ProcessPackets(GNSS.buffer_gps);
				GNSS.valid = false;

				if(GPS.lock > 0 && sync_rtc_time == -1 && GPS.date != 0)
				{
					RTC_TimeTypeDef sTime = {0};
					RTC_DateTypeDef sDate = {0};

					uint32_t hhmmss = GPS.utc_time;
					sTime.Hours = hhmmss / 10000;
					sTime.Minutes = (hhmmss / 100) % 100;
					sTime.Seconds = hhmmss % 100;

					uint32_t ddmmyy = GPS.date;
					sDate.Date = ddmmyy / 10000;
					sDate.Month = (ddmmyy / 100) % 100;
					sDate.Year = ddmmyy % 100;

					if(HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK)
					{
						if(HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK)
						{
							sync_rtc_time = 1;
							#ifdef debug_GNSS
								debug("\r\n[task_gnss] RTC OK: %02d/%02d/20%02d %02d:%02d:%02d UTC\r\n",
									  sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds);
							#endif
						}
					}
				}

			#ifdef debug_GNSS
				debug("\r\n[task_gnss] %s", GNSS.buffer_gps);
				#if debug_GNSS==1
					sprintf((char *)tmp_gps, "TIME:%lu.%lu\r\n[Lat:%f\tLon:%f]\r\nLock:%u Sat:%u\r\n\r\n",
							GPS.utc_time, GPS.utc_time_ms, GPS.lat_dec, GPS.lon_dec, GPS.lock, GPS.satellites);
					debug((const char *)tmp_gps, 60);
				#endif
			#endif
			}else
			{
				#ifdef	debug_GNSS
					debug("\r\n[task_gnss]GNSS not valid!\r\n");
				#endif
			}
		}
		else
		{
			if(start == true)
			{
			    debug("\r\n[task_gnss] TIMEOUT! GNSS Hard Reset...\r\n");

			    HAL_UART_DMAStop(&hlpuart1);
			    HAL_UART_DeInit(&hlpuart1);

			    HAL_GPIO_WritePin(GPS_NRST_GPIO_Port, GPS_NRST_Pin, GPIO_PIN_RESET);
			    vTaskDelay(pdMS_TO_TICKS(100));
			    HAL_GPIO_WritePin(GPS_NRST_GPIO_Port, GPS_NRST_Pin, GPIO_PIN_SET);

			    vTaskDelay(pdMS_TO_TICKS(1500));

			    hlpuart1.Init.BaudRate = 9600;
			    if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
			        Error_Handler();
			    }

			    M10ChangeBaudrate(115200);
			    vTaskDelay(pdMS_TO_TICKS(100));
			    HAL_UART_DeInit(&hlpuart1);
			    hlpuart1.Init.BaudRate = 115200;
			    if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
			        Error_Handler();
			    }

			    M10GGAOnly();
			    vTaskDelay(pdMS_TO_TICKS(100));
			    //M10ZDA(1); Seems it is not necessary
			    //vTaskDelay(pdMS_TO_TICKS(100));

			    //TEST THIS HERE OR WHEN TIME SYNC
			    //M10PSMCT_RAM();

			    old_pos = 0;
			    parse_idx = 0;
			    memset(gnss_rx_buffer, 0, gnss_buff_len);
			    memset(buffer_gps_tmp1, 0, gnss_buff_len);

			    HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, gnss_rx_buffer, gnss_buff_len);
			    __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
			    sync_rtc_time = -1;
			}
		}
	}
}

void add_header(uint32_t base_address)
{
	uint32_t tmp_IMEI = atoi((const char *)IMEI);

	//GPS.lon_dec = 87.6543211;
	//GPS.satellites = 9;
	//GPS.lat_dec = 12.3456789;

	memcpy(&buff_addr_acq[base_address - n_bytes_header], &seconds, 4);
	memcpy(&buff_addr_acq[base_address - n_bytes_header + 4], &GPS.lon_dec, 4);
	buff_addr_acq[base_address - n_bytes_header + 8] = 0xff&tmp_IMEI>>12;	//20 - 8 bits
	buff_addr_acq[base_address - n_bytes_header + 9] = 0xff&tmp_IMEI>>4;	//12 - 8 bits
	buff_addr_acq[base_address - n_bytes_header + 10] = ((0x0f&tmp_IMEI)<<4) | (0x0f&GPS.satellites);

	buff_addr_acq[base_address - n_bytes_header + 11] =  0xff & ((uint8_t)(GPS.lat_dec));
	buff_addr_acq[base_address - n_bytes_header + 12] =  0xff & (((uint32_t)(fmodf(GPS.lat_dec, 1.0)*1000000))>>12);
	buff_addr_acq[base_address - n_bytes_header + 13] =  0xff & (((uint32_t)(fmodf(GPS.lat_dec, 1.0)*1000000))>>4);
	buff_addr_acq[base_address - n_bytes_header + 14] =  (0x0f & ((uint32_t)(fmodf(GPS.lat_dec, 1.0)*1000000))<<4)
																	| ((0x07&GPS.lock)<<1) | (0x01&LeqTime);
}

void encode(float32_t LeqVal, float32_t *TOBdata)
{
	// Variables estáticas para recordar los valores enteros previos entre llamadas
	static int prev_tob_ints[24]; // 23 para TOB + buffer
	static int prev_leq_int;

	// Nuevo cálculo de dirección base con Delta Coding
	uint32_t base_address = n_bytes_header;
	if (n_meas > 0) {
		// La primera muestra ocupa 33 bytes, las siguientes 27 bytes (216 bits)
		base_address = n_bytes_header + 33 + ((n_meas - 1) * 27);
	}

	// Update initial datetime into sending buffer header
	if (n_meas == 0)
		add_header(base_address); // Asume que add_header usa correctamente n_bytes_header (15)

	if(octave)
	{
		if (n_meas == 0)
		{
			// ====================================================================
			// MUESTRA 0: CÓDIGO ORIGINAL (BASE, 33 BYTES)
			// ====================================================================
			// Decimal part (First done outside loop for even number of filters
			buff_addr_acq[base_address] = (uint8_t)( (0x0f&((uint8_t)(fmodf(TOBdata[0], 1.0)*10)))<<4
									| (0x0f&((uint8_t)(fmodf(TOBdata[1], 1.0)*10))));

			for(int ii=1; ii<((NFilters-1)/2); ii++)
				buff_addr_acq[(base_address)+ii] = (uint8_t)( (0x0f&((uint8_t)(fmodf(TOBdata[ii*2], 1.0)*10)))<<4
									| (0x0f&((uint8_t)(fmodf(TOBdata[ii*2 + 1], 1.0)*10))));

			buff_addr_acq[base_address + 11] = (uint8_t)((0x0f&((uint8_t)(fmodf(TOBdata[NFilters-1], 1.0)*10)))<<4);

			// Integer part
			buff_addr_acq[base_address + 11] = ( (0xf0&buff_addr_acq[base_address + 11]) | ((0x7f&((uint8_t)TOBdata[0]))>>3) );

			uint8_t ctr_bitwise = 5, ctr_TOB = 0;
			for(int ii=0; (ii+ctr_TOB+1)<NFilters; ii++)
			{
				buff_addr_acq[(base_address + 12) + ii] = ( ((0x7f&((uint8_t)TOBdata[ii+ctr_TOB]))<<(ctr_bitwise)) |
										((0x7f&((uint8_t)TOBdata[ii+ctr_TOB+1]))>>(7-ctr_bitwise)));
				ctr_bitwise++;
				if(ctr_bitwise>7)
				{
					ctr_bitwise = 1;
					ctr_TOB++;
				}

			}
			buff_addr_acq[(base_address + 31)] = (0x7f&((uint8_t)TOBdata[NFilters-1]))<<3;
			buff_addr_acq[(base_address + 31)] = buff_addr_acq[(base_address + 31)] | ((0x7f&((uint8_t)LeqVal))>>4);
			buff_addr_acq[(base_address + 32)] = ((0x7f&((uint8_t)LeqVal))<<4) | (0x0f&((uint8_t)(fmodf(LeqVal, 1.0)*10)));

			// GUARDAR REFERENCIAS PARA LAS SIGUIENTES MUESTRAS DELTAS
			for(int i = 0; i < NFilters; i++) {
				prev_tob_ints[i] = (int)TOBdata[i];
			}
			prev_leq_int = (int)LeqVal;
		}
		else
		{
			// ====================================================================
			// MUESTRAS 1 a 7: COMPRESIÓN DELTA (27 BYTES)
			// ====================================================================

			// 1. Empaquetar las fracciones (igual que el original, toman 11.5 bytes)
			buff_addr_acq[base_address] = (uint8_t)( (0x0f&((uint8_t)(fmodf(TOBdata[0], 1.0)*10)))<<4
									| (0x0f&((uint8_t)(fmodf(TOBdata[1], 1.0)*10))));

			for(int ii=1; ii<((NFilters-1)/2); ii++)
				buff_addr_acq[(base_address)+ii] = (uint8_t)( (0x0f&((uint8_t)(fmodf(TOBdata[ii*2], 1.0)*10)))<<4
									| (0x0f&((uint8_t)(fmodf(TOBdata[ii*2 + 1], 1.0)*10))));

			buff_addr_acq[base_address + 11] = (uint8_t)((0x0f&((uint8_t)(fmodf(TOBdata[NFilters-1], 1.0)*10)))<<4);

			// 2. Calcular los 24 Deltas (23 TOB + 1 Leq) con Clipping de -16 a 15
			int8_t deltas[24];
			for(int i = 0; i < NFilters; i++) {
				int delta = (int)TOBdata[i] - prev_tob_ints[i];
				if (delta > 15) delta = 15;
				if (delta < -16) delta = -16;
				deltas[i] = delta;
				prev_tob_ints[i] += delta; // Actualizar con el delta real transmitido
			}
			int delta_leq = (int)LeqVal - prev_leq_int;
			if (delta_leq > 15) delta_leq = 15;
			if (delta_leq < -16) delta_leq = -16;
			deltas[NFilters] = delta_leq;
			prev_leq_int += delta_leq;

			// 3. Escribir los 120 bits de deltas (5 bits) y los 4 bits del Leq_frac
			// Bit_cursor marca la posición absoluta del bit de inicio. Las fracciones ya tomaron 92 bits.
			uint32_t bit_cursor = (base_address * 8) + 92;

			// a) Escribir los 24 deltas (120 bits)
			for(int i = 0; i < NFilters + 1; i++) {
				uint8_t val5 = deltas[i] & 0x1F; // Garantizar solo 5 bits (complemento a 2 automático)
				for (int b = 4; b >= 0; b--) {   // MSB primero
					uint8_t bit = (val5 >> b) & 1;
					uint32_t byte_idx = bit_cursor / 8;
					uint8_t bit_idx = 7 - (bit_cursor % 8);

					if (bit) buff_addr_acq[byte_idx] |= (1 << bit_idx);
					else     buff_addr_acq[byte_idx] &= ~(1 << bit_idx);

					bit_cursor++;
				}
			}

			// b) Escribir la fracción del Leq (4 bits finales)
			uint8_t leq_frac = (uint8_t)(fmodf(LeqVal, 1.0)*10) & 0x0F;
			for (int b = 3; b >= 0; b--) {
				uint8_t bit = (leq_frac >> b) & 1;
				uint32_t byte_idx = bit_cursor / 8;
				uint8_t bit_idx = 7 - (bit_cursor % 8);

				if (bit) buff_addr_acq[byte_idx] |= (1 << bit_idx);
				else     buff_addr_acq[byte_idx] &= ~(1 << bit_idx);

				bit_cursor++;
			}
		}
	}
	else
	{
		//TODO:Implement when no octave bands are sent
	}

	n_meas++;
	if(n_meas >= block_payloads)
	{
		n_meas = 0;
		if(buff_addr_acq == &leqs_buffer_acq1[0])
		{
			buff_addr_acq = &leqs_buffer_acq2[0];
			buff_addr_snd = &leqs_buffer_acq1[0];
		}
		else
		{
			buff_addr_acq = &leqs_buffer_acq1[0];
			buff_addr_snd = &leqs_buffer_acq2[0];
		}
		//task_state = DOUT;
		/* Trigger data storage/sending task directly when the block is full */
		osSemaphoreRelease(sem_send_dataHandle);
	}

	//TODO: Do the rtc update in a different time
	if((RecTime != 0) && (rtc_read()>=endTime))
		stopSampling();

	#ifdef debug_Leq
		//TODO: There still some long-term time change that should be addressed, probably waiting 1ms when 254/255
		HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
		//HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
		char buffer[32];
		strftime(buffer, 32, "%d/%m/%y %T", localtime(&seconds));

		debug("\r\nLeq: %04.2f \t %s %04lu/%04lu",	LeqVal, buffer,
								rtc_time.SubSeconds, rtc_time.SecondFraction);

		#if debug_Leq==-10//0
			//TODO: Seems this changes RTC prescaler
			//Debug TOB
			debug("\r\n");
			/*for(int ii = 0; ii<NFilters_low; ii++)
			{
				sprintf((char *)tmp_leq,"%03d.%01d | ", (int)TOBdata[ii], (int)(fmodf(TOBdata[ii], 1.0)*10));
				debug((const char*) &tmp_leq, 13);
			}*/
			for(int8_t jj=0; jj<NLeqSlow; jj++)
			{
				debug("[%d]", jj);
				for(int ii = 0; ii<NFilters; ii++)
				{
					//sprintf((char *)tmp_leq,"%03d.%01d | ", (int)FastTOB[ii][jj], (int)(fmodf(FastTOB[ii][jj], 1.0)*10));
					sprintf((char *)tmp_leq,"%03.1f | ", FastTOB[ii][jj]);
					debug((const char*) &tmp_leq, 13);
				}
				debug("\r\n");
			}
			//debug("\r\n\r\n");
		#endif
		#if debug_Leq==1
			//TODO: Seems this changes RTC prescaler
			//Debug TOB
			debug("\r\n");
			for(int ii = 0; ii<NFilters; ii++)
			{
				sprintf((char *)tmp_leq,"%03d.%01d | ", (int)TOBdata[ii], (int)(fmodf(TOBdata[ii], 1.0)*10));
				debug((const char*) &tmp_leq, 13);
			}
		debug("\r\n\r\n");
		//tcount++;
		#endif
		#if debug_Leq==2
			//DEBUG
			//debug("\r\nIMEI: %06u", (uint32_t) ( leqs_buffer[0]<<12 |  leqs_buffer[1]<<4 | ((0xf0&leqs_buffer[2])>>4)) );
			//debug("\r\nLeqTime: %04u - %04u", LeqTime, (uint16_t) ( (0x0f&leqs_buffer[2])<<8 |  leqs_buffer[3]) );
			//uint32_t sec_tmp = *(uint32_t *)&leqs_buffer[4], sec_orig = rtc_read();
			//debug("\r\nDatetime: %lu - %u", sec_orig, sec_tmp);
			//debug("\r\nLength: %lu - %u", n_meas, leqs_buffer[8]);

			//debug("\r\nLeq:%3.1f, %hu.%hu", LeqVal, (uint8_t) ( 0x7f&leqs_buffer[55]>>1),
					//(uint8_t) ( ((0x01&leqs_buffer[55])<<3) | ((0xe0&leqs_buffer[56])>>5) ));

			//debug("\r\nDATA: ");
				//for(int ii=0;ii<57;ii++)
					//debug("%02x", leqs_buffer[ii]);

			//debug("\r\n");*/
		#endif
	#endif

	//TODO: Why modifies prescaler?
	//Alaready commented out
	#ifdef LOW_POWER
		//-- Power saving --//
		if((USB_plugged == 0) & (LPWA.status !=2))
		{
			HAL_SuspendTick();
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		}
	#endif
}

void stopSampling()
{
	start=false;
	HAL_SAI_DMAStop(&hsai_BlockA1);
	HAL_GPIO_WritePin(GPS_NRST_GPIO_Port, GPS_NRST_Pin, GPIO_PIN_RESET);
	//debug("\r\nStop\r\n");   //Debug stop
	green_led(GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

}

void startSampling()
{
	start = true;
	n_meas = -1;
	tcount = 0;
	if(gps_enabled)
	{
		HAL_GPIO_WritePin(GPS_NRST_GPIO_Port, GPS_NRST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPS_NRST_GPIO_Port, GPS_NRST_Pin, GPIO_PIN_SET);
		vTaskDelay(1000);
		M10GGAOnly();
		vTaskDelay(100);
		M10ZDA(1); // Activate date-time string
		vTaskDelay(100);

		M10ChangeBaudrate(115200);
		HAL_UART_DeInit(&hlpuart1);
		hlpuart1.Init.BaudRate = 115200;
		if (HAL_UART_Init(&hlpuart1) != HAL_OK)
		{
			#ifdef debug_GNSS
				debug("\r\nError baudrate change");
			#endif
		}

		/* Stop active DMA transfers before modifying hardware registers */
		HAL_UART_DMAStop(&hlpuart1);

		/* Configure DMA for continuous Circular Mode */
		hlpuart1.hdmarx->Init.Mode = DMA_CIRCULAR;
		if (HAL_DMA_Init(hlpuart1.hdmarx) != HAL_OK)
		{
		    Error_Handler();
		}

		/* Reset buffer tracker and start Idle-Line detection */
		old_pos = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, gnss_rx_buffer, gnss_buff_len);

		/* Disable the Half-Transfer (HT) interrupt */
		__HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);

		uint32_t seconds_init =  rtc_read();

		while((GPS.lock < 1) || (sync_rtc_time == -1))
		{
			uint32_t seconds =  rtc_read();
			#ifdef debug_GNSS
				debug("Waiting valid GNSS signal and RTC sync: %d - %d!\r\n", GPS.lock, seconds - seconds_init);
			#endif
			red_led(GPIO_PIN_SET);
			vTaskDelay(1000);
		}
		red_led(GPIO_PIN_RESET);
		M10ZDA(0);
		//M10PSMCT_BBR();
		//vTaskDelay(5000);
		vTaskDelay(100);
		M10PSMCT_RAM();
		//M10PSMOO_RAM();
		vTaskDelay(5000);
	}

	eeprom_init_queue();

	if(RecTime!=0)
	{
		char buffer[32];
		endTime =  rtc_read();
		strftime(buffer, 32, "%d/%m/%y %T", localtime(&endTime));
		endTime=endTime+3600*RecTime;
		strftime(buffer, 32, "%d/%m/%y %T", localtime(&endTime));
	}

	LeqFastCount=0;   //Number of fast samples taken

	auxSave=0;
	sync_rtc_time = -1;

	seconds = rtc_read();

	if(HAL_SAI_Receive_DMA(&hsai_BlockA1, sampling_buffer, fast_125ms)!= HAL_OK)
	{
		#ifdef DEBUG
			debug("Error starting I2S module!\r\n");
		#endif
		stopSampling();
		return;
	}else
		__HAL_DMA_DISABLE_IT(&hdma_sai1_a, DMA_IT_HT);	//Disable half completed DMA SAI interrupt

	#ifdef DEBUG
		debug("*[startSampling] Started!\r\n");
	#endif
}

void store_config_64(uint64_t config_data)
{
	uint8_t ctr = 0;
	HAL_StatusTypeDef stat = HAL_OK;
	if (HAL_FLASH_Unlock()!= HAL_OK)
			debug("\r\nError unlocking memory...");
	else
		debug("\r\nMEMORY UNLOCKED!");
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	//EraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE;
	EraseInitStruct.NbPages = 1;
	EraseInitStruct.Page	= 252;
	EraseInitStruct.Banks = FLASH_BANK_1;
	vTaskDelay(150);
	stat = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) ;
	while ((stat!= HAL_OK) & (ctr<3))
	{
		debug("\r\nError clearing memory...");
		ctr++;
		vTaskDelay(150);
		stat = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) ;
	}
	if (stat==HAL_OK)
		debug("\r\nMEMORY ERASED!");

	vTaskDelay(150);
	ctr=0;
	stat = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ADDR_FLASH_PAGE_252,  config_data);
	while ( (stat != HAL_OK) & (ctr<3))
	{
		debug("\r\nError writing config (%d)...", stat);
		ctr++;
		vTaskDelay(150);
		stat = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ADDR_FLASH_PAGE_252,  config_data);
	}

	if (stat==HAL_OK)
		debug("\r\nCONFIG STORED!");

	HAL_FLASH_Lock();

	debug("\r\nStore_config_64 finished!");
}

void store_config(float32_t cal_loc, uint8_t gps_loc, uint8_t lpwa_loc,
		uint8_t octave_loc, uint16_t RecTime_loc, uint16_t LeqTime_loc)
{
	uint64_t data_out = ((*(uint64_t *)&cal_loc)<<32);
	uint32_t tmp = (gps_loc<<31) | ((lpwa_loc&0x1)<<30) | ((octave_loc&0x1)<<29)
				| ((RecTime_loc&0x1FFF)<<16) | ((LeqTime_loc&0xFFF)<<4);
	data_out |=tmp;
	store_config_64(data_out);
	////debug("\r\nstore_config Done!");
}

void load_config(float32_t *cal_loc, uint8_t *gps_loc, uint8_t *lpwa_loc,
			uint8_t *octave_loc, uint16_t *RecTime_loc, uint16_t	*LeqTime_loc)
{
	Address =ADDR_FLASH_PAGE_252;
	uint64_t *p = (uint64_t *)ADDR_FLASH_PAGE_252;
	////debug("\r\nload_config: %lx - %lx",(uint32_t)((*p)>>32), (uint32_t)((*p)&0xFFFFFFFF));
	uint32_t pt1 = (*p)>>32;
	uint32_t pt2 = (uint32_t)((*p)&0xFFFFFFFF);
	*cal_loc = *(float32_t*)&pt1;
	*gps_loc = (pt2>>31);
	*lpwa_loc = (pt2>>30)&0x01;
	*octave_loc = (pt2>>29)&0x01;
	*RecTime_loc = ((pt2>>16)&0x1FFF);
	*LeqTime_loc = ((pt2>>4)&0xFFF);
	////debug("\r\nload_config Done!");
}

void SystemClock_Config_LOW_IIR(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Configure LSE Drive Capability
	  */
	  HAL_PWR_EnableBkUpAccess();
	  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
	                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	  RCC_OscInitStruct.HSICalibrationValue = 64;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = 2;
	  RCC_OscInitStruct.PLL.PLLN = 29;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV29;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
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
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Enables the Clock Security System
	  */
	  HAL_RCC_EnableCSS();
}

void SystemClock_Config_LOW(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
			|RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 64;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 3;
	RCC_OscInitStruct.PLL.PLLN = 18;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV12;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();

}

void SystemClock_Config_HIGH(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 19;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV19;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	debug("\r\nWakeup timer");
	__HAL_GPIO_EXTI_CLEAR_IT(PB_REC_Pin);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_RTCEx_DeactivateWakeUpTimer(hrtc);
}

/**
  * @brief  UART error callback to handle hardware glitches (like Overrun errors).
  * This prevents the GNSS and LPWA peripherals from freezing permanently.
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        HAL_UART_DMAStop(huart);

        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);

    }
    else if (huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_DMAStop(huart);
        if(LPWGNS_config == true) {
            HAL_UART_Receive_DMA(huart, buffer_rx, 1);
        } else {
            HAL_UART_Receive_DMA(huart, &buffer_lpwa[counter_lpwa], 1);
        }
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_task_main */
/**
  * @brief  Function implementing the main_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_task_main */
void task_main(void *argument)
{
  /* USER CODE BEGIN 5 */
	debug("\r\n****STARTING...(TEST)!****");
	HAL_GPIO_WritePin(LPWA_NRST_GPIO_Port, LPWA_NRST_Pin, GPIO_PIN_RESET);

	//store_config(cal, gps_enabled, lpwa_enabled, octave, RecTime, LeqTime);

	load_config(&cal, &gps_enabled, &lpwa_enabled, &octave, &RecTime, &LeqTime);

	//LeqTime = 0;
	//gps_enabled = false;
	//lpwa_enabled = false;

	if (lpwa_enabled == true && LeqTime > 1)//Online mode only 125ms/1s
		LeqTime = 1;

	/*WRITE_ENABLE();
	Chip_Erase();
	WRITE_DISABLE();
	#ifdef debug_USB
		debug("\r\nChip_Erase");
	#endif*/

	eeprom_init_queue();
	debug("\r\nData_len: %d", get_eeprom_data_len());

	debug("\r\nCal: %f, gps:%d, lwpa:%d, octave:%d, Rec:%d, Leq:%d", cal,
			gps_enabled, lpwa_enabled, octave, RecTime, LeqTime);

	MAIN_Queue_init(&BUFF_main, (uint8_t **) &leqs_buffer_main[0][0], block_len + 1, main_buffer_size);
	MAIN_Queue_init(&BUFF_usb, (uint8_t **) &USB_buff_msgs, usb_buff_len + 1, usb_max_n_msg);
	MAIN_Queue_init(&BUFF_LPWA, (uint8_t **) &LPWA_buff, LPWA_rx_len, max_blocks_LPWA);

	#ifdef NO_MIC
		for (int ii = 0; ii < fast_125ms; ii++)
		{
			sine_working_buffer[ii] = 1000 * arm_sin_f32(out_counter);

			out_counter += delta;
			if (out_counter > 2 * M_PI)
				out_counter -= 2 * M_PI;
			////debug("%f, ", working_buffer[ii]);
		}
	#endif

	if (lpwa_enabled == true)
		IoT_taskHandle = osThreadNew(task_IoT, NULL, &IoT_task_attributes);
	else
		offline_taskHandle = osThreadNew(task_offline, NULL, &offline_task_attributes);

	/* Infinite loop */

	//HAL_LPTIM_Counter_Start_IT(&hlptim1, 797);	//DEBUG - 0.01 ms
	//TODO:Find out why this helps to start with low current consumption
	vTaskDelay(500);
	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
	{
		debug("\r\nUSB (%d, %d)",  hUsbDeviceFS.ep0_state, hUsbDeviceFS.dev_state);
		usb_taskHandle = osThreadNew(task_usb, NULL, &usb_task_attributes);
		USB_plugged = 1;

	}
	green_led(GPIO_PIN_SET);
	vTaskDelay(1000);
	osSemaphoreAcquire(sem_initHandle, osWaitForever);
	debug("\r\nSYSTEM READY!\r\n");

	#ifdef LOW_POWER
		////------ Power saving ---////
		if((USB_plugged == 0) & (LPWA.status !=2))
		{
			vTaskDelay(1000);
			HAL_SuspendTick();
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		}
	#endif

		// --- DYNAMIC IMEI EXTRACTION ON BOOT ---
	if (lpwa_enabled == true)
	{
		//Wake up the modem to request the IMEI
		LPWA_enable();

		// Enable data queuing for AT commands
		lpwa_flag_rx = true;

		if(S7022_CGSN(IMEI, 500) == osOK)
		{
			#ifdef debug_LPWA
				debug("\r\n[task_main] IMEI Extracted: %s\r\n", IMEI);
			#endif
		}
		else
		{
			#ifdef debug_LPWA
				debug("\r\n[task_main] IMEI extraction failed. Using fallback.\r\n");
			#endif
			memcpy(IMEI, "000000", 6);
		}

		// Disable data queuing after transaction
		lpwa_flag_rx = false;

		// 2. If Np dictates sleeping mode, turn the modem off immediately
		if((n_blocks_LPWA > n_blocks_LPWA_limit) && (USB_plugged == 0))
		{
			LPWA_disable();
		}
	}
	else
	{
		// 3. Complete offline fallback: Use STM32 Hardware Unique ID
		// Address 0x1FFF7590 is the standard UID_BASE for STM32L4
		uint32_t mcu_uid = *(uint32_t *)0x1FFF7590;
		sprintf((char *)IMEI, "%06lu", mcu_uid % 1000000);

		#ifdef debug_LPWA
			debug("\r\n[task_main] Offline mode. MCU UID used as IMEI: %s\r\n", IMEI);
		#endif
	}

	/*vTaskDelay(2000);
	debug("\r\nStarting LPWA");
	LPWA_enable();
	lpwa_flag_rx = true;	//Enable LPWA data queuing
	while(S7022_CEREG(500, 10)!= osOK){}
	lpwa_flag_rx = false;	//Enable LPWA data queuing
	LPWA_disable();
	debug("\r\nLPWA stopped");

	vTaskDelay(2000);
	debug("\r\nStarting LPWA");
	LPWA_enable();
	lpwa_flag_rx = true;	//Enable LPWA data queuing
	while(S7022_CEREG(500, 10)!= osOK){}
	lpwa_flag_rx = false;	//Enable LPWA data queuing
	LPWA_disable();
	debug("\r\nLPWA stopped");

	vTaskDelay(2000);
	debug("\r\nStarting LPWA");
	LPWA_enable();
	lpwa_flag_rx = true;	//Enable LPWA data queuing
	while(S7022_CEREG(500, 10)!= osOK){}
	lpwa_flag_rx = false;	//Enable LPWA data queuing
	LPWA_disable();
	debug("\r\nLPWA stopped");*/

	green_led(GPIO_PIN_RESET);
	//startSampling();
	for (;;)
	{
		osSemaphoreAcquire(sem_start_stopHandle, osWaitForever);
		HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 614, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
		if (start == false)
			startSampling();
		else
			stopSampling();

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task_signal_processing */
/**
* @brief Function implementing the signal_processi thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_signal_processing */
void task_signal_processing(void *argument)
{
  /* USER CODE BEGIN task_signal_processing */
	const float32_t c = 2.0/(32000*4000);	// c = 2/(Fs*N) N-> does not take into account zero padding
	const float32_t c_low = 2.0/(4000*4096);
	const float32_t delta_f = Fs/4096.0;
	const float32_t delta_f_low = (Fs/8.0)/4096.0;

	float32_t auxF = 0.0;
	/* Infinite loop */

	debug("\r\nV 1.1 \r\n");

	arm_biquad_cascade_df2T_init_f32(&iirDec8, iir_order_dec_o06_f08, sos_dec_o06_f08, iir_dec_state8);
	arm_rfft_fast_init_f32(&fft_inst, fast_125ms + 96);

	/* Flag to synchronize georeferencing task with a safe offset */
	uint8_t georef_pending = 0;

	for(;;)
	{
		//osSemaphoreAcquire(sem_125Handle, osWaitForever);
		osSemaphoreAcquire(sem125_countingHandle, osWaitForever);

		//TODO: Move this out of task_signal_processing
		if (start == true && hlpuart1.RxState == HAL_UART_STATE_READY)
		{
			__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
			old_pos = 0;
			parse_idx = 0;
			HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, gnss_rx_buffer, gnss_buff_len);
			__HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
		}

		#ifdef NO_MIC
			//To work without mic
			arm_mean_f32(sine_working_buffer, fast_125ms, &auxF);
			arm_offset_f32(sine_working_buffer, -auxF, working_buffer, fast_125ms);

			//memcpy(working_buffer, audio, fast_125ms*4);

		#else
			for(int ii=0;ii<fast_125ms;ii++)	//TODO: A bias observed (7300)
							working_buffer[ii] = (float32_t)(((int32_t) (~ ((sampling_buffer[ii*4+2]&0xFF)<<10 | (sampling_buffer[ii*4+1]&0xFF)<<2
															| (sampling_buffer[ii*4]&0xC0)>>6) + 1 ) <<14 ) >>14) - 7980;
						//DC removal
						//arm_mean_f32(outFFT, fast_125ms, &auxF);
						//arm_offset_f32(outFFT, -auxF, working_buffer, fast_125ms);
						//memcpy(working_buffer, &data[audio_ctr*fast_125ms], fast_125ms*4);
		#endif

		/////// ****** PSD BASED ****** //////////
		if(octave)
		{
			float32_t sum = 0;

			//Decimate and accumulate samples
			#ifndef IIR_DEBUG
				arm_biquad_cascade_df2T_f32Mod2(&iirDec8, working_buffer, outFFT, fast_125ms); //Filter

				for(int jj = 0; jj<fast_125ms; jj+=8)
					acc_dec_buffer[jj/8] = outFFT[jj]*win_coeff_dec[jj/8];


				memset(&acc_dec_buffer[(int)(fast_125ms/8.0)], 0, (int)((4096 - fast_125ms/8.0)*4));
				arm_rfft_fast_f32(&fft_inst, acc_dec_buffer, outFFT, 0);
				arm_cmplx_mag_squared_f32(outFFT, acc_dec_buffer, 2048);
				arm_mult_f32(acc_dec_buffer, compCoeff_LF, outFFT, 2048);	//Mic frequency compensation
				outFFT[0] = 0;
				for(int ii = 0; ii<NFilters_low;ii++)
				{
					sum = 0;
					for(int jj = freq_idxs_LF[ii]; jj<freq_idxs_LF[ii+1]; jj++)
						sum+=outFFT[jj]*delta_f_low;
					LeqFastTOB[ii][LeqFastCount] = 10*log10f_fast(c_low*sum) + cal_def_LF + 13.5 + cal;	//TODO: Explain why this difference
				}

				//PSD
				memset(&working_buffer[fast_125ms], 0, 96*4);
				arm_rfft_fast_f32(&fft_inst, working_buffer, outFFT, 0);
				arm_cmplx_mag_squared_f32(outFFT, working_buffer, 2048);
				arm_mult_f32(working_buffer, compCoeff, outFFT, 2048);	//Mic frequency compensation
				sum = 0;
				for(int jj = 1; jj<2048; jj++)
					sum+=outFFT[jj]*delta_f*ACoeff[jj];//A weighting
				LeqFastSamples[LeqFastCount]= 10*log10f_fast(c*sum) + cal_def + cal;

				//OTOB Higher frequencies
				outFFT[0] = 0;
				for(int ii = NFilters_low; ii<NFilters;ii++)
				{
					sum = 0;
					for(int jj = freq_idxs[ii]; jj<freq_idxs[ii+1]; jj++)
						sum+=outFFT[jj]*delta_f;
					LeqFastTOB[ii][LeqFastCount] = 10*log10f_fast(c*sum) + cal_def + cal;

				}

				if((LeqFastCount+1)>=NLeqSlow)  //Compute 1s samples
				{
					for(int ii=0; ii<NFilters; ii++)
						LeqSecondsTOB[ii][0] = Leq(LeqFastTOB[ii], NLeqSlow);
				}
		#else
				/////// ****** FILTER BASED ****** //////////
				uint32_t N_dec = 1;
				//7.9 khz, 10 khz
				/*for(int ss=0; ss<n_filters_high_order; ss++)
				{
					arm_biquad_cascade_df2T_init_f32(&iirS, iir_order+1, &sos4[ss*((iir_order+1)*5)], iir_state);
					arm_biquad_cascade_df2T_f32Mod2(&iirS, working_buffer, outFFT, fast_125ms); //Filter
					arm_power_f32(outFFT, fast_125ms, &Pxx);
					Pxx = sqrtf(Pxx/fast_125ms);
					LeqFastTOB[(NFilters - n_filters_high_order) + ss][LeqFastTOBCount] = 20*log10f_fast(Pxx) + cal;
				}

				//6.3 khz - 63 hz
				for(int ss=(NFilters - n_filters_high_order - 1); ss >= 4; ss--)
				{
					arm_biquad_cascade_df2T_init_f32(&iirS, iir_order, &sos3[((ss-1)%3)*(iir_order*5)], iir_state);
					arm_biquad_cascade_df2T_f32Mod2(&iirS, working_buffer, outFFT, fast_125ms/N_dec); //Filter
					arm_power_f32(outFFT, fast_125ms/N_dec, &Pxx);
					Pxx = sqrtf(Pxx/(fast_125ms/N_dec));
					LeqFastTOB[ss][LeqFastTOBCount] = 20*log10f_fast(Pxx) + cal;

				}

				LeqFastTOBCount++;
				if(LeqFastTOBCount>=8)  //Compute 1s samples
				{

					for(int ss=3; ss >= 1; ss--)
					{
						arm_biquad_cascade_df2T_init_f32(&iirS, iir_order, &sos3[((ss-1)%3)*(iir_order*5)], iir_state);
						arm_biquad_cascade_df2T_f32Mod2(&iirS, aux_low_freqs[ss], outFFT, 248); //Filter
						arm_power_f32(outFFT, 248, &Pxx);
						Pxx = sqrtf(Pxx/248);
						LeqSecondsTOB[ss][LeqSecondsCount-1] = 20*log10f_fast(Pxx) + cal;
					}

					//25.1 Hz Filter
					arm_biquad_cascade_df2T_init_f32(&iirS, iir_order, &sos3[2*(iir_order*5)], iir_state);
					arm_biquad_cascade_df2T_f32Mod2(&iirS, aux_low_freqs[0], outFFT, 120); //Filter
					arm_power_f32(outFFT, 120, &Pxx);
					Pxx = sqrtf(Pxx/120);
					LeqSecondsTOB[0][LeqSecondsCount-1] = 20*log10f_fast(Pxx) + cal;

					LeqFastTOBCount=0;
					for(int i=4;i<NFilters;i++)
						LeqSecondsTOB[i][LeqSecondsCount-1] = Leq(LeqFastTOB[i], NLeqSlow);
					osSemaphoreRelease(sem_georefHandle);
				}else
				{
					////------ Power saving ---////
					//HAL_SuspendTick();
					//HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
				}*/
				//// ****** IIR NOT OPTIMIZED ********** ////
				arm_biquad_cascade_df2T_init_f32(&iirS, NUM_SECTIONS, AFilterCoeff, iir_state);
				arm_biquad_cascade_df2T_f32Mod(&iirS, working_buffer, outFFT, fast_125ms, AFilterGains);	//TODO:Find out why Mod ¿?
				arm_power_f32(outFFT, fast_125ms,&Pxx);
				Pxx = sqrtf(Pxx/fast_125ms);
				Pxf = 20*log10f_fast(Pxx) + cal;// + 61.01580;
				LeqFastSamples[LeqFastCount]=Pxf;
				//LeqFastCount++;

				if(LeqFastCount>=8)
				{
					LeqFastCount=0;
					LeqSeconds[LeqSecondsCount]=Leq(LeqFastSamples, NLeqSlow);   //Compute 1s Leq and save it in global seconds var
					LeqSecondsCount++;  //Increase 1s sampling count
					if(octave == false)
						osSemaphoreRelease(sem_georefHandle);
				}

				if(octave)
				{
					for(int ss=0;ss<NFilters;ss++)
					{
						arm_biquad_cascade_df2T_init_f32(&iirS, NUM_SECTIONS, &thirdOctIIR[ss*initIRR], iir_state);
						arm_biquad_cascade_df2T_f32Mod(&iirS, working_buffer, outFFT, fast_125ms, &gains[ss*3]); //Filter
						arm_power_f32(outFFT, fast_125ms, &Pxx);
						Pxx = sqrtf(Pxx/fast_125ms);
						LeqFastTOB[ss][LeqFastTOBCount] = 20*log10f_fast(Pxx) + cal;
					}
				}
		#endif

		}else
		{

		}

		LeqFastCount++;
		if (LeqTime == 0)
		{
			//task_state = GEOREF;
			osSemaphoreRelease(sem_georefHandle);

			if (LeqFastCount >= 8)
			{
				LeqFastCount = 0;
				seconds++; //seconds = rtc_read();
				red_led(GPIO_PIN_SET);
			}
			else if (LeqFastCount == 4)
			{
				red_led(GPIO_PIN_RESET);
			}

			/*if (task_state == DOUT)
			{
				task_state = WAIT;
				osSemaphoreRelease(sem_send_dataHandle);
			}*/
		}
		else
		{
			if(LeqFastCount >= 8)
			{
			    LeqFastCount = 0;
			    seconds++;
			    /* Prevent buffer overflow in case of RTOS desynchronization */
				/* 60 is the maximum size defined for LeqSeconds array      */
				if (LeqSecondsCount >= 60)
				{
					LeqSecondsCount = 0;
				}
			    LeqSeconds[LeqSecondsCount] = Leq(LeqFastSamples, NLeqSlow);
			    if(octave)
			    {
			        for(int i = 0; i < NFilters; i++)
			        {

			            LeqSecondsTOB[i][LeqSecondsCount] = Leq(LeqFastTOB[i], NLeqSlow);
			        }
			    }

			    LeqSecondsCount++;

			    if(LeqSecondsCount >= LeqTime)
			    {
			        //task_state = GEOREF;
			    	/* Set flag to trigger georeferencing task safely in case 2 */
			    	georef_pending = 1;
			        //osSemaphoreRelease(sem_georefHandle);
			    } else {
			        #ifdef LOW_POWER
			            if((USB_plugged == 0) & (LPWA.status !=2))
			            {
			                HAL_SuspendTick();
			                HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			            }
			        #endif
			    }
			    red_led(GPIO_PIN_SET);
			} else
			{
				switch(LeqFastCount)
				{
					case 1:
						//To avoid overloading the first 125 ms
					break;

					case 2:
						/* Trigger georeferencing task with offset if pending */
						if(georef_pending == 1)
						{
							georef_pending = 0;
							osSemaphoreRelease(sem_georefHandle);
						}

						/*if(task_state == GEOREF)
						{
							task_state = WAIT;
							osSemaphoreRelease(sem_georefHandle);
						}*/else
						{
						#ifdef LOW_POWER
							////------ Power saving ---////
							if((USB_plugged == 0) & (LPWA.status !=2))
							{
								HAL_SuspendTick();
								HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
							}
						#endif
						}
					break;

					case 3:
						/*if(task_state == DOUT)
						{
							task_state = WAIT;
							osSemaphoreRelease(sem_send_dataHandle);
						}else
						{*/
							#ifdef LOW_POWER
								 ////------ Power saving ---////
								if((USB_plugged == 0) & (LPWA.status !=2))
								{
									HAL_SuspendTick();
									HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
								}
							#endif
						//}
					break;

					case 4:	//Half second led on
						red_led(GPIO_PIN_RESET);

					default:
						#ifdef LOW_POWER
							////------ Power saving ---////
							if((USB_plugged == 0) & (LPWA.status !=2))
							{
								HAL_SuspendTick();
								HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
							}
						#endif
					break;
				}
			}
		}
	}
  /* USER CODE END task_signal_processing */
}

/* USER CODE BEGIN Header_task_georeferencing */

/**
 * @brief Function implementing the georeferencing_ thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_task_georeferencing */
void task_georeferencing(void *argument)
{
  /* USER CODE BEGIN task_georeferencing */
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(sem_georefHandle, osWaitForever);
		//debug("\r\nGR");
		//HAL_GPIO_TogglePin(STATUS_P_GPIO_Port, STATUS_P_Pin);
		//HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
		auxSave++;

		if(LeqTime >= 1) // SLOW (1s or more)
		{
			// Energetic average calculation automatically handles 1 or more seconds
			float32_t final_Leq = Leq(LeqSeconds, LeqTime);

			for(int i = 0; i < NFilters; i++) {
				LeqAuxTOB[i] = Leq(LeqSecondsTOB[i], LeqTime);
			}

			encode(final_Leq, LeqAuxTOB);

			// CRITICAL FIX: Reset counter for the next accumulation cycle
			LeqSecondsCount = 0;
		}
		else if (LeqTime == 0) // FAST (125ms)
		{
			uint8_t idx = (LeqFastCount == 0) ? 7 : (LeqFastCount - 1);

			for(int i=0;i<NFilters;i++)
				LeqAuxTOB[i] = LeqFastTOB[i][idx];

			encode(LeqFastSamples[idx], LeqAuxTOB);
		}

		//TODO: Why modifies prescaler?
		#ifdef LOW_POWER
			////------ Power saving ---////
			/*if((USB_plugged == 0) & (LPWA.status !=2))
			{
				HAL_SuspendTick();
				HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			}*/
		#endif
	}
  /* USER CODE END task_georeferencing */
}

/* rst_timeout_LPWA function */
void rst_timeout_LPWA(void *argument)
{
  /* USER CODE BEGIN rst_timeout_LPWA */
	if(LPWA_enable()!=osOK)
	{
		LPWA.status = 3;
		HAL_GPIO_WritePin(RST_port, RST_pin, GPIO_PIN_RESET);
	}
	S7022_rst_all();
	#ifdef debug_LPWA
		debug("\r\nTimeout finished! LPWA.status: %d", LPWA.status);
	#endif
  /* USER CODE END rst_timeout_LPWA */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
