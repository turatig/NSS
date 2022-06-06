/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AMG8833.h"
#include "Step.h"
#include "Jstick.h"
#include "utils.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BIT_BAND_ALIAS_REGION_BASE 0x22000000
#define BIT_BAND_ADDR(offset,bit) (BIT_BAND_ALIAS_REGION_BASE + offset*32 + bit*4)

/*Thermal camera FSM (based on amg8833 chip) control byte offset in bit band region*/
#define AMG_CTRL_OSET 0x0
/*Control bits*/
#define AMG_RD_START  (*(volatile uint32_t*)BIT_BAND_ADDR(AMG_CTRL_OSET,0x7))//amg read start flag
#define AMG_RD_CPLT (*(volatile uint32_t*)BIT_BAND_ADDR(AMG_CTRL_OSET,0x6))//amg read complete
#define AMG_OUT_CPLT (*(volatile uint32_t*)BIT_BAND_ADDR(AMG_CTRL_OSET,0x5))//amg output (through uart) complete. Ready for another read

/*Motor control byte offset in bit band region*/
#define MOTOR_CTRL_OSET 0x1
#define MOTOR_MV (*(volatile uint32_t*)BIT_BAND_ADDR(MOTOR_CTRL_OSET,0x7))

/*Audio control byte offset in bit band region*/
#define AUDIO_CTRL_OSET 0x2
#define BUF1_CPLT (*(volatile uint32_t*)BIT_BAND_ADDR(AUDIO_CTRL_OSET,0x7))
#define BUF2_CPLT (*(volatile uint32_t*)BIT_BAND_ADDR(AUDIO_CTRL_OSET,0x6))
//#define OVR_THRS (*(volatile uint32_t*)BIT_BAND_ADDR(AUDIO_CTRL_OSET,0x5))

/*Push buttons control byte offset in bit band region*/
#define EXTI_BUT_CTRL_OSET 0x3
#define EXTI_BUT_PUSH (*(volatile uint32_t*)BIT_BAND_ADDR(EXTI_BUT_CTRL_OSET,0x7))

#define MODE_TOGGLE (*(volatile uint32_t*)BIT_BAND_ADDR(EXTI_BUT_CTRL_OSET,0x6))
#define LEFT_BUT_PUSH (*(volatile uint32_t*)BIT_BAND_ADDR(EXTI_BUT_CTRL_OSET,0x5))
#define RIGHT_BUT_PUSH (*(volatile uint32_t*)BIT_BAND_ADDR(EXTI_BUT_CTRL_OSET,0x4))


/*Audio buffer size*/
/*Half of the audio buffer is filled by the main playback and process loop*/
#define AUDIO_BUF_SZ 512
/*Used by the DMA as unit of transfer*/
#define AUDIO_TOT_BUF_SZ (AUDIO_BUF_SZ * 2)
/*
 * Cross-correlation buffer multiplying factor.
 * Tells how many times cross-correlation buffer is bigger than input audio buffer.
 */
#define XCOR_BUF_MULT 3
#define XCOR_BUF_SZ (AUDIO_BUF_SZ*XCOR_BUF_MULT)
/*Cross-correlation result array size*/
#define XCOR_SZ AUDIO_BUF_SZ*3
/*Number of last mean values are considered to compute a stable estimate of DC offset*/
#define DC_BUF_SZ 100

/*
 * Debug macros
 * remove comments to debug single components
 */
//#define DAC_DEBUG 1
//#define DEBUG_LOG 1
/*
 * Time benchmark macros
 */
//#define TIME_PREPROC 1
//#define TIME_CONV_CH1 1
//#define TIME_CONV_CH2 1
#define TIME_XCOR 1
//#define TIME_AUDIO_PROC 1
/*
* Set oscilloscope timer trigger
*/
#define CHRONO_START() GPIOE->ODR|=GPIO_PIN_11
/*
* Reset oscilloscope timer trigger
*/
#define CHRONO_STOP() GPIOE->ODR&=~GPIO_PIN_11



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;
DMA_HandleTypeDef hdma_dac2;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
/* Driver data structure to abstract sensors and their interactions*/

AMG8833 cam;
Step motor;
Jstick js;

/*Thermal image buffer*/
uint8_t img_buf[AMG8833_DS];
char msg_buf[128];
HAL_StatusTypeDef status;

/*
 * Audio input and output ping-pong buffers
 */
uint16_t audio_in_buf1[AUDIO_TOT_BUF_SZ];
uint16_t audio_out_buf1[AUDIO_TOT_BUF_SZ];

uint16_t audio_in_buf2[AUDIO_TOT_BUF_SZ];
uint16_t audio_out_buf2[AUDIO_TOT_BUF_SZ];


/*
 * Audio buffer pointers switched by the ISR relative to DMA transfer
 */
volatile uint16_t *audio_in_ptr1;
volatile uint16_t *audio_out_ptr1;

volatile uint16_t *audio_in_ptr2;
volatile uint16_t *audio_out_ptr2;

/*
 * Cross-correlation buffers.
 * These buffers will contain audio samples without DC offset
 */
int xcor_buf1[XCOR_BUF_SZ];
int xcor_buf2[XCOR_BUF_SZ];
int xcor_res[XCOR_BUF_SZ*2-1];

/*
 * Cross-correlation buffer offset.
 * This counter is incremented xcor_buf_oset=( xcor_buf_idx+1 ) % XCOR_BUF_MULT.
 * Remind that XCOR_BUF_SZ = AUDIO_BUF_SZ * XCOR_BUF_MULT
 */
uint8_t xcor_buf_oset;

/*
 * DC offset buffer to compute a stable estimate of DC offset of the two channels.
 * DC offset must be removed in order to have a zero mean signal useful to compute cross-correlation.
 * This buffer will store the last DC_OSET_BUF_SZ mean values computed on input buffers channel1-2
 */
uint16_t dc_buf_ch1[DC_BUF_SZ];
uint16_t dc_buf_ch2[DC_BUF_SZ];
uint16_t dc_buf_idx;

/*
 * Every time RMS of both input buffers happen to be above the threshold this counter is incremented(max 3).
 * Else will be reset to 0
 */
uint8_t ovr_thr_cnt;
/*
 * This flag is set by the preprocessing loop in SSL mode if RMS of signal in ch1-2 is above the threshold
 * Trigger sound localization event
 */
uint8_t ovr_thr;

/*
 * RMS above this threshold to trigger SSL event
 */
uint16_t threshold;
/*Mode bit set by the main loop*/
uint8_t mode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef DAC_DEBUG
	uint16_t square_wv[AUDIO_TOT_BUF_SZ];

	//Test DAC channels reproducing a square wave
	void testDAC(){
		for(int i=0; i<AUDIO_TOT_BUF_SZ; i++){
			if(i>AUDIO_BUF_SZ)
				square_wv[i]=0;
			else
				square_wv[i]=2048;
		}
		HAL_TIM_Base_Start_IT(&htim2);
		HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)square_wv,AUDIO_TOT_BUF_SZ,DAC_ALIGN_12B_R);
		HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_2,(uint32_t*)square_wv,AUDIO_TOT_BUF_SZ,DAC_ALIGN_12B_R);
		while(1){
			__NOP();
		}
	}
#endif


/*
 * Callback function to manage external interrupt push buttons pushed
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	if( !EXTI_BUT_PUSH ){
		EXTI_BUT_PUSH=1;
		//Start debounce timer: interrupt after 50 ms
		HAL_TIM_Base_Start_IT(&htim10);
	}
}
/*
 * Handler for thermal image DMA memory transfer cplt interrupt
 * Thermal image reading is now complete
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C1){
		AMG_RD_CPLT=1;
	}
}

/*
 * Handler for thermal image DMA memory transfer cplt interrupt
 * Thermal image output is now complete
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){
		AMG_OUT_CPLT=1;
	}
}

/*Manage thermal image I/O from AMG sensor to UART interface by checking bits set by:
 * Timer 6 ISR
 * DMA1 Stream 0 (Thermal image I2C Rx) Rx Cplt ISR
 * DMA1 Stream 6 (Thermal image USART2 Tx) Tx Cplt ISR
 */
void thermalImgFSM(){
	  /*
	   * Thermal camera FSM
	   */
	  //If timer6 has expired
	  if(AMG_RD_START){

		  //Command DMA transfer from amg8833
		status=amg8833ReadDMA(&cam,img_buf);
		if(status==HAL_OK)
			  //Clear ctrl bit
			  AMG_RD_START=0;
	   }

	  //If DMA image reading was successful
	  if(AMG_RD_CPLT){

		 //Command DMA transfer to uart2
		 status=HAL_UART_Transmit_DMA(&huart3,img_buf,AMG8833_DS);
		 if(status==HAL_OK){
			 AMG_RD_CPLT=0;
		 }
	  }
	  //if latest data were consumed in output, restart timer6
	  if(AMG_OUT_CPLT){
		  AMG_OUT_CPLT=0;
		  HAL_TIM_Base_Start_IT(&htim6);
	  }
}



/*
 * Handler for audio input DMA memory transfer half-cplt interrupt
 * AUDIO_BUF_SZ sample were converted and put into audio_in_buf.
 * Data can be moved by main application from lower audio_in_buf to higher audio_out_buf
 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){
	if(hadc->Instance==ADC1){

#ifdef TIME_CONV_CH1
		CHRONO_START();
#endif
		audio_in_ptr1=&audio_in_buf1[0];
		audio_out_ptr1=&audio_out_buf1[AUDIO_BUF_SZ];
		BUF1_CPLT=1;
	}

	if(hadc->Instance==ADC2){

#ifdef TIME_CONV_CH2
		CHRONO_START();
#endif
		audio_in_ptr2=&audio_in_buf2[0];
		audio_out_ptr2=&audio_out_buf2[AUDIO_BUF_SZ];
		BUF2_CPLT=1;
	}
}

/*
 * Handler for audio input DMA memory transfer half-cplt interrupt
 * AUDIO_TOT_BUF_SZ sample were taken from audio out buf and fed into DAC.
 * Data can be moved by main application from higher audio_in_buf to lower audio_out_buf
 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac){

#ifdef TIME_CONV_CH1
	CHRONO_STOP();
#endif
	audio_in_ptr1=&audio_in_buf1[AUDIO_BUF_SZ];
	audio_out_ptr1=&audio_out_buf1[0];
	BUF1_CPLT=1;
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac){

#ifdef TIME_CONV_CH2
	CHRONO_STOP();
#endif
	audio_in_ptr2=&audio_in_buf2[AUDIO_BUF_SZ];
	audio_out_ptr2=&audio_out_buf2[0];
	BUF2_CPLT=1;
}

/*
 * Handler for audio input DMA memory transfer half-cplt interrupt
 * AUDIO_TOT_BUF_SZ sample were taken from audio out buf and fed into DAC.
 * Data can be moved by main application from higher audio_in_buf to lower audio_out_buf (SSL mode only)
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(mode){
		if(hadc->Instance==ADC1){

#ifdef TIME_CONV_CH1
			CHRONO_STOP();
#endif
			audio_in_ptr1=&audio_in_buf1[AUDIO_BUF_SZ];
			//audio_out_ptr1=&audio_out_buf1[0];
			BUF1_CPLT=1;
		}

		if(hadc->Instance==ADC2){

#ifdef TIME_CONV_CH2
			CHRONO_STOP();
#endif
			audio_in_ptr2=&audio_in_buf2[AUDIO_BUF_SZ];
			//audio_out_ptr2=&audio_out_buf2[0];
			BUF2_CPLT=1;
		}

	}
}

/*
 * Audio playback process.
 * Transfer audio samples from input buffer to output buffer using pointers set by ISR (ping-pong buffers).
 * Used by main application during calibration mode
 */

void audioPlayback(){

	/*If channel 1 and 2 conversion was completed*/
	//if(BUF1_CPLT && BUF2_CPLT){
		BUF1_CPLT=0;
		BUF2_CPLT=0;

		/*Transfer samples from input to output buffers*/
		for(int i=0;i<AUDIO_BUF_SZ;i++){
			audio_out_ptr1[i]=audio_in_ptr1[i];
			audio_out_ptr2[i]=audio_in_ptr2[i];

		}
	//}
}

/*
 * Audio preprocessing.
 * Transfer audio samples from input buffer to output buffer using pointers set by ISR (ping-pong buffers).
 * DC offset is removed from every sample and threshold overflow is computed.
 * Used by main application during SSL mode
 */
void audioPreproc(){
	uint32_t mean1,mean2;
	uint32_t dc1,dc2;
	float rms1,rms2;

	/*
	 * Preproc loop will set rms values of last buf1 and buf2 to trigger cross-correlation
	 */

	/*If channel 1 and 2 conversion was completed*/
	if(BUF1_CPLT && BUF2_CPLT){

#ifdef 	TIME_PREPROC
		CHRONO_START();
#endif
		BUF1_CPLT=0;
		BUF2_CPLT=0;

		mean1=0;
		mean2=0;

		dc1=0;
		dc2=0;

		rms1=0;
		rms2=0;
		

		/*
		 * Transfer samples from input to cross-correlation buffers
		 */
		for( int i=0 ; i<AUDIO_BUF_SZ || i<DC_BUF_SZ ; i++ ){

			/*
			 * Compute the mean value of input buffers 1 and 2
			 */
			if(i<AUDIO_BUF_SZ){
				mean1+=audio_in_ptr1[i];
				mean2+=audio_in_ptr2[i];
			}
			/*
			 * Compute estimate of DC offset based on the last DC_BUF_SZ mean values computed in buffers
			 */
			if(i<DC_BUF_SZ){
				dc1+=dc_buf_ch1[dc_buf_idx];
				dc2+=dc_buf_ch2[dc_buf_idx];
			}

		}

		/*
		 * Enqueue current mean value of input buffer in dc buffer
		 */
		dc_buf_ch1[dc_buf_idx]=(uint16_t)( mean1/AUDIO_BUF_SZ );
		dc_buf_ch2[dc_buf_idx]=(uint16_t)( mean2/AUDIO_BUF_SZ );

		dc1/=DC_BUF_SZ;
		dc2/=DC_BUF_SZ;

		/*
		 * Subtract DC offset to have 0 mean signal in cross-correlation buffer.
		 * Compute RMS of signals in ch1-2 input buffers
		 */

		for(int i=0,idx;i<AUDIO_BUF_SZ;i++){

			idx=xcor_buf_oset * AUDIO_BUF_SZ + i;
			xcor_buf1[idx]=audio_in_ptr1[i] - dc1;
			xcor_buf2[idx]=audio_in_ptr2[i] - dc2;

			rms1+=( xcor_buf1[idx] * xcor_buf1[idx] ) / AUDIO_BUF_SZ;
			rms2+=( xcor_buf2[idx] * xcor_buf2[idx] ) / AUDIO_BUF_SZ;
		}

		rms1=sqrt(rms1);
		rms2=sqrt(rms2);

#ifdef TIME_PREPROC
		CHRONO_STOP();
#endif
		
		/*
		 * If any of the input buffers has RMS under threshold
		 */
		if(rms1>threshold && rms2>threshold){
			sprintf(msg_buf,"\r\nrms1:%d\r\nrms2:%d\r\nthreshold:%d\r\n",(int)rms1,(int)rms2,threshold);
			HAL_UART_Transmit_DMA(&huart6,(uint8_t*)msg_buf,strlen(msg_buf));
			/*Increment over threshold counter up to 3*/
			if(ovr_thr_cnt<3)
				ovr_thr_cnt++;
		}
		else{
			ovr_thr_cnt=0;
		}


		/*
		 * Update cross-correlation circular buffer offset
		 * XCOR_BUF_SZ = AUDIO_BUF_SZ * XCOR_BUF_MULT
		 */
		xcor_buf_oset=( xcor_buf_oset + 1 ) % XCOR_BUF_MULT;


		//sprintf(msg_buf,"\r\nxcor_oset:%d\r\nrms1:%d\r\nmax_in:%hu\r\ndc:%hu\r\nmax_xcor_buf1:%drms2:%d\r\nthreshold:%d\r\n",
				//xcor_buf_oset,(int)rms1,max16,dc1,max,(int)rms2,threshold);

		/*
		 * Update DC buffer offset idx
		 */
		dc_buf_idx=( dc_buf_idx + 1 ) % DC_BUF_SZ;
	}
}

/*
 * Cross correlation function.
 * Cross correlation circular buffer
 */
void xcor(){


#ifdef TIME_XCOR
	CHRONO_START();
#endif

	for(int i=0;i<AUDIO_BUF_SZ*2-1;i++){
		for(int j=0;j<AUDIO_BUF_SZ;j++){
		}
	}
#ifdef TIME_XCOR
	CHRONO_STOP();
#endif
}


/*
 * Log debug UART interface motor position computed since the latest call to rstAngle(&motor)
 */
void logMotor(){

	sprintf(msg_buf,"Motor position: %f %f %d \r\n\r\n\r\n",
								motor.ang_idx*motor.res,motor.res,motor.ang_idx);
	HAL_UART_Transmit_DMA(&huart6,(uint8_t*)msg_buf,strlen(msg_buf));
}

/*Read joystick's position and EXTI buttons and perform one motor step according to joystick direction*/
void motorControl(){

	JstickDir dir;

	if(MOTOR_MV){
		MOTOR_MV=0;

		dir=jstickGetDirPoll(&js);
		if(dir==LEFT || LEFT_BUT_PUSH){
			LEFT_BUT_PUSH=0;
			step(&motor,1);

			logMotor();

		}
		else if(dir==RIGHT || RIGHT_BUT_PUSH){
			RIGHT_BUT_PUSH=0;
			step(&motor,0);

			logMotor();
		}
	}

}

/*Functions to init calibration/sound source localization mode*/
void initCalibration(){
	  /*Start DMA request to playback audio through DAC channels 1 and 2*/
	  HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)audio_out_buf1,AUDIO_TOT_BUF_SZ,DAC_ALIGN_12B_R);
	  HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_2,(uint32_t*)audio_out_buf2,AUDIO_TOT_BUF_SZ,DAC_ALIGN_12B_R);

	  GPIOD->ODR&=~GPIO_PIN_15;
}

void initSSL(){

	uint16_t dc_init=2048;
	  /*Stop playback loop*/
	  HAL_DAC_Stop_DMA(&hdac,DAC_CHANNEL_1);
	  HAL_DAC_Stop_DMA(&hdac,DAC_CHANNEL_2);

	  /*Reset motor angle idx to 0 to set initial camera offset*/
	  rstAngle(&motor);
	  logMotor();
	  mode=1;

	  /*Reset xcor_buf_oset*/
	  xcor_buf_oset=0;
	  /*Reset DC buffer idx*/
	  dc_buf_idx=0;
	  /*Reset threshold to a start value of 100*/
	  threshold=30;
	  /*Reset over threshold counter*/
	  ovr_thr_cnt=0;

	  /*
	   * Fill dc offset estimation
	   */
	  for(int i=0;i<DC_BUF_SZ;i++){
		  dc_buf_ch1[i]=dc_init;
		  dc_buf_ch2[i]=dc_init;
	  }
	  /*Toggle led to notify the mode change*/
	  GPIOD->ODR|=GPIO_PIN_15;
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
  MX_TIM6_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC3_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


#ifdef DAC_DEBUG
  testDAC();
#endif

  /*
   * Init DMA handle data structures for thermal image in/out transfer
   */
  HAL_DMA_Init(&hdma_i2c1_rx);
  HAL_DMA_Init(&hdma_usart3_tx);

  /*Init amg8833 sensor with ad select pin connected to the ground*/
  amg8833Init(&cam,&hi2c1,0);

  /*Wait until amg8833 is ready*/
  while(!amg8833IsReady(&cam)){
	  GPIOD->ODR|=GPIO_PIN_14;
  }

  GPIOD->ODR&=~GPIO_PIN_14;

  /*Start Timer 6 - Update event every 1/20 s for thermal camera reading*/
  HAL_TIM_Base_Start_IT(&htim6);
  /*Start Timer 7 - Update event every 1/10 s for motor control*/
  HAL_TIM_Base_Start_IT(&htim7);

  /*
   * Start audio clock
   */
  HAL_TIM_Base_Start_IT(&htim2);
  /*
   * Start audio DMA continous reading
   * ADC1 and DAC both works with htim2 conversion clock
   */
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)audio_in_buf1,AUDIO_TOT_BUF_SZ);
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)audio_in_buf2,AUDIO_TOT_BUF_SZ);

  /*Init step motor data structure*/
  initStep(&motor,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIOD,FULL,&htim4);

  /*Init joystick img_buf structure with yellow error pin*/
  initJstick(&js,&hadc3,GPIO_PIN_12,GPIOD);
  /*Start ADC3 associated with joystick*/
  HAL_ADC_Start(&hadc3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  MODE_TOGGLE=0;
  mode=1;
  threshold=100;

  while (1)
  {
	  thermalImgFSM();

	  /*MODE_TOGGLE bit is set by the EXTI4 button line debounce timer TIM10*/
	  if( !MODE_TOGGLE ){
		  /*
		   * Calibration mode:
		   * -microphones AD conversion can be tested connecting an oscilloscope or an amplifier to DAC channels 1/2
		   * -camera can be moved manually using joystick and buttons to set angular offset
		   */
		  if(mode){
			  initCalibration();
			  mode=0;
		  }

		  audioPlayback();
		  motorControl();
	  }
	  else{
		  if(!mode){
			  /*
			   * Sound Source Localization mode:
			   * motor movement tests
			   */
			  initSSL();
			  mode=1;
		  }
		  audioPreproc();
		  if(ovr_thr_cnt>=3){
			  GPIOD->ODR|=GPIO_PIN_13;
		  }
		  else{
			  GPIOD->ODR&=~GPIO_PIN_13;
		  }

/*
#ifdef TIME_AUDIO_PROC
		  CHRONO_START();
#endif
		  audioPreproc();
		  if(ovr_thr){
			  xcor();
		  }
#ifdef TIME_AUDIO_PROC
		  CHRONO_START();
#endif
		  if(LEFT_BUT_PUSH){
			  LEFT_BUT_PUSH=0;
			  threshold-=5;
			  sprintf(msg_buf,"Threshold %d\r\n",threshold);
			  HAL_UART_Transmit_DMA(&huart6,msg_buf,strlen(msg_buf));
		  }
		  else if(RIGHT_BUT_PUSH){
			  RIGHT_BUT_PUSH=0;
			  threshold+=5;
			  sprintf(msg_buf,"Threshold %d\r\n",threshold);
			  HAL_UART_Transmit_DMA(&huart6,msg_buf,strlen(msg_buf));
		  }
		  */
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 62499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 99;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 2499;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD1 PD2 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /*
   * Step api callback: perform one step while moving to an angle in interrupt mode
   */
  else if( htim->Instance == TIM4 ){
	  stepIt(&motor);
  }
  /*
   *
   */
  else if( htim->Instance == TIM6 ){
	  AMG_RD_START=1;
  }
  else if( htim->Instance == TIM7 ){
	MOTOR_MV=1;
  }

  /*External interrupt push buttons debounce timer*/
  else if( htim->Instance == TIM10 ){

	  if( GPIOE->IDR & GPIO_PIN_2 && EXTI_BUT_PUSH  )
		  LEFT_BUT_PUSH=1;

	  else if( GPIOE->IDR & GPIO_PIN_3  && EXTI_BUT_PUSH )
		  RIGHT_BUT_PUSH=1;

	  else if( GPIOE->IDR & GPIO_PIN_4 && EXTI_BUT_PUSH )
		  MODE_TOGGLE^=1;

	  EXTI_BUT_PUSH=0;

  }

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
