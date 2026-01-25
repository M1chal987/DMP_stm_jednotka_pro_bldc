/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/*
 * pinout
 *  A11 - L1 high side
 *  A8 -  L1 low side
 *  B0 - L2 low side
 *  A9 - L2 high side
 *  B1 - L3H high side
 *  C2 - L3L low side
 *  svpwm
 *
 *  SPI2 - nejede clock nahradni SPI 1
 *
 *
 *
 *  video od ST timery
 *  https://www.youtube.com/watch?v=rDaC2N-33Oo
 *
 * */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
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

CORDIC_HandleTypeDef hcordic;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */




uint32_t i = 0;
uint8_t cislo8 = 0;
uint16_t cislo16 = 10;
int32_t cislo32v2 = 0;
uint16_t SPI_received_data = 0;
int16_t ang_velocity;
uint16_t uhel_abs = 0; // range 0 3600
uint16_t uhel_abs_prev;
uint16_t enc_ang_offset = 0;// range 0 3600
extern uint16_t TIM1_ov_cnt;

int32_t deg_sec = 0; // pouziva desetiny stupne

// ================ UART2 ===================
uint8_t UART2_RxBuffer[40] = {0};
uint16_t UART_RxSize = 40;
char Recieved_data[40] = "\0";
uint8_t rec_ack = 1; // received message was read and evaluated
float rec_floats[4];
uint8_t sequencer_write_index;
uint8_t rec_fail;
uint8_t angle_poll = 0;
uint16_t angle_poll_cnt;
//uint16_t wait_cycles = 500; // used for test - control PC dependent


//======== UART plotting ==================
uint8_t plot = 0;   //b0 = plot EN, b1 = plot flag generated in tim1
uint16_t plotCNT = 0;// send x plot variables of data
uint16_t maxPltCnt = 500;
uint16_t dataA[500];
uint16_t dataB[500];
uint16_t dataH[500];

float dataC_Fl[500];
float dataD_Fl[500];
float dataE_Fl[500];
float dataF_Fl[500];
float dataG_Fl[500];

uint16_t plot_data_index = 0;
uint8_t plot_fin;

// ======== ADC variables ===================
uint8_t adc_read_flag;   // 00,01,10,11

extern uint16_t ADC1_data;
extern uint16_t ADC2_data;
uint16_t ADC_TIM_OFFSET = 4000;
uint8_t val_cnt = 16;


float Isense_U_zero = 8274; // 2070     ADC1
float Isense_V_zero = 8258; //			 ADC2
// ======= FOC variables =====================
static float I_d;
static float I_q;
static float I_a;
static float I_b;

const float sinTable[128] = {0, 0.0123681596643367, 0.0247344272819422, 0.037096911095525, 0.0494537199266298, 0.0618029634649443, 0.0741427525574735, 0.0864711994975375, 0.0987864183135472, 0.111086525057515, 0.123369638093258, 0.13563387838424, 0.147877369781027, 0.160098239308295, 0.172294617451347, 0.184464638442114,
							 0.196606440544567, 0.208718166339527, 0.220797963008804, 0.232843982618637, 0.244854382402386, 0.256827325042434, 0.268760978951252, 0.280653518551592, 0.292503124555762, 0.304307984243929, 0.316066291741428, 0.327776248295019, 0.339436062548057, 0.351043950814523, 0.362598137351895, 0.374096854632792,
							 0.385538343615363, 0.396920854012386, 0.408242644559018, 0.419501983279171, 0.430697147750464, 0.441826425367713, 0.452888113604924, 0.463880520275737, 0.474801963792296, 0.485650773422488, 0.496425289545534, 0.507123863905866, 0.517744859865279, 0.528286652653294, 0.538747629615715, 0.549126190461327,
							 0.559420747506701, 0.569629725919079, 0.579751563957284, 0.58978471321063, 0.599727638835797, 0.609578819791626, 0.619336749071799, 0.628999933935384, 0.638566896135185, 0.648036172143884, 0.657406313377922, 0.666675886419107, 0.675843473233886, 0.684907671390277, 0.69386709427241, 0.702720371292642,
							 0.711466148101233, 0.720103086793525, 0.728629866114611, 0.737045181661463, 0.74534774608247, 0.753536289274372, 0.761609558576566, 0.769566318962725, 0.777405353229732, 0.785125462183886, 0.792725464824352, 0.800204198523822, 0.807560519206378, 0.8147933015225, 0.82190143902123, 0.828883844319424,
							 0.835739449268105, 0.842467205115857, 0.849066082669268, 0.855535072450367, 0.861873184851057, 0.868079450284496, 0.874152919333428, 0.880092662895418, 0.885897772324983, 0.891567359572599, 0.897100557320547, 0.902496519115597, 0.907754419498491, 0.912873454130225, 0.917852839915092, 0.922691815120477,
							 0.927389639493386, 0.931945594373688, 0.936358982804054, 0.940629129636571, 0.944755381636032, 0.948737107579861, 0.952573698354675, 0.956264567049469, 0.959809149045395, 0.96320690210214, 0.966457306440873, 0.969559864823762, 0.972514102630038, 0.975319567928601, 0.97797583154716, 0.980482487137879,
							 0.982839151239545, 0.985045463336224, 0.987101085912419, 0.989005704504693, 0.990759027749781, 0.992360787429161, 0.993810738510082, 0.995108659183055, 0.996254350895778, 0.997247638383512, 0.998088369695895, 0.998776416220185, 0.999311672700935, 0.999694057256094, 0.999923511389535, 1};
const float one_over_sqrt3 = 1/sqrt(3);
float I_u; // proud fáze 1
float I_v;// proud fáze 2
float cos_ang = 0;
float sin_ang = 0;

static float Int_I_d_val; // integral components result
static float Int_I_q_val;

float I_d_error;
float I_q_error;

// ============ REGULATOR Coefficients ===========
float Kp_d = 1; // proportional gain I_d
float Ki_d = -5000; // integral gain I_d

float Kp_q = 3; // proportional gain I_q
float Ki_q = 10000; // integral gain I_d

// position regulators
uint16_t des_position; // desired position value
uint16_t pos_sequence[36] = {0,900,1800,2700,0,1800};
uint8_t pos_sequence_len = 7;
uint8_t use_pos_PID = 1;
float pos_Kp = 0.6;
float pos_Ki = 5;
float pos_Kd = 30;
float pos_integrator;
float pos_e;

// velocity regulator
int16_t des_velocity; // desired velocity value
int16_t vel_sequence[5] = {0,100,-100,200,-200};
uint8_t vel_sequence_max_len = 5;
uint8_t use_vel_PID = 0;
float vel_Kp = 0.6;
float vel_Ki = 5;
float vel_Kd = 30;
float vel_integrator;
float vel_e;


// position velocity regulator common variables
uint8_t use_sequencer;
uint8_t sequencer_CNT;
int16_t sequencer_us = 10000;

int16_t I_d_rqst = 0;
uint8_t USE_OPEN_LOOP = 0; // I_d reguest will be directly corection voltage
int16_t I_q_rqst = 0;
// ==================================

uint8_t vect_codes[] = {0,0,0}; // set codes to be accesed by TIM1 ISR order is first,second,null - SVPWM functions
uint8_t prev_null_vect = 0; // values 6,7
uint16_t first_vector_time; // lower angle
uint16_t second_vector_time; // higher angle
uint16_t null_vector_time;

//uint16_t angle_pulse; // TIM 3 PWM read angle from encoder

uint16_t SVPWM_MOD_ANG=0; // current modulation angle
uint16_t SVPWM_MOD_AMP=0; // current modulation amplitude
uint16_t SVPWM_ROTOR_OFFSET = 0; // svpwm offset from rotor position (motor run test?)

uint8_t str[40]; // string array used in UintToStr function

uint8_t TIM1_OV_FLAG;

extern uint32_t ADC_data[2];

//uint8_t stavSekvenceStridani = 0; // not used
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_CORDIC_Init(void);
/* USER CODE BEGIN PFP */
uint16_t StrToUint(uint8_t str[]);
void recieve_eval(void);
void Transmit(uint8_t str[]);
void UintToStr(uint16_t num, char *str);
void BinToStr(uint8_t binData,char *str);
void initDrivePinsAsGPIO(void);
void set_SVPWM_vect(uint8_t vect_code);
void SVPWM(uint16_t amplitude, uint16_t angle);
void plot_tx(void);

void UART_delay_func(){
	//for(uint16_t i = 0; i< wait_cycles;i++){}
	for(uint16_t i = 0; i< 2000;i++){
		asm("nop");
		asm("nop");
		asm("nop");
	}

}

void recieve_eval(){ // TODO : program to be fully controllable trough uart - position, pid values, test multiple values in one command, sequencer
	/*
	 * set of functions
	 * tune position pid, DQ pi controller
	 * set position -- if angle > 3600 enable pos pid only
	 * enable sequencer + write sequence values set length set step time
	 * plot function
	 * enable and disable drive
	 * plot function can disable drive
	 *
	 *
	 * sequencer
	 * c_seqv_tim - set step delay in 0.1 ms    0 - disable sequencer max 30000;
	 * c_seqv_len - set amount of steps in sequencer
	 * c_seqv_step - set_step - internal step counter - resets with c_seqv_len command, writes 1 step of sequence
	 *ang poll enable polling of position data
	 * c set id disables pos pid and sets id to rec. number
	 * c vel enable vel pid and set target value
	 *
	 * Received data - array that stores full received command
	 * */
	uint8_t msg_tx[] = "ack \r\n";
	char commands[13][15] = {"c_enable","c_pos","c_plt","c_pid_pos","c_dq_pi","c_disable","c_seqv_tim","c_seqv_len","c_seqv_step","c_ang_pol","c_set_id","c_vel","c_pid_vel"};
	uint8_t com_cnt = 13; // number of commands
	uint8_t recognised_com = 0;
	uint8_t com_check = 0; // which command is being checked
	uint16_t rec_num;

	// Recognizing command
	while (recognised_com == 0){
		for(uint8_t i = 0;i<15;i++){

			if(commands[com_check][i] == 0){
				recognised_com = com_check+1; // command was recognized
				break;
			}
			if(com_check < com_cnt && commands[com_check][i] != Recieved_data[i]){ // data is not com_check command
				break;
			}

		}
		com_check++;
		if(com_check == com_cnt && recognised_com == 0){
			for(uint8_t i = 0;i<40;i++){
				Recieved_data[i] = 0;
				}
			rec_ack = 1;
			return;
		}
	}

	// ACK message
	msg_tx[4] = recognised_com + '0'; // convert to char and put in ack message
	for(uint8_t i = 0; i<6;i++){ // confirm command
		LL_USART_TransmitData8(USART2, msg_tx[i]);
		UART_delay_func();
	}
	// -----------

	rec_ack = 1;
	if(recognised_com == 3){ // c_plt plotting command
		plot = 0x01;
		plotCNT = maxPltCnt;
		return;
	}
	// handling of received number(s)
	/*
	char* end;
	float rec_fl;
	rec_fl = strtod(Recieved_data,&end);
	*/
	rec_num = StrToUint(Recieved_data);
	UintToStr(rec_num, str);
	str[5] = '\r';
	str[6] = '\n';
	str[7] = 0;
	Transmit(str);




	// apply received data to SVPWM function
	switch(recognised_com){
	default:
		break;
	case 1:
		// enable drive
		LL_TIM_EnableCounter(TIM1);
		break;
	case 2:
		if(rec_num <= 3600){ // set target angle
			use_pos_PID = 1;
			use_vel_PID = 0;
			des_position = rec_num;
		}
		else{use_pos_PID = 1;use_vel_PID = 0;}

		break;
	case 4:
		// tune pid for position regulator
		str_to_floats();
		pos_Kp = rec_floats[0];
		pos_Ki = rec_floats[1];
		pos_Kd = rec_floats[2];
		break;

	case 5: // id iq regulator
		str_to_floats();
		Kp_d = rec_floats[0];
		Ki_d = rec_floats[1];
		Kp_q = rec_floats[2];
		Ki_q = rec_floats[3];
		break;
	case 6 : // disable drive
		LL_TIM_DisableCounter(TIM1);
		break;
	case 7 : // 7 8 9 sequencer uses rec_num
			if(rec_num == 0){use_sequencer = 0;}
			else if(rec_num <= 32000){sequencer_us = rec_num; use_sequencer = 1;}
		break;
	case 8 :
			if(rec_num <= 36){pos_sequence_len = rec_num;sequencer_write_index = 0;}
		break;
	case 9 :
			if(rec_num <= 3600 && sequencer_write_index < pos_sequence_len){pos_sequence[sequencer_write_index] = rec_num; sequencer_write_index++;}
			//else if(sequencer_write_index == pos_sequence_len){}// error condition more steps than length
		break;
	case 10 :
		if(rec_num == 1){angle_poll = 1;}
		else{angle_poll = 0;}
		break;
	case 11 : // id set
		str_to_floats();
		use_pos_PID = 0;
		use_vel_PID = 0;
		I_d_rqst = rec_floats[0];
		break;
	case 12 :
		str_to_floats();
		use_pos_PID = 0;
		use_vel_PID = 1;
		use_sequencer = 0;
		des_velocity = rec_floats[0];
		break;
	case 13:
			// tune pid for velocity regulator
			str_to_floats();
			vel_Kp = rec_floats[0];
			vel_Ki = rec_floats[1];
			vel_Kd = rec_floats[2];
		break;
	}
	// reset data array
	for(uint8_t i = 0;i<40;i++){
	Recieved_data[i] = 0;
	}


}
// string to floats
void str_to_floats(void){
	//float rec_floats[3];
	uint8_t rec_fl_cnt = 0;
	char *err, *p = Recieved_data;
	double val;
		while (*p) {
			val = strtod(p, &err);
			if (p == err) p++;
			else if ((err == NULL) || (*err == 0) || rec_fl_cnt >= 4) { break;}
			else { p = err + 1; rec_floats[rec_fl_cnt] = val; rec_fl_cnt++;} // correct value
		}
}
void Transmit(uint8_t str[]){
	uint8_t i=0;
	while(str[i] != 0){
		LL_USART_TransmitData8(USART2, str[i]);
		UART_delay_func();
		i++;
	}
}

uint16_t StrToUint(uint8_t str[]){
	uint16_t out = 0;
	uint8_t a = 0;
	for(uint8_t i = 0; i<40;i++){
		a = str[i] - '0'; // division underflow can happen - it's fine
		if(a < 10){
			out *= 10;
			out += a;

		}
	}
	return out;
}
void UintToStr(uint16_t num, char *str){ // sets first 5 chars to uint in dec
	for(uint8_t i = 0;i < 40; i++){ // clear data string
		str[i] = 0;
	}

	for(uint8_t i = 0;i < 5; i++){
		str[4-i] = num % 10 + '0';
		num = num / 10;
	}
}
void BinToStr(uint8_t binData,char *str){// sets first 8 chars to bin data
	for(uint8_t i = 0;i < 8; i++){
		str[7-i] = binData & 1<<i;
	}
}
void readEnc(void){
	uint32_t cislo32;
	LL_SPI_TransmitData16(SPI1,0xffff);
	//for (i = 0; i < 1000; i++){} // delay between commands
	//recAngPrev = uhel_abs; // previous angle - speed calculation
	SPI_received_data = LL_SPI_ReceiveData16(SPI1); // -8200 some error
	SPI_received_data &= 0x3fff;
	if(SPI_received_data >= 8192){
		cislo32 = SPI_received_data - 8192;
	}
	else{
		cislo32 = SPI_received_data;// -8200;
	}
	uhel_abs_prev = uhel_abs;
	uhel_abs = (cislo32 * 3600)/8192;
	// offset angle by 330
	uhel_abs =uhel_abs + 3600 - enc_ang_offset;
	uhel_abs = uhel_abs % 3600;

	ang_velocity = uhel_abs - uhel_abs_prev;
	if(ang_velocity > 1800){ang_velocity -= 3600;}
	else if(ang_velocity < -1800){ang_velocity += 3600;}


	//angDiff = recAngPrev - uhel_abs;
	//if(angDiff > 1800){ angDiff = 3600 - angDiff;}// speed when angles are between revs
	//else if(angDiff < - 1800){angDiff = - angDiff -3600;}
	//else{
		//angDiff =  - angDiff;
	//}
	//cislo32 = angDiff;
	//const uint16_t timConst = 800; // timer overflow Frequency
	//degSec = cislo32 * timConst / 10;
}


float pos_PID(uint16_t pos_w, uint16_t pos_y){
	int16_t pos_e_prev = pos_e;
	pos_e = pos_w - pos_y;
	if(pos_e > 1800){pos_e -= 3600;}
	else if(pos_e < -1800){pos_e += 3600;}
	float pos_u; // control output torgue
	pos_integrator += pos_e / 10000;
	if(pos_integrator > 1000){pos_integrator = 1000;}
	if(pos_integrator < -1000){pos_integrator = -1000;}

	pos_u = pos_Kp * pos_e + pos_Ki * pos_integrator + pos_Kd * (pos_e - pos_e_prev);
	return pos_u;
}

float vel_PID(int16_t vel_w, int16_t vel_y){
	int16_t vel_e_prev = vel_e;
	vel_e = vel_w - vel_y;
	float vel_u; // control output torgue
	vel_integrator += vel_e / 10000;
	if(vel_integrator > 1000){vel_integrator = 1000;}
	if(vel_integrator < -1000){vel_integrator = -1000;}

	vel_u = vel_Kp * vel_e + vel_Ki * vel_integrator + vel_Kd * (vel_e - vel_e_prev);
	return vel_u;
}

void plot_save_sample(void){
	dataA[plot_data_index] = ADC1_data;
	dataB[plot_data_index] = ADC2_data;
	dataC_Fl[plot_data_index] = I_d_error;
	dataD_Fl[plot_data_index] = I_q_error;
	dataE_Fl[plot_data_index] = uhel_abs;
	dataF_Fl[plot_data_index] = I_d;
	//dataG_Fl[plot_data_index] = I_q;

	plot_data_index++;
	if(plotCNT == 0 || plotCNT > 1000 || plot > 3){
		plot = 4; // b2 = ready for transmit
		plot_data_index = 0;
		//plot_tx();
		plotCNT = 0;
	}
	else{
		plotCNT--;
	}
}

void plot_tx(void){ // send all acquired data from plot_save_sample
		//__disable_irq;
	//LL_TIM_DisableCounter(TIM1);
	//set_SVPWM_vect(6); // disable all Motor mosfets
	for (uint16_t i=0; i < maxPltCnt;i++){
		snprintf(str,40,"%f",dataC_Fl[i]);
		Transmit(str);
		Transmit(",");
		snprintf(str,40,"%f",dataD_Fl[i]);
		Transmit(str);
		Transmit(",");
		snprintf(str,40,"%f",dataE_Fl[i]);
		Transmit(str);
		Transmit(",");/*
		snprintf(str,40,"%f",dataF_Fl[i]);
		Transmit(str);
		Transmit(",");
		snprintf(str,40,"%f",dataG_Fl[i]);
		Transmit(str);
		Transmit(",");*/
		UintToStr(dataA[i], str);
		str[5] = ',';
		Transmit(str);
		UintToStr(dataB[i], str);
		str[5] = ',';
		Transmit(str);
		UintToStr(dataH[i], str);
		Transmit(str);
		Transmit("\r\n");
	}
	plot = 0;
	//LL_TIM_EnableCounter(TIM1);
}

void initDrivePinsAsGPIO(void){
	/*
	 * pinout
	 *  A11 - L1 high side
	 *  A8 -  L1 low side
	 *  B0 - L2 low side
	 *  A9 - L2 high side
	 *  B1 - L3L low side
	 *  C2 - L3H high side
	 */
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);

	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);

	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
	HAL_Delay(10);
	set_SVPWM_vect(6); // 6 is 000 - can be used continually
}
void set_SVPWM_vect(uint8_t vect_code){
	switch (vect_code){
		/*
		 *  CH1 CH2 CH3
		 * 0 1   0   0	   2	  1
		 * 1 1   1   0		\    /
		 * 2 0   1   0		 \  /
		 * 3 0   1   1   3---------- 0
		 * 4 0   0   1		 /	\
		 * 5 1   0   1 		/	 \
		 * 6 0   0   0	   4	  5
		 * 7 1   1   1
		 * H A11 A9 B1
		 * L A8  B0 C2
		 *
		 * ch3 byl prohozen
		  */
	default:
		//sendDataUart1("default case - bad \n");
		break;
	case 0:
		GPIOA->ODR &= ~(LL_GPIO_PIN_9 | LL_GPIO_PIN_8); // reset A9 A8
		GPIOB->ODR &= ~LL_GPIO_PIN_1; // reset b1

		GPIOA->ODR |= LL_GPIO_PIN_11; // set A11
		GPIOB->ODR |= LL_GPIO_PIN_0; // set B0
		GPIOC->ODR |= LL_GPIO_PIN_2; // set c2
		break;
	case 1:
		GPIOB->ODR &= ~(LL_GPIO_PIN_0 | LL_GPIO_PIN_1); // reset B0 b1
		GPIOA->ODR &= ~LL_GPIO_PIN_8; // reset A8

		GPIOA->ODR |= LL_GPIO_PIN_11 | LL_GPIO_PIN_9; // set A9 A11
		GPIOC->ODR |= LL_GPIO_PIN_2; // set c2
		break;
	case 2:
		GPIOB->ODR &= ~(LL_GPIO_PIN_0 | LL_GPIO_PIN_1); // reset B0 b1
		GPIOA->ODR &= ~LL_GPIO_PIN_11; // reset A11

		GPIOA->ODR |= LL_GPIO_PIN_8 | LL_GPIO_PIN_9; // set A9 A8
		GPIOC->ODR |= LL_GPIO_PIN_2; // set c2
		break;
	case 3:
		GPIOB->ODR &= ~(LL_GPIO_PIN_0); // reset B0
		GPIOA->ODR &= ~LL_GPIO_PIN_11; // reset A11
		GPIOC->ODR &= ~LL_GPIO_PIN_2; // reset c2

		GPIOA->ODR |= LL_GPIO_PIN_8 | LL_GPIO_PIN_9; // set A9 A8
		GPIOB->ODR |= LL_GPIO_PIN_1; // set b1
		break;
	case 4:
		GPIOC->ODR &= ~LL_GPIO_PIN_2; // reset c2
		GPIOA->ODR &= ~(LL_GPIO_PIN_9 | LL_GPIO_PIN_11); // reset A9 A11

		GPIOA->ODR |= LL_GPIO_PIN_8; // set A8
		GPIOB->ODR |= (LL_GPIO_PIN_0 | LL_GPIO_PIN_1); // set B0 b1
		break;
	case 5:
		GPIOC->ODR &= ~LL_GPIO_PIN_2; // reset c2
		GPIOA->ODR &= ~(LL_GPIO_PIN_8 | LL_GPIO_PIN_9); // reset A8 A9

		GPIOA->ODR |= LL_GPIO_PIN_11; // set A11
		GPIOB->ODR |= (LL_GPIO_PIN_0 | LL_GPIO_PIN_1); // set B0 b1
		break;
	case 6:
		GPIOB->ODR &= ~LL_GPIO_PIN_1; // reset b1
		GPIOA->ODR &= ~(LL_GPIO_PIN_11 | LL_GPIO_PIN_9); // reset A9 A11

		GPIOA->ODR |= LL_GPIO_PIN_8; // set A8
		GPIOB->ODR |= LL_GPIO_PIN_0; // set B0
		GPIOC->ODR |= LL_GPIO_PIN_2; // set c2
		break;
	case 7:
		GPIOB->ODR &= ~(LL_GPIO_PIN_0); // reset B0
		GPIOA->ODR &= ~LL_GPIO_PIN_8; // reset A8
		GPIOC->ODR &= ~LL_GPIO_PIN_2; // set c2

		GPIOA->ODR |= LL_GPIO_PIN_11 | LL_GPIO_PIN_9; // set A9 A11
		GPIOB->ODR |= LL_GPIO_PIN_1; // set b1
		break;
	}
}

void CLOSED_LOOP_MAIN(void){
	readEnc();
	if(use_vel_PID){
		I_d_rqst = vel_PID(des_velocity, ang_velocity);
	}

	else if(use_pos_PID){
		I_d_rqst = pos_PID(des_position, uhel_abs);
	}
	// v debugeru jsem dal breakpoiny na začátek a konec funkce při spuštění programu který prošel mezi těmi breakpointy tak s mi prom. co uchovává počet přetečení tim1 zvedala o 5 - 7
	// takže ten program trvá asi 500 us
	/*
	const float sinTable[128] = {0, 0.0174524064372835, 0.034899496702501, 0.0523359562429438, 0.0697564737441253, 0.0871557427476582, 0.104528463267653, 0.121869343405147, 0.139173100960065, 0.156434465040231, 0.17364817766693, 0.190808995376545, 0.207911690817759, 0.224951054343865, 0.241921895599668, 0.258819045102521, 0.275637355816999, 0.292371704722737, 0.309016994374947, 0.325568154457157, 0.342020143325669, 0.3583679495453, 0.374606593415912, 0.390731128489274, 0.4067366430758, 0.422618261740699, 0.438371146789077, 0.453990499739547, 0.469471562785891, 0.484809620246337, 0.5, 0.515038074910054, 0.529919264233205, 0.544639035015027, 0.559192903470747, 0.573576436351046, 0.587785252292473, 0.601815023152048, 0.615661475325658, 0.629320391049838, 0.642787609686539, 0.656059028990507, 0.669130606358858, 0.681998360062499, 0.694658370458997, 0.707106781186548, 0.719339800338651, 0.731353701619171, 0.743144825477394, 0.754709580222772, 0.766044443118978, 0.777145961456971, 0.788010753606722, 0.798635510047293, 0.809016994374948, 0.819152044288992, 0.829037572555042, 0.838670567945424, 0.848048096156426, 0.857167300702112, 0.866025403784439, 0.874619707139396, 0.882947592858927, 0.891006524188368, 0.898794046299167, 0.90630778703665, 0.913545457642601, 0.92050485345244, 0.927183854566787, 0.933580426497202, 0.939692620785908, 0.945518575599317, 0.951056516295154, 0.956304755963035, 0.961261695938319, 0.965925826289068, 0.970295726275997, 0.974370064785235, 0.978147600733806, 0.981627183447664, 0.984807753012208, 0.987688340595138, 0.99026806874157, 0.992546151641322, 0.994521895368273, 0.996194698091746, 0.997564050259824, 0.998629534754574, 0.999390827019096, 0.999847695156391, 1};
	const float one_over_sqrt3 = 1/sqrt(3);
	float I_u; // proud fáze 1
	float I_v;// proud fáze 2
	float cos_ang = 0;
	float sin_ang = 0;
	*/
	// zapsání na data z ADC k nulové hodnotě - půl rozsahu
	I_a = ADC1_data - Isense_U_zero; // directly eqvivalent
	I_v = ADC2_data - Isense_V_zero;

	uint16_t angle_poles = ((3600-uhel_abs)%514); // angle adjusted for pole count // přepočet úhlu z enkoderu vzhledem k počtu pól párům motoru nový rozsah 0 513
	uint8_t qvadrant = angle_poles / 128; // qvadrant 0 - 3
	uint8_t ang_qvadrant = angle_poles % 128;

	switch(qvadrant){
	case 0:
		sin_ang = sinTable[ang_qvadrant];
		cos_ang = sinTable[127 - ang_qvadrant];
		break;
	case 1 :
		sin_ang = sinTable[127 - ang_qvadrant];
		cos_ang = -sinTable[ang_qvadrant];
		break;
	case 2:
		sin_ang = -sinTable[ang_qvadrant];
		cos_ang = -sinTable[127 - ang_qvadrant];
		break;
	case 3 :
		sin_ang = -sinTable[127 - ang_qvadrant];
		cos_ang = sinTable[ang_qvadrant];
		break;
	}
	//motor má 7 pól párů takže jedna ot. enkoderu je 7 otáček elektrických
	//celočíselným prom. pro úhel dávám rozsah 0 - 3600

	//angle_rad = M_PI*angle_poles/1800; // convert 0-3600 to rads přepočet na radiány
	// theoretical "A,B frame" - clarke park transform etc.
	// source:  https://ww1.microchip.com/downloads/aemdocuments/documents/fpga/ProductDocuments/UserGuides/sf2_mc_park_invpark_clarke_invclarke_transforms_ug.pdf

	/*
	 * # I_v  I_b
	 *      \ |
     *       \|
     *		   ---- I_u = I_a
     *		 /
     *		/
	 *	I_w
	 *
	 * */

	// přepočet z proudů fázemi na dva kolmé proudy AB
	//I_a = I_u;
	I_b = (I_a+2*I_v)*one_over_sqrt3;


	// přepočet z proudů AB na DQ které jsou vztaženy k poloze rotoru
	I_d = I_b*cos_ang + I_a*sin_ang;
	I_q = I_a*cos_ang - I_b*sin_ang;

	// obrácení hodnot
	//I_d = -I_d;
	I_q = -I_q;


	// integration - počítání integrátoru pro PI regulátor v DQ rámci
	I_d_error = (I_d_rqst - I_d);
	Int_I_d_val +=  I_d_error / 10000; // TIM1 overflow frequency

	I_q_error = (I_q_rqst - I_q);
	Int_I_q_val += I_q_error / 10000;

	// limiting integration runaway
	if (Int_I_d_val > 100000){Int_I_d_val = 100000;}
	else if (Int_I_d_val < -100000){Int_I_d_val = -100000;}

	if (Int_I_q_val > 100000){Int_I_q_val = 100000;}
	else if (Int_I_q_val < -100000){Int_I_q_val = -100000;}

	// ======= REGULATORS PI
	float V_d; // hodnoty pro korekci pwm rámec DQ - k rotoru
	float V_q;
	V_d = Kp_d*(I_d_rqst - I_d) + Ki_d*Int_I_d_val;
	V_q = Kp_q*(I_q_rqst - I_q) + Ki_q* Int_I_q_val;
	// ==================

	// prepocet korekcí na AB rámec
	float V_a = V_d*cos_ang - V_q*sin_ang;
	float V_b = V_q*cos_ang + V_d*sin_ang;
	// call svpwm function with values

	// prepocet korekcí na amplitudu a úhel modulace - svpwm
	float amp = sqrt( pow(V_a,2) + pow(V_b,2));

	float ang = atan(V_b/V_a);
	ang = ang*1800/M_PI;
	if(V_a < 0){ang += 1800;}
	else if (V_b < 0){ang += 3600;}
	// omezení max. amplitudy
	if(amp < 10000){SVPWM_MOD_AMP = amp;}
	else{
		SVPWM_MOD_AMP =0;
		Int_I_d_val = 0; // reset integratoru
		Int_I_q_val = 0;
	}
	// volání svpwm funkce
	SVPWM_MOD_ANG = ang;
	SVPWM(SVPWM_MOD_AMP,SVPWM_MOD_ANG);

	// funkce na uložení vzorku při dělání grafů přes uart - data se posílají v jiné funkci a je neaktivní dokud ji nezapnu z uartu
	if (plot & 0x02){
		plot_save_sample();
		plot &= ~0x02;
	}
}

void SVPWM(uint16_t amplitude, uint16_t angle){ // amplitude 0-10000 of DCbus, angle 0-3600
	angle = angle % 3600;
	// SVPWM states
	/* space vectors and channel states ch1 ch2 ch3
	 * 0 100	   2	  1
	 * 1 110		\    /
	 * 2 010		 \  /
	 * 3 011     3---------- 0
	 * 4 001		 /	\
	 * 5 101		/	 \
	 * 6 000	   4	  5
	 * 7 111
	 *  steps in one cycle
	 *  	phase vector 0-5
	 *  	new phase vector 0-5 offset by one - not present if angle is n*60 degrees
	 *  	determine optimal null vector for least channel switching 6/7 - based on previous null vector swap or don't swap first and second vector
	 */


	// determine SVPWM sector and angle with degree resolution
	//sendDataUart1("SVPWM line 187 \r\n");
	const uint16_t max_amplitude = 17000;
	const float vector_angles[] = {0,60*M_PI/180,120*M_PI/180,M_PI,240*M_PI/180,300*M_PI/180,2*M_PI,0};
	uint8_t sector = angle / 600; // 0-5
	//uint16_t remainder = angle % 600; // 0-599
	// source https://www.switchcraft.org/learning/2017/3/15/space-vector-pwm-intro z 15.11.2025



	/*
	 * real component amplitude*cos(angle)
	 * equation
	 * real component
	 * amplitude * cos(angle) = first_vector_time*10 000*cos(vector angle) + second_vector_time*10 000*cos(vector angle)
	 * imaginary component
	 * amplitude * sin(angle) = first_vector_time*10 000*sin(vector angle) + second_vector_time*10 000*sin(vector angle)
	 */
	// note make sin(vector_angle[n]) and cos(vector_angle[n]) into constant arrays to save calculation time
	angle = (angle % 600) * 0.85666666; // angle in range between 2 amplitude vectors 0 - 599 -> 0 514
	//float ang = M_PI*angle/1800;
	uint8_t qvadrant = angle * 128;
	uint8_t ang_qvadrant = angle % 128;

	switch(qvadrant){
	case 0:
		sin_ang = sinTable[ang_qvadrant];
		cos_ang = sinTable[127 - ang_qvadrant];
		break;
	case 1 :
		sin_ang = sinTable[127 - ang_qvadrant];
		cos_ang = -sinTable[ang_qvadrant];
		break;
	case 2:
		sin_ang = -sinTable[ang_qvadrant];
		cos_ang = -sinTable[127 - ang_qvadrant];
		break;
	case 3 :
		sin_ang = -sinTable[127 - ang_qvadrant];
		cos_ang = sinTable[ang_qvadrant];
		break;
	}
	second_vector_time = amplitude * sin_ang * 1.1547;
	first_vector_time  = amplitude * cos_ang - second_vector_time * 2;

	null_vector_time = max_amplitude - (first_vector_time + second_vector_time);


	if (0){ // prev_null_vect == 6
		// preferably use vectors 0 2 4
		if(1 ){ //sector % 2 == 0  // keep space vector order
			vect_codes[0] = sector;
			vect_codes[1] = (sector+1)%6;
			TIM1->CCR1 = first_vector_time;

		}
		else{ // swap space vector order
			vect_codes[1] = sector;
			vect_codes[0] = (sector+1)%6;
			TIM1->CCR1 = second_vector_time;
		}
		TIM1->CCR2 = first_vector_time + second_vector_time; // common for both variants
		vect_codes[2] = 7;
		prev_null_vect = 7;

	}
	else{
		if(1){// sector % 2 == 1 // keep space vector order
			vect_codes[0] = sector;
			vect_codes[1] = (sector+1)%6;
			TIM1->CCR1 = first_vector_time;
		}
		else{ // swap space vector order
			vect_codes[1] = sector;
			vect_codes[0] = (sector+1)%6;
			TIM1->CCR1 = second_vector_time;
		}
		TIM1->CCR2 = first_vector_time + second_vector_time; // common for both variants
		//TIM1->CCR3 = ADC_TIM_OFFSET;
		vect_codes[2] = 6;
		prev_null_vect = 6;
	}
	/*
	sendDataUart1("hodnoty casu vektoru ;");
	UintToStr(first_vector_time, str);
	str[5] = ' ';
	str[6] = ';';
	sendDataUart1(str);
	UintToStr(second_vector_time, str);
	str[5] = ' ';
	str[6] = ';';
	sendDataUart1(str);
	UintToStr(null_vector_time, str);
	str[5] = '\r';
	str[6] = '\n';
	sendDataUart1(str);
	*/
	return;
	// add stuff to set up interrupts on tim1 to set gpios according to vector times
	// first_vector_time + second_vector_time cannot exceed 10 000 (max amplitude)
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  //LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_5,LL_GPIO_MODE_OUTPUT); // ledka na A5
  //LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
  //LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
  //LL_GPIO_SetPinMode(GPIOG, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
  LL_TIM_EnableCounter(TIM1);
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableIT_CC1(TIM1);
  LL_TIM_EnableIT_CC2(TIM1);
  LL_TIM_EnableCounter(TIM6);

  USER_EXTI_Init();

  /*
  LL_TIM_EnableCounter(TIM3);
  LL_TIM_EnableIT_CC1(TIM3);
  LL_TIM_EnableIT_CC2(TIM3);
  LL_TIM_EnableIT_UPDATE(TIM3);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
*/
  LL_USART_EnableIT_RXNE(USART2);

  LL_SPI_Enable(SPI1);
  LL_SPI_SetMode(SPI1,LL_SPI_MODE_MASTER);
  initDrivePinsAsGPIO();

  UART_delay_func();
  EXTI->SWIER1 &= ~1;
  //LL_EXTI_GenerateSWI_0_31(LL_EXTI_LINE_0);
  //LL_SPI_SetNSSMode(SPI2,LL_SPI_NSS_HARD_OUTPUT);
  // interrupt function is in  stm32g4xx_it.c
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	/*
	if (stavSekvenceStridani == 0){
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
	}
	else if(stavSekvenceStridani == 2){
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
	}
	*/
	//loop_cntr++;
	//CLOSED_LOOP_MAIN();


	if(TIM1_OV_FLAG){
		if(angle_poll){
			angle_poll_cnt++;
			if(angle_poll_cnt > 500){
				UintToStr(uhel_abs, str);
				str[5] = '\n';
				str[6] = '\r';
				Transmit(str);
				angle_poll_cnt = 0;
			} // send angle to PC
		}
		if(use_sequencer && TIM1_ov_cnt > sequencer_us){
			if(use_pos_PID){
				des_position = pos_sequence[sequencer_CNT];
				sequencer_CNT++;
				sequencer_CNT = sequencer_CNT % pos_sequence_len;
			}
			if(use_vel_PID){
				des_velocity = vel_sequence[sequencer_CNT];
				sequencer_CNT++;
				sequencer_CNT = sequencer_CNT % vel_sequence_max_len;
			}
			TIM1_ov_cnt = 0;
		}
		//readEnc();
		/*if(use_vel_PID){
			I_d_rqst = vel_PID(des_velocity, ang_velocity);
		}

		else if(use_pos_PID){
			I_d_rqst = pos_PID(des_position, uhel_abs);
		}*/
		/*if(USE_OPEN_LOOP == 0){
			CLOSED_LOOP_MAIN();
		}

		else{
			SVPWM(SVPWM_MOD_AMP,SVPWM_MOD_ANG);
		}*/
		//LL_EXTI_GenerateSWI_0_31(LL_EXTI_LINE_0);
		TIM1_OV_FLAG = 0;
		dataH[plot_data_index] = TIM1->CNT;
	}
	if(rec_fail == 1){
		for(uint8_t i = 0; i < 40; i++){
			if(Recieved_data[i] != 0){LL_USART_TransmitData8(USART2, Recieved_data[i]);}
			Recieved_data[i] = 0;
			UART_delay_func();
		}
		Transmit("\r\n");
		rec_fail = 0;
	}
	else if(rec_ack == 0){
		recieve_eval();
	}
	if(plot == 4){plot_tx();}
	//if((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_READY) != 0UL) {
	//HAL_ADC_StartSampling(&hadc1);

	//}


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0   ------> ADC1_IN1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 interrupt Init */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /* USER CODE BEGIN ADC1_Init 1 */
  //NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),6, 6));
  //NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM6_TRGO;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC1, 0);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
  LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_4, LL_ADC_OVS_SHIFT_NONE);
  LL_ADC_SetOverSamplingDiscont(ADC1, LL_ADC_OVS_REG_CONT);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_DUAL_REG_SIMULT;
  ADC_CommonInitStruct.MultiDMATransfer = LL_ADC_MULTI_REG_DMA_EACH_ADC;
  ADC_CommonInitStruct.MultiTwoSamplingDelay = LL_ADC_MULTI_TWOSMP_DELAY_2CYCLES;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
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

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**ADC2 GPIO Configuration
  PA1   ------> ADC2_IN2
  PC4   ------> ADC2_IN5
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* ADC2 interrupt Init */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /* USER CODE BEGIN ADC2_Init 1 */
  //NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),6, 6));
  //NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC2, 0);
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
  LL_ADC_ConfigOverSamplingRatioShift(ADC2, LL_ADC_OVS_RATIO_4, LL_ADC_OVS_SHIFT_NONE);
  LL_ADC_SetOverSamplingDiscont(ADC2, LL_ADC_OVS_REG_CONT);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC2);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC2);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC2_Init 2 */

  // ADC1 is already initialized MX_ADC1_Init
  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
  LL_ADC_EnableIT_EOC(ADC1);
  UART_delay_func();
  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);
  while (!(LL_ADC_IsActiveFlag_ADRDY(ADC1) && LL_ADC_IsActiveFlag_ADRDY(ADC1))){}
  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_REG_StartConversion(ADC1);
  LL_ADC_REG_StartConversion(ADC2);

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration
  PA4   ------> SPI1_NSS
  PA5   ------> SPI1_SCK
  PB4   ------> SPI1_MISO
  PB5   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  NVIC_SetPriority(TIM1_CC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_CC_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 12800;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 100;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.CompareValue = 200;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.CompareValue = 10000;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC3REF);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_ENABLE);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1279;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim6, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART2);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
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
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOG);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOG, LL_GPIO_PIN_10);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_2);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void USER_EXTI_Init(void){
	LL_EXTI_InitTypeDef exti;
	exti.Line_0_31 = LL_EXTI_LINE_0;
	exti.Mode = LL_EXTI_MODE_IT;
	exti.LineCommand = ENABLE;
	exti.Trigger = LL_EXTI_TRIGGER_RISING;

	NVIC_SetPriority(EXTI0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
	NVIC_EnableIRQ(EXTI0_IRQn);

	LL_EXTI_Init(&exti);


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
