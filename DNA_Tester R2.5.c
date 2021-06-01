//DNA Tester release R2.4
//Update per acutal PCB layout
//Adding Bluetooth Write - R1.0
//Adding Bluetooth Read - R1.2
//During start-up phase, turn off heater by init PWM first, then adding delay at start-up before enabling BOD - r1.4
//Change the UART baud rate from 9600 to 38400 to decrease the MCU resource from 30ms to 10ms in UART_Write -r1.5
//r1.6 changes
//Add temp offset error (temp sensor & real chamber difference); affect calculation of display_temp & user_temp_adc
//Heater status will follow mobile heater_command bit to change other than turning on when measure_command bit is ON
//r1.6.1 changes
//Add flag "rising_temp_reached" to allow :
// - the system to use 100% PWM for the heater in the 1st heater cycle;
// - the system to turn on HEATER LED only if target temp is reached for once, not only when the temp is within tolerance
//r1.6.2
// - correct LED flashing and temperature thresholds for PWM max, min values
//r1.7
// - Pack the data packet into one single packet: COMMAND TYPE = 0; length = 'E' (=14)
// - Data sequence is: laserA, laserB, temp, status (packed in 1byte)
// - It will be sent in the routine LED_status_update
//r1.7.1
// - bug fix on status data array number
//r1.7.2
// - bug fix on status data to handle temp_ok flag which value will be of two bits: 0,1,2
// - Change TX_NACK_count from 2 to 5 to avoid BT LED flash too frequently
//r1.7.3
// - rising_temp_reached = 1 is checked using the averaged temp, not instantaneous temp
// - Correct ADC_CLK divider to 7
//r.1.7.4
// - correct rising temp bug, correct TEMP_ERROR_COUNT routine to limit the max value
// - correct status bit "laser on" to mobile
// - correct fatal bugs in bluetooth_write and generation of data packet
// - status byte changed to 2-byte data in bluetooth_write
//r1.7.4-test
// - temp_offset always = 0 for testing/evaluation of raw temp sensor
//r2.0
// - Modify for the 6-capsule version and temporary for EV board
// - Major impacts:
// - UART rate change from 38400 to 115200
// - ADC0-5 : corresponding to laser detectors; ADC6 : temp sensor
// - Data packet size increase from 18 to 34; length defines changed from character to decimal
// - IO are modified to fit the EV board instead of the actual product

//r2.1-T
// - Modify the IO to fit the final schematics
// - VREF = 3.3V
// - Release PC0,1,2,3 as debug LEDs
// - Temp offset = 0 for testing
// - Disable BOD for large current test

//r2.2
// - Enable BOD
// - Correct the temp sensor detection bug; need to restart a new ADC for each sensor
// - Add comments about temp_offset calculation

//r2.2-T
// - remove temp_offset

//r2.3
// - modify temp_offset equation due to new 6 capsule device

//r2.4
// - Add laser adjustment factor

//2.5
// - Limit the maximum laser value = 3300 after adjustment factor


//Use TMR0 timer interrupt to do regular routines

//PWM parameters are set with resolution at 0.1% (range = 0-1000) after modification of BPWM.c and PWM.c in the lower layer

//Interrupts:
//BOD (Brown-out detector) 			- LVL0 (highest)
//Key interrupts (GPIO interrupts) 	- LVL1
//UART1 RX interrupt from BT		- LVL1
//TMR0 Interrupt (Timer0)			- LVL2

#include "NUC131.h"

//PORT NAME ASSIGNMENT

//PA0 (ADC0) : Laser A1 detection
//PA1 (ADC1) : Laser B1 detection
//PA2 (ADC2) : Laser A2 detection
//PA3 (ADC3) : Laser B2 detection
//PA4 (ADC4) : Laser A3 detection
//PA5 (ADC5) : Laser B3 detection
//PA6 (ADC6) : Temperature sensor detection
//PA7 (VREF) : External VDDA
//PA12 : POWER LED
//PA13 : HEATER LED
//PA14 : LASER Indicator LED
//PA15 : BT LED
//PB0 (SENSOR_A): Thermal sensor select A
//PB1 (SENSOR_B): Thermal sensor select B
//PC7 (KEY1) : WDT test key
//PC6 (KEY2) : HEATER ON/OFF key
//PF8 (KEY3) : Reserved
//PB8 (KEY4) : BOX_OPEN
//PF4 (PWM)  : HEATER PWM control
//PB12 : LASER_LED (laser module)
//UART1: BT connection

//DEBUG LEDs
//PC0,1,2,3

//Test Points
//PD6, 14

#define HEATER_LED 		PA13
#define LASER_INDICATOR PA14
#define BT_LED 			PA15
#define POWER_LED 		PA12
#define LASER_LED 		PB12
#define BOX_OPEN_DETECT PB8
#define HEATER_KEY 		PC6
#define THERMAL_SENSE_A PB0
#define THERMAL_SENSE_B PB2

//Timer Fixed Parameters
#define TIMER_PERIOD 		50 // Timer0 period in ms
#define PID_TIMER_COUNT 	100// unit in msec, Divisible by TIMER PERIOD
#define LED_TIMER_COUNT 	500// unit in msec, Divisible by TIMER PERIOD
#define LASER_TIMER_COUNT 	500// unit in msec, Divisible by TIMER PERIOD

//HEATER fixed parameters
//Temperature = T / VREF x 4096
//VREF for ADC = 3.3V
#define ROOM_TEMP 			25 		//in degC
#define TEMP_ERROR_LIMIT 	60 		//in unit of LED_TIMER_COUNT
#define DEFAULT_TEMP		65 		//in degC
//VREF = 3.3V
#define VREF				3300	//in mV
#define DEFAULT_TEMP_OFFSET	20		//Temp sensor - Temp chamber = -2degC @ temp set at 65degC
//Define heater PWM max & min values per different temp settings to minimize temp variation around the target temp
//This is to be fine-tune after construction of the casing & heater block
#define HEATER_MAX_65deg 	1000
#define HEATER_MAX_45deg 	800
#define HEATER_MAX_35deg 	500
#define HEATER_MIN_65deg 	300
#define HEATER_MIN_45deg 	100
#define HEATER_MIN_35deg 	0

//Communications related
#define COMMAND_LASER 		0
#define COMMAND_TEMP		1
#define COMMAND_STATUS		2

//Heater variables with initialization
short int	heater_pwm = 900;
short int 	heater_pwm_max = 900;
short int 	heater_pwm_min = 0;
short int	mini_pwm_per_temp = 100;
int 		current_temp_adc = ROOM_TEMP * 10*4096/VREF;
int 		framed_temp_adc = 0;
int 		average_temp_adc = 0;
int 		display_temp = ROOM_TEMP;
short int 	temp_error_count = 1;
float 		temp_offset = 0; //Temp sensor - Temp chamber = -2degC when temp is set at 65degC
int 		user_temp_adc = ((DEFAULT_TEMP * 10)- DEFAULT_TEMP_OFFSET)*4096/VREF;
uint8_t		rising_temp_reached = 0;


//Laser variables with initialization
short int 	current_laser_1A = 0;
short int 	current_laser_1B = 0;
int 		framed_laser_1A = 0;
int 		framed_laser_1B = 0;
int 		average_laser_1A = 0;
int 		average_laser_1B = 0;
int 		display_laser_1A = 0;
int 		display_laser_1B = 0;

short int 	current_laser_2A = 0;
short int 	current_laser_2B = 0;
int 		framed_laser_2A = 0;
int 		framed_laser_2B = 0;
int 		average_laser_2A = 0;
int 		average_laser_2B = 0;
int 		display_laser_2A = 0;
int 		display_laser_2B = 0;

short int 	current_laser_3A = 0;
short int 	current_laser_3B = 0;
int 		framed_laser_3A = 0;
int 		framed_laser_3B = 0;
int 		average_laser_3A = 0;
int 		average_laser_3B = 0;
int 		display_laser_3A = 0;
int 		display_laser_3B = 0;

float laser_factor[6] = {1,1,1,1,1,1};

//System Status flags with initialization
uint8_t 	status_power_ok = 1; // 1: VDD > 4.4V (5V_DC > 4.7V)
uint8_t 	status_box_open = 0; // 1: box is open
uint8_t 	status_heater = 1; // 1: heater is on
uint8_t 	status_laser = 0; // 1: laser status is on
uint8_t 	status_bluetooth = 0; //0: linking/searching; 1: linked
uint8_t 	status_temp_ok = 0;
uint8_t 	WDT_flag = 0;

//Timer variables
int 		ticks_per_sec = 1000/TIMER_PERIOD;
uint8_t 	LED_timer = 1; // initial value
uint8_t 	LASER_timer = 1; // initial value
uint8_t 	LASER_average_timer = 1; // initial value
uint8_t 	TEMP_timer = 1; // initial value
uint8_t 	PID_timer = 1; // initial value


//UART Parameter
uint8_t 	UART_RX_Byte = 0;
uint8_t		RX_data_packet[10]={};
uint8_t		g_UART1_RX_Count = 0;
uint8_t		g_UART1_RX_Byte = 0;
uint8_t 	g_UART1_RX_Status = 2;

//BT Communications variables
char 		output_string[4]={};
char		g_TX_data_buffer[31]={};
uint8_t 	RX_data_valid_ACK=0;
uint8_t		TX_data_valid_ACK=0;
uint8_t		TX_NACK_count=0;

//Create a wait state loop function in micro second
void wait(int time_0_5us)
{
	int aaa=0;
	for (aaa=0; aaa<time_0_5us; aaa++)
	{}
}

//=================================================================
//                    PID parameters
// Must be global as they must not be reset in each PID calculation
//=================================================================
float  	Kp				= 0.15;
float 	Ki				= 0.01;//not used
float  	Kd				= 1;
short int 	last_error	= 0;
short int 	update_heater_pwm 	= 900;
short int 	new_heater_pwm 		= 0;
float   temp_heater_pwm 		= 900;
//=================================================================
//     ***********PID calculation**************
//=================================================================
short int PID_Calcul(int currentPWM,int last_Err,short int current_temp_adc,short int target_temp_adc)
{
    short int current_error=0;
    short int derivative=0;

	current_error=target_temp_adc-current_temp_adc;
    derivative=current_error-last_Err;
    temp_heater_pwm=currentPWM+(Kp*current_error)+(Kd*derivative);
    update_heater_pwm=temp_heater_pwm;

    // Update the current error as last_error for the next calculation
	// Note last_error is a global variable
	last_error = current_error;

	//Limit the heater PWM not to exceed it physical limits to avoid damage
	//ADC (VREF = 3.3V) for 35degC = 424 after offset adjustment
	//ADC (VREF = 3.3V) for 45degC = 543 after offset adjustment
	if (target_temp_adc > (558-15))
    {
    	heater_pwm_min = HEATER_MIN_65deg;							//46 to 65degC with offset
		if (rising_temp_reached==0)
			heater_pwm_max = 1000;
		else
			heater_pwm_max = HEATER_MAX_65deg;

    }
    else if ((target_temp_adc <= (558-15)) && (target_temp_adc >= (434-10)))	//35 to 45degC with offset
    {
    	heater_pwm_min = HEATER_MIN_45deg;
		if (rising_temp_reached==0)
			heater_pwm_max = 1000;
		else
			heater_pwm_max = HEATER_MAX_45deg;
    }
    else
    {
    	heater_pwm_min = HEATER_MIN_35deg;
		if (rising_temp_reached==0)
			heater_pwm_max = 1000;
		else
			heater_pwm_max = HEATER_MAX_35deg;
    }

	if(update_heater_pwm > heater_pwm_max)
    {
        update_heater_pwm = heater_pwm_max;
    }
    else if(update_heater_pwm < heater_pwm_min)
    {
        update_heater_pwm = heater_pwm_min;
    }
	return update_heater_pwm;
}


// Brown-out detector interrupt routine
// Brown-out related routines are inside sys.c and sys.h
// This will change status flags
void BOD_IRQHandler(void)
{
	SYS_UnlockReg();//Disable Write-protection for the critical registers
	SYS_CLEAR_BOD_INT_FLAG();
    SYS_LockReg();//Enable Write-protection for the critical registers

	status_power_ok = 0; //power failure!!
	//Set suitable flag values to disable heater & laser
	status_heater = 0;
	status_laser = 0;
}


void UART1_IRQHandler(void)
{
    uint32_t		u32IntSts= UART1->ISR;
    uint8_t     	t_UART1_RX_Buffer[1] = {0};
    unsigned int 	t_UART1_RX_Byte      = 0;

    if(u32IntSts & UART_IS_RX_READY(UART1))
    {
       	UART_Read(UART1, t_UART1_RX_Buffer, 1);
       	t_UART1_RX_Byte = (unsigned int)t_UART1_RX_Buffer[0];
       	g_UART1_RX_Byte = t_UART1_RX_Byte;
       	//UART_Write(UART1, t_UART_00_RX_Result_Buffer, 1);

       	if(g_UART1_RX_Byte==0x41)
       	{
       		g_UART1_RX_Status 	= 0; 		//START rec'd
       		g_UART1_RX_Count 	= 0;
       		RX_data_packet[0]= g_UART1_RX_Byte;
       	}
       	else if(g_UART1_RX_Byte==0x42)
       	{
       		if(g_UART1_RX_Status==1)
       		{
       			g_UART1_RX_Status	= 2;	//STOP rec'd
       			//RX_data_packet[g_UART1_RX_Count] = g_UART1_RX_Byte;
       			//UART_Write(UART1, g_UART_00_RX_Result_Buffer, 1);

				//Data verification by checking the data length & ACK
				//RX_data_packet[1] is the data length of the packet
				if ((RX_data_packet[1]-0x30)==(g_UART1_RX_Count-1))
				{
					//check input temperature from mobile is in the valid range
					int temp_input = 0;
					temp_input = (RX_data_packet[2]-0x30)*10 + (RX_data_packet[3]-0x30);
					if ((temp_input<=65) && (temp_input>=35))
					{
					//Heater status from the 4th data packet byte
					//Update heater status only if it is '1' or '0'
						if (((RX_data_packet[4]-0x30)==0)||((RX_data_packet[4]-0x30)==1))
						{
							//Measurement start/stop command from the 4th byte
							//Update status only if it is '1' or '0'
							//Send valid ACK only if the data byte checkings are correct (length ok, data values are '0' or '1')
							status_heater = RX_data_packet[4]-0x30; //update heater status according to mobile data

							if ((RX_data_packet[5]-0x30)==1) //Measurement is turned ON
							{
								//Calculate the user setting temp adc value from 2nd & 3rd data packet bytes
								//update user_temp_adc for PID

								//temp_offest is temperature dependent and hence should be calculated using equation
								temp_offset = 0.954*temp_input-21.1;
								//temp_offset =0;

								//User_temp_adc is then found with calculated temp_offset
								user_temp_adc = ((temp_input*10)-temp_offset)*4096/VREF;
								//mini_pwm_per_temp = 500-(65-temp_input)*12;
								//status_heater = 1;			//turn on heater status
								status_laser = 1; 			//turn on laser status
								RX_data_valid_ACK = 1;		//RX ACK back to mobile
								TX_data_valid_ACK = 1; 		//Set this flag as to indicate the previous TX data from Main unit is rec'd
							}
							else if ((RX_data_packet[5]-0x30)==0) //Measurement is turned OFF
							{
								//Calculate the user setting temp adc value from 2nd & 3rd data packet bytes
								//update user_temp_adc for PID

								//temp_offest is temperature dependent and hence should be calculated using equation
								temp_offset = 0.954*temp_input-21.1;
								//temp_offset = 0;
								//User_temp_adc is then found with calculated temp_offset
								user_temp_adc = ((temp_input*10)-temp_offset)*4096/VREF;
								status_heater = RX_data_packet[4]-0x30; //update laser status
								status_laser = 0; 			//turn off laser status
								RX_data_valid_ACK = 1;		//RX ACK back to mobile
								TX_data_valid_ACK = 1; 		//Set this flag as to indicate the previous TX data from Main unit is rec'd
							}
							else
								RX_data_valid_ACK = 0;		//RX ACK not valid if Measurement status byte error
						}
						else
							RX_data_valid_ACK = 0;			//RX ACK not valid if Heater status byte error
					}
					else
							RX_data_valid_ACK = 0;			//RX ACK not valid if Temp input value error
				}
				else
					RX_data_valid_ACK = 0;				//RX ACK not valid if data length error
       		}
       		else
       			RX_data_valid_ACK = 0;					//RX ACK not valid if STOP byte received but UART1_RX_Status != 1
       	}
       	else
       	{
       		if((g_UART1_RX_Status==0)||(g_UART1_RX_Status==1))
       		{
       			g_UART1_RX_Status 	= 1;//Command Type and User Data inside the data packet
       			g_UART1_RX_Count++;
       			RX_data_packet[g_UART1_RX_Count] = g_UART1_RX_Byte;
       		}
       	}
    }
}




//Key interrupt service routine
//This will change status flags
void GPCDEF_IRQHandler(void)
{
	// Check if PC.06 KEY2(HEATER_KEY) interrupt occurred
	if (GPIO_GET_INT_FLAG(PC, BIT6))
	{
        GPIO_CLR_INT_FLAG(PC, BIT6);
        if (status_power_ok == 1) //Mask the key if power is failure
        	status_heater = !status_heater;
	}
	// Check if PB.07 WDT Test KEY1 interrupt occurred
	else if (GPIO_GET_INT_FLAG(PC, BIT7))
    {
        	GPIO_CLR_INT_FLAG(PC, BIT7);
            WDT_flag = !WDT_flag; //toggle the WDT test flag
    }
	else
    {
    // Un-expected interrupt. Just clear all PC, PD, PE and PF interrupts
        PC->ISRC = PC->ISRC;
        PD->ISRC = PD->ISRC;
        PE->ISRC = PE->ISRC;
        PF->ISRC = PF->ISRC;
    }
}

void GPAB_IRQHandler(void)
{

    // Un-expected interrupt. Just clear all PA & PB interrupts
        PA->ISRC = PA->ISRC;
        PB->ISRC = PB->ISRC;
}


//Conversion from 4-digit decimal integer to 4 data bytes in ASCII format
void dec_to_char(int four_digit_input)
{
	short int	output_digit[4] = {};

	output_digit[0]	= (four_digit_input)/1000; 						//get the thousandth digit
	output_digit[1]	= (four_digit_input-output_digit[0]*1000)/100 ; 	//get the hundredth digit
	output_digit[2]	= (four_digit_input-output_digit[0]*1000-output_digit[1]*100)/10; 	//get the tenth digit
	output_digit[3]	= (four_digit_input-output_digit[0]*1000-output_digit[1]*100-output_digit[2]*10); //get the unit digit

	//====================================================================================
	//Conversion to string
	for (int i=0; i<4; i++)
	{
		output_string[i] = output_digit[i] 	+0x30;
	}
	//====================================================================================
}


//A complete Command Write to BT SPP
void Bluetooth_write (uint8_t command_type, uint8_t length, char input_string[])
{
	char	data_packet[34]={};

	//====================================================================================
	//Construct the data packet
	data_packet[0]  	= 0x41;					// Start Byte 'A'
	//data_packet[1]  	= 'E';					// User-Data Length = 14 in ASCII, excluding Command byte
	data_packet[1]		= 0x1F;					// User-Data Length = 31, excluding Command byte; value = 31(d)
	data_packet[2]  	= command_type + 0x30;	// Command type in ASCII

	for (uint8_t index=3; index<(length+2); index++)
	{
		data_packet[index]  = input_string[index-3];	// fill in User Data Byte 00 to length
	}
	data_packet[length+2] 	= 0x42;				// Stop Byte 'B'

	//====================================================================================
	// Write the data packet to Bluetooth
	// Data packet size is length+3 [command(=1)+data(=13)+length(=1)+start(=1)+stop(=1)]
	UART_Write(UART1, data_packet, (length+3));

	//Reset ACK from Mobile to 0 and wait for next packet from Mobile
	TX_data_valid_ACK=0;
}



//Timer0 Interrupt Service Routine
void TMR0_IRQHandler(void)
{
    // Clear Timer0 time-out interrupt flag
    TIMER_ClearIntFlag(TIMER0);

    // LED5 for Timer Interrupt Event
    PC0 = 0;

    //Reset WDT to re-count again
    if (WDT_flag==0)
    	{
    	SYS_UnlockReg();//Disable Write-protection for the critical registers
    	WDT_RESET_COUNTER();
    	SYS_LockReg();//Enable Write-protection for the critical registers
    	}
    //Read the temperature & laser values from the sensors with temp sensor1
	THERMAL_SENSE_A=0; THERMAL_SENSE_B=0; wait(50);//Select sensor1
    ADC_START_CONV(ADC);
    //Wait conversion done (ADF flag = 1)
    while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT)); //max = 1.5us
    // Clear the A/D interrupt flag for safe
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
	current_laser_1A = ADC_GET_CONVERSION_DATA(ADC,0);
	current_laser_1B = ADC_GET_CONVERSION_DATA(ADC,1);
	current_laser_2A = ADC_GET_CONVERSION_DATA(ADC,2);
	current_laser_2B = ADC_GET_CONVERSION_DATA(ADC,3);
	current_laser_3A = ADC_GET_CONVERSION_DATA(ADC,4);
	current_laser_3B = ADC_GET_CONVERSION_DATA(ADC,5);
    current_temp_adc = ADC_GET_CONVERSION_DATA(ADC,6);

    //Read the temperature & laser values from the sensors with temp sensor2
	THERMAL_SENSE_A=1; THERMAL_SENSE_B=0; wait(50);//Select sensor2
    ADC_START_CONV(ADC);
    //Wait conversion done (ADF flag = 1)
    while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT)); //max = 1.5us
    // Clear the A/D interrupt flag for safe
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
    current_temp_adc += ADC_GET_CONVERSION_DATA(ADC,6);
    current_temp_adc = current_temp_adc/2;// Take the average of the two sensors


    //**************************************************************************
    //Call the routine to do PID based on PID timer
    //Call PID if status_heater is on
	//Turn off heater_pwm if status_heater is off
    if (PID_timer == PID_TIMER_COUNT/TIMER_PERIOD)  //PID timer reached
    {
    	if (status_heater ==1)
    	{
    	    PC2=0;//ON LED for PID
    	    new_heater_pwm = PID_Calcul(heater_pwm,last_error,current_temp_adc,user_temp_adc);
    		heater_pwm = new_heater_pwm;
    		PC2=1;//OFF LED for PID
    	}
    	else
    		heater_pwm = 0;
    	//**********************************************************************************
		//Reset PID timer
    	PID_timer = 1;
	}
	else
	{
		PID_timer++;
    }


    //Averaging the temp with LED_TIMER_COUNT so that the temp is averaged over the period of status/display updates
    //Use Averaged_temp for temp_ok flag detection
    average_temp_cal();

    //Averaging the laser detector values with LASER_TIMER_COUNT so that the laser values are averaged over the period of laser values updates
    average_laser_cal();

    //Turn on the laser according to the laser status flag and box status flag
	if ((status_laser == 1) && (status_box_open ==0))
	{
		LASER_LED = 1;
		PC3 = 0; //emulates LASER LED
	}
	else
	{
		LASER_LED = 0;
		PC3 = 1; //emulates LASER LED1
	}
	LASER_read();
	LED_status_update();
	PC0=1; //OFF LED for TMR0
}


//Average Temp is calculated
void  average_temp_cal (void)
{

	if (TEMP_timer == LED_TIMER_COUNT/TIMER_PERIOD) //LED & Temperature average time is the same
	{
		//Calculate average temp over the LED_TIMER_COUNT period
		//Update the status_temp_ok flag based on this average value
		framed_temp_adc = framed_temp_adc + current_temp_adc;
		average_temp_adc = framed_temp_adc / TEMP_timer;

		//Temp rise reached for the 1st time FLAG indication
		//MAX_PWM is set at 100% if negative, else follow different MAX_PWM values
		if (average_temp_adc > user_temp_adc)
			rising_temp_reached = 1;

		//Update status_temp_ok flag if within +/-5%
		if (abs(average_temp_adc-user_temp_adc)<=(user_temp_adc*5/100))
			status_temp_ok = 1;
		else
			status_temp_ok = 0;

		//Failure heater circuit check
		if (average_temp_adc < 0x3E)
		{
			temp_error_count++;
		}
		else
		{
			temp_error_count = 1; //Reset counter if temp becomes normal
		}
		if (temp_error_count >= TEMP_ERROR_LIMIT)
		{
			status_temp_ok = 2; //Report temperature circuit error
			status_heater = 0; //Turn off the heater for safety
			temp_error_count = TEMP_ERROR_LIMIT; // Limit the max temp error count
		}

		//************************************************************

		//Pass the average temp to display
		//Display temp = Real Temp x 10 degC (4 SIG FIGs)
		//RESET Temperature variables
		TEMP_timer = 1;
		display_temp = average_temp_adc * VREF/4096 + temp_offset;
		framed_temp_adc = 0;
		average_temp_adc = 0;
	}
	else
	{
		framed_temp_adc = framed_temp_adc + current_temp_adc;
		TEMP_timer++;
	}
}


//Average Laser is calculated
void  average_laser_cal (void)
{

	if (LASER_average_timer == LASER_TIMER_COUNT/TIMER_PERIOD) //LASER Read timer & laser average time is the same
	{
		//Calculate average laser over the LASER_TIMER_COUNT period
		//Update the status_temp_ok flag based on this average value
		framed_laser_1A = framed_laser_1A + current_laser_1A;
		average_laser_1A = framed_laser_1A / LASER_average_timer;
		framed_laser_1B = framed_laser_1B + current_laser_1B;
		average_laser_1B = framed_laser_1B / LASER_average_timer;

		framed_laser_2A = framed_laser_2A + current_laser_2A;
		average_laser_2A = framed_laser_2A / LASER_average_timer;
		framed_laser_2B = framed_laser_2B + current_laser_2B;
		average_laser_2B = framed_laser_2B / LASER_average_timer;

		framed_laser_3A = framed_laser_3A + current_laser_3A;
		average_laser_3A = framed_laser_3A / LASER_average_timer;
		framed_laser_3B = framed_laser_3B + current_laser_3B;
		average_laser_3B = framed_laser_3B / LASER_average_timer;
		//************************************************************

		//Pass the average laser values to display in UART later
		//Use laser correction factor to calibrate
		//Laser detector value will be display in XXXX mV (4 SIG FIG)
		LASER_average_timer = 1;
		display_laser_1A = average_laser_1A * VREF/4096/laser_factor[0];
		if (display_laser_1A > 3300) display_laser_1A = 3300;
		display_laser_1B = average_laser_1B * VREF/4096/laser_factor[1];
		if (display_laser_1B > 3300) display_laser_1B = 3300;
		display_laser_2A = average_laser_2A * VREF/4096/laser_factor[2];
		if (display_laser_2A > 3300) display_laser_2A = 3300;
		display_laser_2B = average_laser_2B * VREF/4096/laser_factor[3];
		if (display_laser_2B > 3300) display_laser_2B = 3300;
		display_laser_3A = average_laser_3A * VREF/4096/laser_factor[4];
		if (display_laser_3A > 3300) display_laser_3A = 3300;
		display_laser_3B = average_laser_3B * VREF/4096/laser_factor[5];
		if (display_laser_3B > 3300) display_laser_3B = 3300;

		//*************************************************************

		//Reset all the laser values
		framed_laser_1A = 0;
		framed_laser_1B = 0;
		average_laser_1A = 0;
		average_laser_1B = 0;

		framed_laser_2A = 0;
		framed_laser_2B = 0;
		average_laser_2A = 0;
		average_laser_2B = 0;

		framed_laser_3A = 0;
		framed_laser_3B = 0;
		average_laser_3A = 0;
		average_laser_3B = 0;

		//*************************************************************
	}
	else
	{
		framed_laser_1A = framed_laser_1A + current_laser_1A;
		framed_laser_1B = framed_laser_1B + current_laser_1B;
		framed_laser_2A = framed_laser_2A + current_laser_2A;
		framed_laser_2B = framed_laser_2B + current_laser_2B;
		framed_laser_3A = framed_laser_3A + current_laser_3A;
		framed_laser_3B = framed_laser_3B + current_laser_3B;

		LASER_average_timer++;
	}
}


void LASER_read(void)
{
	char data_buffer[24] ={};

	//***********************Send the Laser ADC values************************************
	//Send averaged laser detector values to UART

	if (LASER_timer == LASER_TIMER_COUNT/TIMER_PERIOD)
	{
		//Convert Laser 1A ADC to characters
		dec_to_char(display_laser_1A);
		for (int i=0; i<4; i++)
		{
			data_buffer[i] = output_string[i];
			g_TX_data_buffer [i] = output_string[i];//save to global buffer
		}

		//Convert Laser 1B ADC to characters
		dec_to_char(display_laser_1B);
		for (int i=0; i<4; i++)
		{
			data_buffer[i+4] = output_string[i];
			g_TX_data_buffer [i+4] = output_string[i];//save to global buffer
		}

		//Convert Laser 2A ADC to characters
		dec_to_char(display_laser_2A);
		for (int i=0; i<4; i++)
		{
			data_buffer[i+8] = output_string[i];
			g_TX_data_buffer [i+8] = output_string[i];//save to global buffer
		}

		//Convert Laser 2B ADC to characters
		dec_to_char(display_laser_2B);
		for (int i=0; i<4; i++)
		{
			data_buffer[i+12] = output_string[i];
			g_TX_data_buffer [i+12] = output_string[i];//save to global buffer
		}

		//Convert Laser 3A ADC to characters
		dec_to_char(display_laser_3A);
		for (int i=0; i<4; i++)
		{
			data_buffer[i+16] = output_string[i];
			g_TX_data_buffer [i+16] = output_string[i];//save to global buffer
		}

		//Convert Laser 3B ADC to characters
		dec_to_char(display_laser_3B);
		for (int i=0; i<4; i++)
		{
			data_buffer[i+20] = output_string[i];
			g_TX_data_buffer [i+20] = output_string[i];//save to global buffer
		}

		//Reset laser timer
		LASER_timer = 1;
	}
	else
		LASER_timer++;
	//**********************************************************************************
}


void LED_status_update(void)
{
	//Update LED according to status flags
	//Send average temperature and status flags to UART

	//uint8_t status[7]={};
	uint8_t status1 = 0;
	uint8_t status2 = 0;
	char 	data_buffer[7]={};

	//Update LED status and handle LED flashing in sync manner

	//Heater Indicator Updates
	if (status_heater==1)
	{
		if ((status_temp_ok == 1) && (rising_temp_reached == 1))
			HEATER_LED = 0; // Turn on HEATER LED if heater is on and target temp is within tolerance AND target temp is reached for once already

		//Slow flashing if temp is not within tolerance or target temp is not yet reached for once
		else if (((status_temp_ok == 0) || (rising_temp_reached == 0)) && (LED_timer > (LED_TIMER_COUNT/TIMER_PERIOD/2)))
			HEATER_LED = 1; //Turn off the LED in the 1st half
		else if (((status_temp_ok == 0) || (rising_temp_reached == 0)) && (LED_timer <= (LED_TIMER_COUNT/TIMER_PERIOD/2)))
			HEATER_LED = 0; //Turn on the LED in the next half
	}
	//Case for temp circuit failure!
	//Fast Flashing if failure!!
	else if ((status_heater==0) && (status_temp_ok==2))
	{
		if (LED_timer > (LED_TIMER_COUNT/TIMER_PERIOD*3/4))
		{
			HEATER_LED = 0; //OFF for 4th quarter
		}
		else if (LED_timer > (LED_TIMER_COUNT/TIMER_PERIOD/2))
		{
				HEATER_LED = 1; //ON for 3rd quarter
		}
		else if (LED_timer > (LED_TIMER_COUNT/TIMER_PERIOD/4))
		{
				HEATER_LED = 0; //ON for 2nd quarter
		}
		else
		{
				HEATER_LED = 1; //OFF for 1st quarter
		}
	}
	else
		HEATER_LED = 1; // OFF if status_heater = 0 & status_temp_ok !=2

	//******************************************************************************
	//Bluetooth status checking
	if (TX_data_valid_ACK==0)
		TX_NACK_count++;
	else
		TX_NACK_count=0;

	if (TX_NACK_count >=5)
	{
		status_bluetooth = 0;	//not linked if not ACK from mobile for two LED Timers
		TX_NACK_count = 5;		//limit the max value of TX_NACK counter
	}
	else
		status_bluetooth = 1;

	//BT link Indicator updates
	if (status_bluetooth==1) // BT linked
		BT_LED = 0; //turn on
	else if ((status_bluetooth ==0) && (LED_timer > (LED_TIMER_COUNT/TIMER_PERIOD/2)))
		BT_LED = 1; //Turn off the LED in the 1st half
	else
		BT_LED = 0; //Turn on the LED in the next half

	//Power Indicator updates
	if (status_power_ok==1)
		POWER_LED = 0; //Turn on
	//Fast Flashing the LED if power failure!
	else
		{
		if (LED_timer > (LED_TIMER_COUNT/TIMER_PERIOD*3/4))
			{
				POWER_LED = 0; //OFF for 4th quarter
			}
		else if (LED_timer > (LED_TIMER_COUNT/TIMER_PERIOD/2))
			{
				POWER_LED = 1; //ON for 3rd quarter
			}
		else if (LED_timer > (LED_TIMER_COUNT/TIMER_PERIOD/4))
			{
				POWER_LED = 0; //ON for 2nd quarter
			}
		else
			{
				POWER_LED = 1; //OFF for 1st quarter
			}
		}

	//LASER Indicator status updates
	if ((status_laser==1) && (status_box_open==0))  //laser on
		LASER_INDICATOR = 0; //turn on
	else
		LASER_INDICATOR = 1; //turn off

	//Send status flags to UART every LED_timer cycle
	//Decrement or Reset LED_timer counter
	if (LED_timer == LED_TIMER_COUNT/TIMER_PERIOD) //update LED according to status flag per LED_timer so that flashing can be achieved
		{
		PD14 =0;

		//*****************************************************
		//Report status flags to Bluetooth
		//Pack into 1 single byte to save packet size

		//Status has two bytes and use lower nibble only
		//Status1
		status1 = status1 | status_power_ok;
		status1 = status1 | (status_box_open<<1);
		status1 = status1 | (status_heater<<2);
		status1 = status1 | (((status_laser) && (!status_box_open))<<3);
		//Status2
		status2 = status2 | status_bluetooth;
		status2 = status2 | (status_temp_ok<<1); // this occupies bit 1&2
		status2 = status2 | (RX_data_valid_ACK<<3); //temp_ok is 0,1,2 and so this status bit must be shifted to bit3

		//Convert Temp ADC to characters and send to global display buffer
		dec_to_char(display_temp);
		for (int i=0; i<4; i++)
		{
			data_buffer[i] = output_string[i];
			g_TX_data_buffer[i+24] = output_string[i];
		}

		//Send status byte to global display buffer
		g_TX_data_buffer[28] = status1;
		g_TX_data_buffer[29] = status2;

		//Send out data to Bluetooth with 30-byte data payload + 1 Byte Command
		Bluetooth_write(COMMAND_LASER, 31, g_TX_data_buffer);

		RX_data_valid_ACK = 0; //Clear RX_data_valid_ACK immediately to wait for a new async packet from mobile

		//Reset LED_timer
		LED_timer = 1;
		PD14=1;
		}
	else
		{
		LED_timer++;
		}

}


void SYS_Init(void)
{
	//-------------------------------------------------------------------------------
	// Init System Clock
	//-------------------------------------------------------------------------------

	// Enable Internal RC 22.1184MHz clock
	CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

	// Waiting for Internal RC clock ready
	CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

	// Switch HCLK clock source to Internal RC and HCLK source divide 1
	CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

	// Enable external XTAL 12MHz clock
	CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

	// Waiting for external XTAL clock ready
	CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

	// Set core clock as PLL_CLOCK from PLL
	CLK_SetCoreClock(50000000);

    /* Enable peripheral clock first */
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_EnableModuleClock(PWM1_MODULE);
	CLK_EnableModuleClock(ADC_MODULE);

	/* Peripheral clock source selection then*/
	CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_LIRC, 0);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HCLK, NULL);
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL3_PWM1_S_PCLK, 0);
	// ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz
	CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(7));

    // Reset UART1
    SYS_ResetModule(UART1_RST);

	// Reset PWM1
    SYS_ResetModule(PWM1_RST);

    // Disable the PA0-7 digital input path to avoid the leakage current.
	GPIO_DISABLE_DIGITAL_PATH(PA, 0xFF);

	// Enable Brown-out detector and its interrupt
	// BOD interrupt mode, set at 3.74 - 3.84V threshold

	SYS_EnableBOD(SYS_BODCR_BOD_INTERRUPT_EN,SYS_BODCR_BOD_VL_3_7V);
	SYS_DISABLE_BOD_RST();//Enable Brown-out INT

	//---------------------------------------------------------------------------------------------------------
    // Init I/O Multi-function
    //---------------------------------------------------------------------------------------------------------
    // Set GPF multi-function pins for PWM1 Channel4 (PF4), ICE interface
	// Configure ADC Analog Input Pins
	// Configure external AD VDDA reference
	// Configure UART1
	SYS->VREFCR = 0x00; //access VREFCR to external VREF at PA7
    SYS->ALT_MFP3 = SYS_ALT_MFP3_PF4_PWM1_CH4;
    SYS->ALT_MFP4 = SYS_ALT_MFP4_PA7_Vref;
    SYS->GPA_MFP = SYS_GPA_MFP_PA7_Vref | SYS_GPA_MFP_PA6_ADC6 | SYS_GPA_MFP_PA5_ADC5 | SYS_GPA_MFP_PA4_ADC4 | SYS_GPA_MFP_PA3_ADC3 | SYS_GPA_MFP_PA2_ADC2 | SYS_GPA_MFP_PA1_ADC1 | SYS_GPA_MFP_PA0_ADC0;
    SYS->GPB_MFP = SYS_GPB_MFP_PB5_UART1_TXD | SYS_GPB_MFP_PB4_UART1_RXD;
    SYS->GPF_MFP = SYS_GPF_MFP_PF7_ICE_DAT | SYS_GPF_MFP_PF6_ICE_CLK | SYS_GPF_MFP_PF4_PWM1_CH4;

}


int32_t main(void)
{
	SYS_UnlockReg();//Disable Write-protection for the critical registers
    SYS_Init();
    SYS_LockReg();//Enable Write-protection for the critical registers

    // LED & outputs

    GPIO_SetMode(PC, BIT0, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PC, BIT1, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PC, BIT2, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PC, BIT3, GPIO_PMD_OUTPUT);

	// Laser output
	GPIO_SetMode(PB, BIT12, GPIO_PMD_OUTPUT);

	// Key inputs with pull-ups
    GPIO_SetMode(PC, BIT6, GPIO_PMD_QUASI); //KEY2
    GPIO_SetMode(PC, BIT7, GPIO_PMD_QUASI); //KEY1
    GPIO_SetMode(PF, BIT8, GPIO_PMD_QUASI); //KEY3
    GPIO_SetMode(PB, BIT8, GPIO_PMD_QUASI); //KEY4

    //Test points
    GPIO_SetMode(PD, BIT6, GPIO_PMD_OUTPUT); //TP2
    GPIO_SetMode(PD, BIT14, GPIO_PMD_OUTPUT); //TP

    //Thermal sensor selection outputs
    GPIO_SetMode(PB, BIT0, GPIO_PMD_OUTPUT); //Thermal sensor A
    GPIO_SetMode(PB, BIT2, GPIO_PMD_OUTPUT); //Thermal sensor B

    //IO Initialization
	//Turn on power LED
    POWER_LED = 0;
	//Turn off other LEDs and Laser LEDs
	BT_LED = 1;
	LASER_INDICATOR = 1;
    LASER_LED=0;

    //Thermal sensor selection outputs
    THERMAL_SENSE_A=0; THERMAL_SENSE_B=0;

	PC0 = 1; PC1 = 1; PC2 = 1; PC3 = 1;
    PD6=1;
    PD14=1;

    //*********************HEATER PWM configure******************
	//Make sure HEATER is OFF at early stage during startup
    PWM_ConfigOutputChannel(PWM1, 4, 100, 0); //PF4, PWM1_CH4 Heater off
    PWM_EnableOutput(PWM1, 0x10);
    PWM_Start(PWM1, 0x10);
    //***********************************************************

	//*****************Brown-out INT Set-up*********************
    wait(20000); //wait around 15ms until power is stable during start-up
    PD6=0;
    NVIC_SetPriority(BOD_IRQn,0); //set Brown-out INT priority (=0)
	NVIC_EnableIRQ(BOD_IRQn);
	//**********************************************************

	//*********************Init ADC*****************************
	// Set the ADC operation mode as
	// single-cycle scan mode, input mode as single-end
	// enable the analog input channels, bit0 for CH0, bit1 for CH1 and so on
	ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, 0x7F); // open for ADC0-6
	//ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, 0xFF); // open for ADC0-6
	// Power on ADC module
	ADC_POWER_ON(ADC);

	// Clear the A/D interrupt flag for safe
	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
	//***********************************************************


	// Init UART1
	UART_Open(UART1, 115200);//increased baud rate
	UART_EnableInt(UART1, (UART_IER_RDA_IEN_Msk));//Enable UART1 RX INT
	NVIC_SetPriority(UART1_IRQn,1); //set UART INT priority (=1)
	NVIC_EnableIRQ(UART1_IRQn);

    //Configure the keys to enable interrupt by falling edge trigger
	GPIO_EnableInt(PC, 6, GPIO_INT_FALLING);//Heater on/off key
    GPIO_EnableInt(PC, 7, GPIO_INT_FALLING);//WDT Test key
	NVIC_SetPriority(GPAB_IRQn,1); //set Key INT priority (=1)
	NVIC_SetPriority(GPCDEF_IRQn,1); //set Key INT priority (=1)
	NVIC_EnableIRQ(GPAB_IRQn);
	NVIC_EnableIRQ(GPCDEF_IRQn);

    //Set key & switch debounce
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_1);
	GPIO_ENABLE_DEBOUNCE(PB, BIT8);
    GPIO_ENABLE_DEBOUNCE(PC, BIT6);
	GPIO_ENABLE_DEBOUNCE(PC, BIT7);
	GPIO_ENABLE_DEBOUNCE(PF, BIT8);


	//*********************Init Timer0***************************
	// Open Timer0 in periodic mode, enable interrupt, # of ticks per second
    NVIC_SetPriority(TMR0_IRQn,2); //set TMR0 INT priority (=2)
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, ticks_per_sec);
    TIMER_EnableInt(TIMER0);
    // Enable Timer0 NVIC
    NVIC_EnableIRQ(TMR0_IRQn);

    //WDT set-up and start
    //Timer start and RESET after 1.6sec
    //Need to unlock write-protect
    SYS_UnlockReg();
    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_18CLK, TRUE, FALSE);
    SYS_LockReg();

    // Start Timer0 counting
    TIMER_Start(TIMER0);
	//***********************************************************


    while(1)
    {
    	//Continuous monitoring the box-open switch
    	if (BOX_OPEN_DETECT==1) //box open detected
    	{
    	   	status_box_open = 1;
    	}
    	else
    	{
    		status_box_open = 0; //box is closed
    	}

    	//****************************HEATER CONTROL*****************************************
   		PWM_ConfigOutputChannel(PWM1, 4, 100, heater_pwm);
		//***********************************************************************************
    }

}







