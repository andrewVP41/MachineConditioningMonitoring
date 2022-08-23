/************************************************* 
 ***************************************************
 *
 * FILE NAME  :  MachineMonitoring.c
 * DESCRIPTION:  This program reads acceleration and 
 *               compares the X,Y, and Z acceleration to the given threshold to set the alarm off 
 *               
 *
 **************************************************/

#include "mraa_beaglebone_pinmap.h"
#include <stdio.h>
#include <time.h>
#include <signal.h>
#define LCD_ADDR 		 0x3E
// check the clave address : as of now keeping SA0 =1
#define I2C_SLAVE_ADDR 0x6B
#define CTRL_REG5_XL			0x1F
#define CTRL_REG6_XL			0x20
#define CTRL_REG7_XL			0x21
#define CTRL_REG9           	0x23

#define FIFO_CTRL				0x2E
#define FIFO_SRC				0x2F
#define OUT_X_L_XL           	0x28

// Sensor Sensitivity Constants
// Values set according to the typical specifications provided in
// table 3 of the LSM9DS1 datasheet. (pg 12)
#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732


mraa_i2c_context I2Chandle;
mraa_i2c_context i2cp;
float aRes;
float AccelVal[3];
int16_t aRaw[3];
int16_t ax, ay, az; // x, y, and z axis readings of the Accelerometer
int base = 30000; // 30us base increment
int period = 255; // base*period = 7.65 ms or 130 Hz cycle
int red_duty = 128;
int red_pwm_ch = 1;// set which pwm channel with in a pwmchip
char pwm_per_red[] ="/sys/class/pwm/pwm-2:1/period";
char pwm_duty_red[] ="/sys/class/pwm/pwm-2:1/duty_cycle";
char pwm_enable_red[] ="/sys/class/pwm/pwm-2:1/enable";
char pwm_chip_un0[] = "/sys/class/pwm/pwmchip0/unexport";
char pwm_chip_ex0[] = "/sys/class/pwm/pwmchip0/export";
char pwm_chip_un2[] = "/sys/class/pwm/pwmchip2/unexport";
char pwm_chip_ex2[] = "/sys/class/pwm/pwmchip2/export";

FILE *ppntr,*wpntr,*epntr;
FILE *fp; // file to read the data to





int close_pwm()    // Disable all the PWM's otherwise they will remian active. 
		   // Initilizing active PWM pins will cause a segment fault. 
{
	epntr = fopen(pwm_enable_red,"w");
	if (epntr != NULL)
	{
		fprintf(epntr,"0");
		fclose(epntr);
	}
	epntr = fopen(pwm_chip_un0,"w");
	if (epntr != NULL)
	{
		fprintf(epntr,"0");
		fclose(epntr);
	}
	epntr = fopen(pwm_chip_un0,"w");
	if (epntr != NULL)
	{
		fprintf(epntr,"1");
		fclose(epntr);
	}
       	epntr = fopen(pwm_chip_un2,"w");
	if (epntr != NULL)
	{
		fprintf(epntr,"1");
		fclose(epntr);
	}
}

int set_duty(int r_d)
{ 

	int pulse_w;
	wpntr = fopen(pwm_duty_red,"w");
	if (wpntr == NULL) 
	{
		printf("Can't set red pwm pin pulse width, exiting\n");
		close_pwm();
		return MRAA_ERROR_UNSPECIFIED;
	}
	pulse_w = (int) r_d*base;
	//printf("Red Duty Cycle = %3.1f \%\n",r_d/2.55);
	fprintf(wpntr,"%d",pulse_w);
	fclose(wpntr);

	return MRAA_SUCCESS;
}

int set_period(int per)
{
	int pulse_w;	
	ppntr = fopen(pwm_per_red,"w");
	if (ppntr == NULL) 
	{
		printf("Can't set red pwm pin period, exiting\n");
		close_pwm();
		return MRAA_ERROR_UNSPECIFIED;
	}
	fprintf(ppntr,"%d",per*base);
	fclose(ppntr);

	/*ppntr = fopen(pwm_per_green,"w");
	if (ppntr == NULL) 
	{
		printf("Can't set green pwm pin period, exiting\n");
		close_pwm();
		return MRAA_ERROR_UNSPECIFIED;
	}
	fprintf(ppntr,"%d",per*base);
	fclose(ppntr);
	ppntr = fopen(pwm_per_blue,"w");
	if (ppntr == NULL) 
	{
		printf("Can't set blue pwm pin period, exiting\n");
		close_pwm();
		return MRAA_ERROR_UNSPECIFIED;
	}
	fprintf(ppntr,"%d",per*base);
	fclose(ppntr);*/

	printf("Period set to %5.2f ms\n",(float)per*base/1000000);

	return MRAA_SUCCESS;

}

void exit_signal(int signum)
{
	printf("\nExiting PWM Program \n");
	close_pwm();
	exit(signum);
}

int blink()
{

	mraa_gpio_context button1;
	i2cp = mraa_i2c_init_raw (I2CP_BUS);
	mraa_i2c_frequency (i2cp, MRAA_I2C_STD);
	button1 = mraa_gpio_init(B1);
	int B1_val = mraa_gpio_read(button1);
	mraa_result_t status = MRAA_SUCCESS;
	char buffer[20];
	mraa_init();
	int iten,red_dir,dir_cntr; 
	signal(SIGINT, exit_signal);
	
	// Initialize LED PWM pwmchip2

	ppntr = fopen(pwm_per_red,"r");
	if (ppntr != NULL)
	{
		fclose(ppntr);
		printf("PWM already enabled\n");
	}
	else
	{
		epntr=fopen(pwm_chip_ex2,"w");
		printf("Enabling PWM\n");
		fprintf(epntr,"%d",red_pwm_ch);
		fclose(epntr);
	}
		
	sleep(1); // ************  Note: A pause on enable is required!

	// Now that the PWM chip has been enabled, set the period and duty cycle

	set_period(period);
	set_duty(red_duty);

	// Enable the outputs

	epntr = fopen(pwm_enable_red,"w");
	if (epntr == NULL) 
	{
		printf("Can't enable red pwm pin, exiting\n");
		close_pwm();
		return MRAA_ERROR_UNSPECIFIED;
	}
	fprintf(epntr,"%d",1);
	fclose(epntr);


	// loop forever looping pwm duty cycles.

	int button_val = 0;
	while(!button_val)
	{
	    int B1_val = mraa_gpio_read(button1);

	    if(!B1_val) {
		button_val = 1;
	    }
	    set_duty(0); 			
	    usleep(100000);
	    set_duty(255);
	    usleep(100000);
	 			
	}
	set_duty(255);//stop blinking by setting the LED on
	return MRAA_SUCCESS;
}

void home_LCD (void)
{
        uint8_t buf[2] = {0x00,0x02};
        mraa_i2c_write(i2cp, buf, 2);  //Set to Home
}

void home2_LCD (void)
{
        uint8_t buf[] = {0x00,0x02,0xC0};
        mraa_i2c_write(i2cp, buf, 3);  //Set to Start of 2nd line 0X40 
}

void LCD_Print2 (char* str)
{
        uint8_t buf[80]={0};  // Set Buffer to all Null
        int i = 0, strl;      
        home2_LCD ();
        buf[i] = 0x40;  //register for display
        i++;
        strl = strlen((char*)str);
        for (int j = 0; j < strl; j++)
        {
               buf[i] = str[j];
               i++;

        }
         mraa_i2c_write(i2cp, buf, i);
}

void LCD_Print (uint8_t* str)
{
        uint8_t buf[80]={0};   // Set Buffer to all Null
	uint8_t buf1[2]={0x00,0x80};	
        int32_t i = 0, strl, j=0; 
        buf[i] = 0x40;  //register for display
        i++;
        strl = strlen((char*)str);
        for (j = 0; j < strl; j++)
        {
                buf[i] = str[j];
		i++;
  
        }
        
	mraa_i2c_write(i2cp, buf1, 2);
        mraa_i2c_write(i2cp, buf, i);
}

void clear_LCD (void)
{
        uint8_t buf[2] = {0x00,0x01};
        mraa_i2c_write(i2cp, buf, 2);  //Clear Display
}

void Accelerometer_Init()
{
    // Accelerometer settings
    uint8_t tempRegValue = 0;
    uint8_t sampleRate,highResEnable;
    uint8_t bandwidth;
    uint8_t scale;
    
    //	CTRL_REG5_XL (0x1F) (Default value: 0x38): [DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	
	tempRegValue |= (1<<5);	
	tempRegValue |= (1<<4);
	tempRegValue |= (1<<3);
	
	mraa_i2c_write_byte_data(I2Chandle,tempRegValue,CTRL_REG5_XL);
	
	// CTRL_REG6_XL (0x20) (Default value: 0x00) :[ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	
	tempRegValue = 0;
	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
	sampleRate = 6;

	tempRegValue |= (sampleRate & 0x07) << 5;
	
	// accel scale can be 2, 4, 8, or 16
	scale = 16;
	aRes = SENSITIVITY_ACCELEROMETER_16; // Calculate g / ADC tick, stored in aRes variable
	
	
	if(scale ==  4)
			tempRegValue |= (0x2 << 3);
	else if (scale == 8)
			tempRegValue |= (0x3 << 3);
	else if (scale == 16)
			tempRegValue |= (0x1 << 3);
	else
			tempRegValue |= (0x0 << 3); // Otherwise it'll be set to 2g (0x0 << 3)
	// Accel cutoff freqeuncy can be any value between -1 - 3. 
	// -1 = bandwidth determined by sample rate
	// 0 = 408 Hz   2 = 105 Hz
	// 1 = 211 Hz   3 = 50 Hz
	bandwidth = -1;
	if (bandwidth >= 0)
	{
		tempRegValue |= (1<<2); // Set BW_SCAL_ODR
		tempRegValue |= (bandwidth & 0x03);
	}
	
	mraa_i2c_write_byte_data(I2Chandle,tempRegValue,CTRL_REG6_XL);
	
	// CTRL_REG7_XL (0x21) (Default value: 0x00): [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	tempRegValue = 0;
	highResEnable = 0;
	if (highResEnable)
	{
		//tempRegValue |= (1<<7); // Set HR bit
		//tempRegValue |= (highResBandwidth & 0x3) << 5;
	}
	
	mraa_i2c_write_byte_data(I2Chandle,tempRegValue,CTRL_REG7_XL);
	
}


void initI2C(void)  // sets up both I2C Busses
{
	mraa_init();
//Initialize Display i2c Bus
    i2cp = mraa_i2c_init_raw (I2CP_BUS);
	mraa_i2c_frequency (i2cp, MRAA_I2C_STD);
	mraa_i2c_address(i2cp, LCD_ADDR);

    I2Chandle = mraa_i2c_init_raw (I2CS_BUS); // write correct I2C dev number here
    mraa_i2c_frequency (I2Chandle, MRAA_I2C_STD);
	mraa_i2c_address(I2Chandle, I2C_SLAVE_ADDR);
    
}

void enableFIFO(int enable)
{
    uint8_t temp = mraa_i2c_read_byte_data(I2Chandle,CTRL_REG9);
	if (enable)
	    temp |= (1<<1);
	else 
	    temp &= ~(1<<1);
	    
	mraa_i2c_write_byte_data(I2Chandle, temp,CTRL_REG9);
	mraa_i2c_write_byte_data(I2Chandle, 0xDF, FIFO_CTRL); //set threshold to 32 samples

}

void ReadAccelValues(int x, int y, int z)
{
	int i;
	float sample;
	char buf [20];
	char url_string[] = "https://docs.google.com/spreadsheets/d/1rg4W08CbqMqMyZeSlz-1WK2TtkiCX4LNvVTLs7EUuhU/edit#gid=0";
	char cloud_buffer [1024];

	uint8_t tempBuffer[6]; // We'll read six bytes from the accel into temp
	int32_t AccelRawData[3] = {0, 0, 0}, ii;
	fp = fopen( "Data.txt", "w" );

	
	
	for (i = 0; i < 2000; i ++)
	{
		AccelRawData[0] =0;
		AccelRawData[1] =0;
		AccelRawData[2] =0; 
		ax,ay,az=0;

		mraa_i2c_read_bytes_data(I2Chandle, OUT_X_L_XL, tempBuffer, 6) ;// Read 6 bytes, beginning at OUT_X_L_XL

		ax = (tempBuffer[1] << 8) | tempBuffer[0]; // Store x-axis values into ax
		ay = (tempBuffer[3] << 8) | tempBuffer[2]; // Store y-axis values into ay
		az = (tempBuffer[5] << 8) | tempBuffer[4]; // Store z-axis values into az
			
		AccelRawData[0] = ax;
		AccelRawData[1] = ay;
		AccelRawData[2] = az;// Assumes sensor facing up!
  
		for (ii = 0; ii < 3; ii++)
		{
			AccelVal[ii] = aRes * AccelRawData[ii]; //Now the value is in g i.e m/s2
		}
		
		sprintf (buf, " X     Y    Z ");
		LCD_Print ((uint8_t*)buf);
		sprintf(buf,"%3.2f %3.2f %3.2f   ", AccelVal[0], AccelVal[1], AccelVal[2]);
		LCD_Print2 (buf);
		printf(" \n Acceleration in :\n X direction = %3.2f\t Y direction = %3.2f\t Z direction = %3.2f g\n", AccelVal[0], AccelVal[1], AccelVal[2]);
		
		if(i%10 == 0){
		fprintf(fp, "%3.2f\t %3.2f\t %3.2f\n", AccelVal[0], AccelVal[1], AccelVal[2] );
		//fprintf(fp, "Acceleration in :\n X direction = %3.2f\t Y direction = %3.2f\t Z direction = %3.2f g\n", AccelVal[0], AccelVal[1], AccelVal[2] );
		}
		
		if( AccelVal[0] > x || AccelVal[1] > y || AccelVal[2] > z){
			blink();
		}
		sleep (1);
	}
	
	
	fclose(fp);
	
	
		
}

int main(void) {
   	int status;
	float x,y,z; 
    	initI2C();
	clear_LCD();
	Accelerometer_Init();
	status = mraa_i2c_read_byte(I2Chandle);
	 if (status < 0){
              printf("Failed to Initialize Accelerometer -> Exiting program\n");
              mraa_i2c_stop(I2Chandle);
              mraa_deinit();
              return EXIT_FAILURE;
         }

	printf("Enter X threshold ");
	scanf("%f", &x);
	printf("Enter Y threshold ");
	scanf("%f", &y);
	printf("Enter Z threshold ");
	scanf("%f", &z);
	printf("\n Acceleration threshold in :\n X direction = %3.2f\t Y direction = %3.2f\t Z direction = %3.2f g\n", x, y, z);
	ReadAccelValues(x,y,z);
	return 0;
}

