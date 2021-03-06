
/********************************************************************************
 * eYSIP-2015
 * PC Controlled Two Wheel Balanced Bot
 * Author List: B Suresh, Ramiz Hussain, Devendra Kr Jangid
 * Mentors: Piyush Manavar, Saurav Shandilya
 * Filename: adxl.h
 * Functions:write_byte(unsigned char, unsigned char), read_byte(unsigned char), sign(int), init_adxl(), acc_angle(), comp_filter(float, float),
 * 			 init_devices(), lcd_port_config(), twi_init(), pr_int(int, int, int, int)	
 * Global Variables:None
 *
 * 
*********************************************************************************/
#define F_CPU 14745600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "lcd.c"


#define	SLA_W	0xA6             // Write address for DS1307 selection for writing	
#define	SLA_R	0xA7             // Write address for DS1307 selection for reading  

//------------------------------------------------------------------------------
// define ADXL345 register addresses
//------------------------------------------------------------------------------
#define X1	 0x32            
#define X2 	 0x33
#define Y1 	 0x34
#define Y2   0x35
#define Z1 	 0x36
#define Z2   0x37




//------------------------------------------------------------------------------
//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;      //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;    // all the LCD pins are set to logic 0 except PORTC 7
}

//------------------------------------------------------------------------------
// I2C Peripheral Function Prototypes
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// I2C initilise
//------------------------------------------------------------------------------

//TWI initialize
// bit rate:72
void twi_init(void)
{
 TWCR = 0x00;   //disable twi
 TWBR = 0x10; //set bit rate
 TWSR = 0x00; //set prescale
 TWAR = 0x00; //set slave address
 TWCR = 0x04; //enable twi
}

//------------------------------------------------------------------------------
// Procedure:	write_byte 
// Inputs:		data out, address
// Outputs:		none
// Description:	Writes a byte to the ADXL345 given the address register 
//------------------------------------------------------------------------------
void write_byte(unsigned char data_out,unsigned char address)
{
 TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);       // send START condition  
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 _delay_ms(10);                                    

 TWDR = SLA_W;                                     // load SLA_W into TWDR Register
 TWCR  = (1<<TWINT) | (0<<TWSTA) | (1<<TWEN);      // clear TWINT flag to start tramnsmission of slave address 
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 _delay_ms(10);

 TWDR = address;                                   // send address of register byte want to access register
 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to start tramnsmission of address 
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 _delay_ms(10);

 TWDR = data_out;                       // convert the character to equivalent BCD value and load into TWDR
 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to start tramnsmission of data byte
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 _delay_ms(10);

 TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);       // send STOP condition
}

//------------------------------------------------------------------------------
// Procedure:	read_byte 
// Inputs:		address
// Outputs:		none
// Description:	read a byte from the RTC from send the address register 
//------------------------------------------------------------------------------
unsigned char read_byte(unsigned char address)
{  
 unsigned char rtc_recv_data;

 
TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);      // send START condition  
while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set

 

 TWDR = SLA_W;									   // load SLA_W into TWDR Register
 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to start tramnsmission of slave address 
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set

 TWDR = address;                                   // send address of register byte want to access register
 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to start tramnsmission of slave address 
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 


 TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);       // send RESTART condition
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set


 
 TWDR = SLA_R;									   // load SLA_R into TWDR Register
 TWCR  = (1<<TWINT) | (0<<TWSTA) | (1<<TWEN);      // clear TWINT flag to start tramnsmission of slave address 
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 
 
 

 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to read the addressed register
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 rtc_recv_data = TWDR;
 
 TWDR = 00;                                        // laod the NO-ACK value to TWDR register 
 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to start tramnsmission of NO_ACK signal
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
  
 return(rtc_recv_data);                            // return the read value to called function
}


// initialise the LCD and I2C 
void init_devices()
{
 cli();              // disable all interrupts 
 lcd_port_config();  // configure the LCD port 
 lcd_set_4bit();
 lcd_init();
 twi_init();         // configure the I2C, i.e TWI module 
 sei();              // re-enable interrupts
 //all peripherals are now initialized
}

//-------------------------------------------------------------------------------
/*
* Function Name: pr_int
* Input: a(row), b(column), c(signed int), d(precision)
* Output: display on lcd
* Logic: Based on the sign of c an additional sign is added before the display of the number
* Example Call:pr_int(1,1,-6565,4);
*
*/
//-------------------------------------------------------------------------------
void pr_int(int a,int b,int c,int d) /* get negative values*/
{
	if (c<0)
	{
		lcd_cursor(a,b);
		lcd_string("-");
		lcd_print(a,b+1,abs(c),d);
	} 
	else
	{
		lcd_cursor(a,b);
		lcd_string("+");
		lcd_print(a,b+1,c,d);
	}
}

//To convert unsigned 16 bit integer to signed integer
int sign (unsigned int n)
{
	if (n>32767)
	{
		return (n-65536);
	}
	else
		return n;
		
}

//To initiate the accelerometer and put it in measure mode 
void init_adxl(void)
{   
 
 init_devices();

	write_byte(0x0,0x2D);			
	write_byte(0x8,0x2D);			//To put ADXL345 in measurement mode
}


//-------------------------------------------------------------------------------
/*
* Function Name: acc_angle()
* Input: None
* Output: angle
* Logic: 1.Read the registers and combine and return the signed value
* 		 2.Taking the arctan(horizontal component/vertical component).The other component need to be neutral
*		 3.Converting radians to degrees
* Example Call:var=acc_angle();
*
*/
//-------------------------------------------------------------------------------
int acc_angle(void)
{
	    uint16_t x_byte = 0,y_byte = 0,z_byte = 0;
		uint8_t x_byte1 = 0,x_byte2 = 0,y_byte1 = 0,y_byte2 = 0,z_byte1 = 0,z_byte2 = 0;
		int x_acc,y_acc,z_acc;
		float angle;
 
	  
	   x_byte1 = read_byte(X1);						//Reading registers
	
	   x_byte2 = read_byte(X2);

	   y_byte1 = read_byte(Y1);

	   y_byte2 = read_byte(Y2);

	   z_byte1 = read_byte(Z1);

	   z_byte2 = read_byte(Z2);
	     
	  x_byte=x_byte2;								//merging the values from two registers to one variable
	  x_byte = (x_byte << 8);
	  x_byte |= x_byte1;
	  x_acc=sign(x_byte);
	  
	  y_byte=y_byte2;
	  y_byte = (y_byte << 8);
	  y_byte |= y_byte1;
	  y_acc=sign(y_byte);
	  
	  z_byte=z_byte2;
	  z_byte = (z_byte << 8);
	  z_byte |= z_byte1;
	  z_acc=sign(z_byte);
	  
	  angle=(atan((y_acc*1.0)/(z_acc*1.0)));		//Angle calculation in radians
	  angle *= 10*180.0/3.14;						//Angle conversion to degrees
	  
	return angle;
}


//--------------------------------------------------------------------------------




