/*
 * battery_logger.ino
 *
 * Created: 3/20/2015 10:55:15 PM
 * Author: Peter
 */ 
#include <Wire.h>
#include <LiquidCrystal_SPI/LiquidCrystal.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#define F_CPU 16000000UL
#include <util/delay.h>

/*
 * constants/macros
 */
#define ENC_POS_CHANGE 0x01
#define ENC_BUTTON_PRESSED 0x02
#define ENC_UP 0x04
#define ENC_DOWN 0x08
#define BOOL5 0x10
#define BOOL6 0x20
#define BOOL7 0x40
#define BOOL8 0x80
#define ENC_PIN_AB 0x30
#define ENC_PIN_A 5
#define ENC_PIN_B 4
#define ENC_PIN_BUTTON 2 
#define ENC_READ_PIN PIND
#define ENC_INT_VECT PCINT2_vect
#define ENC_PCIE PCIE2
#define ENC_INT_MSK PCMSK2
#define ENC_INT_PIN_A PCINT21
#define ENC_INT_PIN_B PCINT20
#define V_SNS_PIN A3
#define I_SNS_PIN A4
#define I_SET_PIN 9
#define OPAMP_ENABLE_PIN 3
#define V_REF 4.99
#define V_SUP 4.99
#define V_SNS_COEFF 0.00487
#define ADC_RES 1024
#define PWM_RES 1024
#define I_SET_PWM OCR1A

/*
 * global variables
 */
volatile uint8_t enc_current_state;
volatile uint8_t enc_last_state;
volatile int8_t enc_count = 0;
volatile char flags;
int batt_disch_i;
float batt_min_v;
double volts, amps, watt_hour, millamp_hour;
float EEMEM v_sns_coeff;
float EEMEM i_sns_coeff;
float EEMEM i_set_coeff;

/*
 * function prototypes
 */
void setup(void);
void lcd_discharge_labels(void);
void lcd_discharge_update(void);
void lcd_complete_message(void);
void lcd_complete_summary(void);
void lcd_clear_line(int);
void lcd_print_P();
double adc_read(int);
float select_battery(void);
float set_discharge_current(void);
int set_num_of_cells(void);
int selection(int);
int state_0();
int state_1();
int state_2();
int state_3();
int state_4();
void eeprom_read(void);
void eeprom_write(float, int);

/*
 * Connect LCD via SPI. Data pin is #11, Clock is #13 and Latch is #10
 */
LiquidCrystal lcd(11, 13, 10);

int main(int argc, char** argv)
{
	init();	
	setup();	// uC setup
	sei();	// enable global interrupts

	/*
	 * My "state machine" is an array of function names.
	 * Functions returns integer value which is an array index containing next function.
	 */
	int(*state_machine[7])();

	state_machine[0] = state_0;	// Idle
	state_machine[1] = state_1;	// Battery setup
	state_machine[2] = state_2; // Discharge and measure
	state_machine[3] = state_3;	// Discharge complete
	//state_machine[4] = state_4;
	//state_machine[5] = state_5;
	state_machine[6] = state_6; // Calibration

	int current_state = 0;
	
	/*
	 * To enter calibration mode the rotary encoder switch must be depressed during uC reset.
	 */
	if(digitalRead(ENC_PIN_BUTTON) == 1)
	{
		delay(500);
		if(digitalRead(ENC_PIN_BUTTON) == 1) {current_state = 6;} // Verify that switch is still depressed.
	}

	for (;;)
	{
		if (current_state == -1)
			break;

		current_state = (*state_machine[current_state])();

		delay(1);
	}
}

/*************************************************************************
u_controller setup function
*************************************************************************/
void setup()
{
	/*
	 * Set up the LCD's number of columns and rows.
	 */
	lcd.begin(16, 2);
	lcd.setBacklight(HIGH);
	
	/*
	 * Set up I/O.
	 */
	pinMode(ENC_PIN_A, INPUT);
	pinMode(ENC_PIN_B, INPUT);
	pinMode(ENC_PIN_BUTTON, INPUT);
	pinMode(V_SNS_PIN, INPUT);
	pinMode(I_SNS_PIN, INPUT);
	pinMode(I_SET_PIN, OUTPUT);
	pinMode(OPAMP_ENABLE_PIN, OUTPUT);

	/*
	 * Set up Fast PWM on Timer1.
	 * To generate smoother analog signal PWM frequency is increased to 15 kHz.
	 */
	TCCR1A = 0;	// clear Arduino values
	TCCR1B = 0;	// clear Arduino values
	TCCR1A |= (1 << COM1A1);	// OC1A Non-inverting mode
	TCCR1A |= (1 << WGM11) | (1 << WGM10);	// Fast PWM 10-bit
	TCCR1B |= (1 << WGM12);	// Fast PWM 10-bit
	TCCR1B |= (1 << CS10);	// No prescaling

	// set up rotary encoder
	EICRA |= (1 << ISC01) | (1 << ISC00);	// set interrupt sense control to rising edge
	EIMSK |= (1 << INT0);	// Enable external interrupt
	PCICR |= (1 << ENC_PCIE);	// Enable pin change interrupt
	ENC_INT_MSK |= (1 << ENC_INT_PIN_A) | (1 << ENC_INT_PIN_B);
	enc_last_state = ENC_READ_PIN & ENC_PIN_AB;	// Get initial encoder state
	enc_last_state = enc_last_state >> 4;	// align bits
}

/*************************************************************************
wait for battery to be attached.
minimum voltage to register battery as connected is 0.9 V.
*************************************************************************/
int state_0()
{
	for (;;)
	{
		/*
		 *With 0 signal applied to the op-amp the FET still turns on slightly.
		 *To keep FET off, op-amp is powered down during idle state.
		 */
		digitalWrite(OPAMP_ENABLE_PIN, LOW);
		delay(1);
		
		// sample voltage on battery connection
		volts = adc_read(V_SNS_PIN);
		
		// if battery voltage is within acceptable limit go on to next state
		if (volts >= 0.9)
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd_print_P(PSTR("Batt V: "));
			lcd.setCursor(8, 0);
			lcd.print(volts);
			delay(1000);
			
			return 1;
		}
		// otherwise wait and check again
		else
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd_print_P(PSTR("No battery."));
		}
		delay(1000);
	}
}

/*************************************************************************
set up battery pack parameters
*************************************************************************/
int state_1()
{
	/*
	To calculate discharge cutoff voltage we need to determine
	battery chemistry and number of cells connected in series.
	*/
	batt_min_v = select_battery() * set_num_of_cells();
	
	/*
	Battery discharge current is set by reference signal from PWM
	low pass filtered in hardware to obtain analog value.
	*/
	batt_disch_i = (set_discharge_current() * PWM_RES) / V_SUP;
	
	return 2;
}

/*************************************************************************
discharge battery at constant current until cutoff voltage limit
integrate energy used at one second intervals
*************************************************************************/
int state_2()
{
	// polling timer variables
	uint16_t seconds = 0;
	uint32_t previous_time;
	uint32_t interval = 1000; // milliseconds

	// reset counters
	watt_hour = 0;
	millamp_hour = 0;

	lcd_discharge_labels();	// Print static elements
	previous_time = millis();

	
	digitalWrite(OPAMP_ENABLE_PIN, HIGH);	// power up op-amp to enable MOSFET
	delay(1);

	/*
	 * Discharge current is proportional to set point voltage in 1V to 1A ratio.
	 * Current is controlled by adjusting PWM duty cycle to obtain desired average voltage, 
	 * then low pass filtering resultant square wave to produce analog signal.
	 */
	I_SET_PWM = batt_disch_i;	// Set PWM duty cycle
	delay(1);
	
	// run until low voltage limit is reached
	while (volts > batt_min_v)
	{
		if ((previous_time + interval) <= millis())
		{
			previous_time += interval;
			
			seconds++;	// keep track of elapsed time (not using this right now)
			
			// voltage will go down over time
			volts = adc_read(V_SNS_PIN);
			// current should remain constant
			amps = adc_read(I_SNS_PIN);
			
			// running total of energy delivered by the battery under test
			watt_hour = watt_hour + (volts * amps) / 3600;
			millamp_hour = millamp_hour + (amps * 1000.0) / 3600;

			lcd_discharge_update();
		}
	}

	I_SET_PWM = 0;	// Set current to minimum.
	digitalWrite(OPAMP_ENABLE_PIN, LOW);	// power down op-amp to completely turn off MOSFET

	return 3;
}

int state_3()
{
	for (;;)
	{
		lcd_complete_message();
		delay(2000);
		lcd_complete_summary();
		delay(2000);
	}
}

int state_6()
{
	float temp_coeff;
	uint16_t adc_val = 0;
	int temp = 400;
	
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Calibration mode"));
	lcd.setCursor(0, 1);
	lcd_print_P(PSTR("Press to start"));
	
	while ((flags & ENC_BUTTON_PRESSED) == 0)
	{
		;
	}
	flags &= ~ENC_BUTTON_PRESSED;	// clear flag
	
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Apply 4.50 V"));
	lcd.setCursor(0, 1);
	lcd_print_P(PSTR("Press when rdy"));
	
	while ((flags & ENC_BUTTON_PRESSED) == 0)
	{
		;
	}
	flags &= ~ENC_BUTTON_PRESSED;	// clear flag
	
	for(int i = 0; i < 10; i++)
	{
		adc_val += analogRead(V_SNS_PIN);
		delay(10);
	}
	adc_val = adc_val / 10;
	
	temp_coeff = 4.5/adc_val;
	
	eeprom_write(temp_coeff, 0);
	eeprom_read();
	/*
	while ((flags & ENC_BUTTON_PRESSED) == 0)
	{
		// check if rotary encoder has moved
		if (flags & ENC_POS_CHANGE)
		{
			temp += enc_count;
			if(temp < 0) temp = 0;
			
			lcd_clear_line(1);
			lcd.setCursor(5, 1);
			lcd.print(temp/100.0);
			
			flags &= ~ENC_POS_CHANGE;  // clear flag
		}
		PCICR |= (1 << ENC_PCIE); // Re-enable interrupt
	}
	flags &= ~ENC_BUTTON_PRESSED;	// clear flag
	*/
	//delay(20000);
}

void eeprom_read()
{
	float float_num;
	
	lcd.clear();
	lcd.setCursor(0, 0);
	
	float_num = eeprom_read_float((float*)0);
	lcd.print(float_num);
	
	delay(10000);
}

void eeprom_write(float float_num, int addr)
{
	eeprom_update_float((float*)addr, float_num);
}

double adc_read(int pin)
{
	double temp;

	temp = analogRead(pin);
	temp = (temp * V_REF) / ADC_RES;
	//temp = temp * 0.00487;

	return temp;
}



float select_battery()
{
	String batt_types[] = { "NiMh", "Lead Acid", "Li-ion" };
	float min_cell_v[] = {   0.9,    1.95,        2.8 };
	uint8_t arr_elem_count = sizeof(min_cell_v)/sizeof(float);
	int8_t temp = 2;
		
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Battery Type."));
	
	flags |= ENC_POS_CHANGE;	// set flag to force conditional check and print default vale
	
	// loop will run until button press is detected
	while ((flags & ENC_BUTTON_PRESSED) == 0)
	{
		// check if rotary encoder has moved
		if (flags & ENC_POS_CHANGE)
		{
			temp += enc_count;
			// moving through the array in a circle
			if(temp > arr_elem_count-1) temp = 0;
			if(temp < 0) temp = arr_elem_count-1;
			
			lcd_clear_line(1);
			lcd.setCursor(0, 1);
			lcd.print(batt_types[temp]); // prints array element
			
			flags &= ~ENC_POS_CHANGE;  // clear flag		
		}
		PCICR |= (1 << ENC_PCIE); // Re-enable interrupt
	}
	flags &= ~ENC_BUTTON_PRESSED;	// clear flag
	
	return min_cell_v[temp];
}

int set_num_of_cells()
{
	int8_t temp = 1;
	
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Number of cells."));
	
	flags |= ENC_POS_CHANGE;	// set flag to force conditional check and print default vale
	
	while ((flags & ENC_BUTTON_PRESSED) == 0)
	{
		// check if rotary encoder has moved
		if (flags & ENC_POS_CHANGE)
		{
			temp += enc_count;
			if(temp < 1) temp = 1;
			
			lcd_clear_line(1);
			lcd.setCursor(0, 1);
			lcd.print(temp);
			
			flags &= ~ENC_POS_CHANGE;  // clear flag
		}
		PCICR |= (1 << ENC_PCIE); // Re-enable interrupt
	}
	flags &= ~ENC_BUTTON_PRESSED;	// clear flag
	
	return temp;
}

float set_discharge_current()
{
	int discharge_current[] = { 100, 250, 500, 750, 1000, 1500, 2000, 2500, 3000 };
	uint8_t arr_elem_count = sizeof(discharge_current)/sizeof(int);
	int8_t temp = 1;
		
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Select current."));
	
	
	flags |= ENC_POS_CHANGE;	// set flag to force conditional check and print default vale

	// loop will run until button press is detected
	while ((flags & ENC_BUTTON_PRESSED) == 0)
	{
		// check if rotary encoder has moved
		if (flags & ENC_POS_CHANGE)
		{
			// moving through the array in a circle
			temp += enc_count;
			if(temp > arr_elem_count-1) temp = 0;
			if(temp < 0) temp = arr_elem_count-1;
			
			lcd_clear_line(1);
			lcd.setCursor(0, 1);
			lcd.print(discharge_current[temp]); // prints array element
			
			flags &= ~ENC_POS_CHANGE;  // clear flag
			
		}
		PCICR |= (1 << ENC_PCIE); // Re-enable interrupt
	}
	flags &= ~ENC_BUTTON_PRESSED;	// clear flag
	
	return discharge_current[temp] / 1000.0;
}

void lcd_discharge_labels()
{
	lcd.clear();
	lcd_print_P(PSTR("V: "));
	lcd.setCursor(9, 0);
	lcd_print_P(PSTR("A: "));
	lcd.setCursor(0, 1);
	lcd_print_P(PSTR("Wh: "));

}

void lcd_discharge_update()
{
	lcd.setCursor(4, 0);
	lcd.print(volts);
	lcd.setCursor(12, 0);
	lcd.print(amps);
	lcd.setCursor(4, 1);
	lcd.print(watt_hour);
	lcd.setCursor(10, 1);
	lcd.print(millamp_hour);
}

void lcd_complete_message()
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Discharge"));
	lcd.setCursor(0, 1);
	lcd_print_P(PSTR("Completed"));
}

void lcd_complete_summary()
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Wh: "));
	lcd.setCursor(5, 0);
	lcd.print(watt_hour);
	lcd.setCursor(0, 1);
	lcd_print_P(PSTR("mAh: "));
	lcd.setCursor(5, 1);
	lcd.print(millamp_hour);
}

void lcd_clear_line(int line)
{
	lcd.setCursor(0, line);
	lcd_print_P(PSTR("                "));
}

void lcd_print_P(const PROGMEM char *s)
{
	uint8_t c;
	while ((c = pgm_read_byte_near(s++)) != 0)
	lcd.print((char)c);
}

/* Interrupt routine triggered by button press */
ISR(INT0_vect)
{
	flags |= ENC_BUTTON_PRESSED;
}

/* Interrupt routine triggered by change of rotary encoder pin state */
ISR(ENC_INT_VECT)
{
	// Disable interrupt to prevent further triggering
	PCICR &= ~(1 << ENC_PCIE);

	// Read rotary encoder pins
	uint8_t temp = ENC_READ_PIN & ENC_PIN_AB;
	// align bits
	temp = temp >> 4;

	// Attempt to filter false triggers; branch is taken when new reading is different from previous
	if (temp != enc_last_state)
	{
		// Combine current and previous encoder value into 4 bit sequence.
		enc_current_state = temp;
		enc_current_state |= (enc_last_state << 2);
		enc_last_state = temp;

		// Check if encoder state is valid and update variables
		//  0b00001101  0b00000100  0b00000010  0b00001011
		if (enc_current_state == 0b00000010)
		{
			enc_count = -1;
			//flags |= ENC_DOWN;
			flags |= ENC_POS_CHANGE; // set change flag
		}
		// 0b00001110  0b00001000  0b00000001  0b00000111
		if (enc_current_state == 0b00001110)
		{
			enc_count = 1;
			//flags |= ENC_UP;
			flags |= ENC_POS_CHANGE; // set change flag
		}
	}
	else PCICR |= (1 << ENC_PCIE); // Re-enable interrupt
}