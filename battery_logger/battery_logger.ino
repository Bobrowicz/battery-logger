
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
#define V_SNS_1_ADC 0x02
#define I_SNS_ADC 0x04
#define I_SET_PIN 9
#define OPAMP_ENABLE_PIN 3
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
uint16_t discharge_current[] = { 100, 250, 500, 750, 1000, 1500, 2000 };

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
float select_battery(void);
int set_discharge_current(void);
int set_num_of_cells(void);
int state_0();
int state_1();
int state_2();
int state_3();
int state_4();
int state_6();
void wait_for_button(void);
float read_voltage();
float read_current();
int verify_calibration_coeeficients();
uint16_t read_ADC(uint8_t);

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
	state_machine[4] = state_4;
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
	Serial.begin(9600);
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
	
	ADMUX = 0;
	ADCSRA = 0;
	ADMUX |= (1 << REFS0); // Reference set to AVcc
	ADMUX |= (1 << MUX1);
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler division factor set to 128
	ADCSRA |= (1 << ADEN);
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1<<ADSC)); // Wait while first conversion finishes.
	

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
	//uint16_t adc_val = read_ADC(V_SNS_1_ADC);
	//Serial.println(adc_val);
	/*
	 * Check if data stored in eeprom is valid.
	 * Return value of 0 means that check failed.
	 * Return value of 1 means that check passed.
	 */
	if(verify_calibration_coeeficients() == 0) return 4;
		
	for (;;)
	{
		/*
		 * With 0 signal applied to the op-amp the FET still turns on slightly.
		 * To keep FET off, op-amp is powered down during idle state.
		 */
		digitalWrite(OPAMP_ENABLE_PIN, LOW);
		
		// sample voltage on battery connection
		volts =  read_voltage();
		
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
	batt_disch_i = set_discharge_current();
	
	return 2;
}

/*************************************************************************
discharge battery at constant current until cutoff voltage limit
integrate energy used at one second intervals
*************************************************************************/
int state_2()
{
	// polling timer variables
	//uint16_t seconds = 0;
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
			
			//seconds++;	// keep track of elapsed time
			
			// voltage will go down over time
			volts = read_voltage();
			// current should remain constant
			amps = ((read_ADC(I_SNS_ADC)*5.0)/4096) + 0.012;
			
			// running total of energy delivered by the battery under test
			watt_hour = watt_hour + (volts * amps) / 3600;
			millamp_hour = millamp_hour + (amps * 1000.0) / 3600;

			lcd_discharge_update();
			
			Serial.print("V: ");
			Serial.print(volts,6);
			Serial.print("	A: ");
			Serial.println(amps,6);
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

int state_4()
{
	lcd.clear();
	lcd.print("Fault");
	for (;;)
	{
		delay(10000);
	}
}

/*
 * This function calculates coefficients used to convert ADC reading to human friendly numbers,
 * and PWM duty cycle values for each current setpoint.
 * These values are saved to EEPROM consecutively.
 */ 
int state_6()
{
	digitalWrite(OPAMP_ENABLE_PIN, LOW);	// power down op-amp to completely turn off MOSFET
	I_SET_PWM = 0;	// Set current to minimum.
	
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Calibration mode"));
	lcd.setCursor(0, 1);
	lcd_print_P(PSTR("Press to start"));
	
	wait_for_button();
	
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Calibrating"));
	lcd.setCursor(0, 1);
	lcd_print_P(PSTR("Current flow"));
	delay(1000);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Press to start"));
	
	wait_for_button();
	
	float temp_coeff = 0;
	uint16_t adc_val = 0;
	uint8_t eeprom_addr = 0;
		
	digitalWrite(OPAMP_ENABLE_PIN, HIGH);	// power up op-amp to enable MOSFET
	
	
	for(uint8_t i = 0; i < sizeof(discharge_current) / sizeof(discharge_current[0]); i++)
	{
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd_print_P(PSTR("Set I to "));
		lcd.print(discharge_current[i]);
		lcd.setCursor(0, 1);
		lcd_print_P(PSTR("Press to measure"));
		
		while ((flags & ENC_BUTTON_PRESSED) == 0)
		{
			// check if rotary encoder has moved
			if (flags & ENC_POS_CHANGE)
			{
				temp_coeff += enc_count;
				if(temp_coeff < 0) temp_coeff = 0;
				
				I_SET_PWM = (int)temp_coeff;
				
				flags &= ~ENC_POS_CHANGE;  // clear flag
			}
			PCICR |= (1 << ENC_PCIE); // Re-enable interrupt
		}
		flags &= ~ENC_BUTTON_PRESSED;	// clear flag
		
		eeprom_update_word((uint16_t*)eeprom_addr,(uint16_t)temp_coeff);
		eeprom_addr += 2; // next memory address
		
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd_print_P(PSTR("Value stored."));
		delay(1000);
	}
	
	I_SET_PWM = 0;	// Set current to minimum.
	digitalWrite(OPAMP_ENABLE_PIN, LOW);	// power down op-amp to completely turn off MOSFET
	
	/*
	//const char* prompt = PSTR("Apply 1.0 A");
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Calibrating"));
	lcd.setCursor(0, 1);
	lcd_print_P(PSTR("Current Sense"));
	delay(3000);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Apply 1.0 A"));
	//lcd_print_P(prompt);
	lcd.setCursor(0, 1);
	lcd_print_P(PSTR("Press to measure"));
	
	I_SET_PWM = 1023;	// Turn MOSFET fully ON; current limited by external supply.
	
	wait_for_button();
	
	adc_val = read_ADC(I_SNS_ADC);
	I_SET_PWM = 0;	// Set current to minimum.
	digitalWrite(OPAMP_ENABLE_PIN, LOW);	// power down op-amp to completely turn off MOSFET
	
	temp_coeff = 1.0/adc_val;
	eeprom_update_float((float*)eeprom_addr, temp_coeff);
	eeprom_addr = eeprom_addr + sizeof(temp_coeff);
	*/	
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Calibrating"));
	lcd.setCursor(0, 1);
	lcd_print_P(PSTR("Voltage Sense"));
	delay(3000);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Apply 5.00 V"));
	lcd.setCursor(0, 1);
	lcd_print_P(PSTR("Press to measure"));
	
	wait_for_button();
	
	adc_val = read_ADC(V_SNS_1_ADC);
	temp_coeff = 5.0/adc_val;
	eeprom_update_float((float*)eeprom_addr, temp_coeff);
	
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd_print_P(PSTR("Cal complete"));
	delay(2000);
	
	return 0;
}

int verify_calibration_coeeficients()
{
	uint16_t eeprom_int_val;
	float eeprom_float_val;
	uint16_t eeprom_addr = 0;
	for(uint8_t i = 0; i < sizeof(discharge_current) / sizeof(discharge_current[0]); i++)
	{
		eeprom_int_val = eeprom_read_word((uint16_t*)eeprom_addr);
		if(eeprom_int_val == 0xFFFF)
		{
			return 0;
		}
		eeprom_addr += sizeof(discharge_current[i]);
	}

	
	eeprom_float_val = eeprom_read_float((float*)eeprom_addr);
	if(eeprom_float_val != eeprom_float_val)
	{
		return 0;
	}
	eeprom_addr += 4;
	
	eeprom_float_val = eeprom_read_float((float*)eeprom_addr);
	if(eeprom_float_val != eeprom_float_val)
	{
		return 0;
	}
	
	return 1;
}

void wait_for_button()
{
	while ((flags & ENC_BUTTON_PRESSED) == 0)
	{
		;
	}
	flags &= ~ENC_BUTTON_PRESSED;	// clear flag
}

uint16_t read_ADC(uint8_t ADC_channel)
{
	uint16_t ADC_sum = 0;
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (ADC_channel & 0x0F);
	// oversample signal
	for(uint8_t n = 0; n < 16; n++)
	{
		//single conversion mode
		ADCSRA |= (1<<ADSC);
		// wait until ADC conversion is complete
		while( ADCSRA & (1<<ADSC) );
		ADC_sum += ADC;
	}
	// return decimated result
	return (ADC_sum >> 2);
}

float read_voltage()
{	
	static uint8_t eeprom_addr = sizeof(discharge_current) + 4;
	static float v_sns_coeff = eeprom_read_float((float*)eeprom_addr);
	
	return read_ADC(V_SNS_1_ADC) * v_sns_coeff;
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

int set_discharge_current()
{
	int8_t temp = 1;
	uint8_t arr_elem_count = sizeof(discharge_current)/sizeof(discharge_current[0]);
		
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
	
	/*
	 * PWM compare value for selected discharge current is stored in EEPROM.
	 * To calculate memory address multiply array index by size of the element.
	 */
	uint8_t eeprom_addr = temp * sizeof(discharge_current[temp]);
	return eeprom_read_word((uint16_t*)eeprom_addr);
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
	lcd.setCursor(3, 0);
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
/*
float voltage_measure()
{
	uint16_t adc_read_value;
	uint8_t adc_channel[] = {V_SNS_1_PIN};
	float v_coeff[] = {0.009806, 0.029418};
	static int8_t i = 1;
	
	adc_read_value = analogRead(adc_channel[i]);
	
	while(adc_read_value < 400 && i != 0)
	{
		i = i - 1;
		adc_read_value = analogRead(adc_channel[i]);
	}
	
	while(adc_read_value > 1000 && i != 1)
	{
		i = i + 1;
		adc_read_value = analogRead(adc_channel[i]);
	}
	
	return adc_read_value * v_coeff[i];
}
*/