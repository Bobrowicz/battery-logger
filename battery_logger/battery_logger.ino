
#include <LiquidCrystal_SPI\LiquidCrystal.h>
#include <Wire.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>

/*
** constants/macros
*/
#define ENC_POS_CHANGE 0x01
#define ENC_BUTTON_PRESSED 0x02
#define BOOL3 0x04
#define BOOL4 0x08
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
#define I_SNS_PIN A2
#define V_SET_PIN 9
#define OPAMP_ENABLE_PIN 8
#define V_REF 4.7
#define V_SUP 5.01
#define ADC_RES 1024
#define PWM_RES 1024
#define MOSFET_PWM OCR1A

/*
** global variables
*/
volatile uint8_t enc_current_state;
volatile uint8_t enc_last_state;
volatile uint8_t enc_count = 0;
volatile char flags;
int batt_disch_i;
double volts, amps, watt_hour, millamp_hour, batt_min_v;
double min_cell_v[] = { 0.9, 1.95, 2.8 };
int num_of_cells[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };
int discharge_current[] = { 100, 250, 500, 750, 1000, 1500, 2000, 2500, 3000 };
String batt_types[] = { "NiMh", "Lead Acid", "Li-ion" };
int *testPW; // for testing; change to something more meaningful later

/*
** function prototypes
*/
void setup(void);
void lcd_discharge_labels(void);
void lcd_discharge_update(void);
void lcd_complete_message(void);
void lcd_complete_summary(void);
void lcd_clear_line(int);
double adc_read(int);
//int selection(int *);
int selection(int);
int state_0();
int state_1();
int state_2();
int state_3();
void lcd_print_string(char);

// Connect via SPI. Data pin is #11, Clock is #12 and Latch is #10
LiquidCrystal lcd(11, 12, 10);

int main(int argc, char** argv)
{
	// dont know what init() does but it's needed
	init();
	// uC setup
	setup();
	// enable global interrupts
	sei();

	// set up state machine
	int(*state_machine[7])();

	state_machine[0] = state_0;
	state_machine[1] = state_1;
	state_machine[2] = state_2;
	state_machine[3] = state_3;
	//state_machine[4] = state_4;
	//state_machine[5] = state_5;
	//state_machine[6] = state_6;

	int current_state = 0;

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
	Serial.begin(9600); // for testing
	analogReference(EXTERNAL);

	// set up the LCD's number of columns and rows:
	lcd.begin(16, 2);
	lcd.setBacklight(HIGH);

	// i/o
	pinMode(ENC_PIN_A, INPUT);
	pinMode(ENC_PIN_B, INPUT);
	pinMode(ENC_PIN_BUTTON, INPUT);
	pinMode(V_SNS_PIN, INPUT);
	pinMode(I_SNS_PIN, INPUT);
	pinMode(V_SET_PIN, OUTPUT);
	pinMode(OPAMP_ENABLE_PIN, OUTPUT);

	// fast pwm timer1
	TCCR1A = 0x83;
	TCCR1B = 0x9;

	// set up rotary encoder
	rot_enc_init();
}

void rot_enc_init(void)
{
	// Set interrupt sense control to rising edge
	EICRA |= (1 << ISC01) | (1 << ISC00);
	// Enable external interrupt
	EIMSK |= (1 << INT0);
	// Enable pin change interrupt
	PCICR |= (1 << ENC_PCIE);
	ENC_INT_MSK |= (1 << ENC_INT_PIN_A) | (1 << ENC_INT_PIN_B);

	// Get initial encoder state
	enc_last_state = ENC_READ_PIN & ENC_PIN_AB;
	enc_last_state = enc_last_state >> 4;
}

/*************************************************************************
wait for batttery to be attached.
minimum voltage to register battery as connected is 0.9 V.
*************************************************************************/
int state_0()
{
	for (;;)
	{
		// power up op-amp buffer to check battery voltage
		digitalWrite(OPAMP_ENABLE_PIN, HIGH);
		delay(1);
		// sample voltage on battery connection
		volts = adc_read(V_SNS_PIN);
		
		
		// if battery voltage within acceptable limit go on to next state
		if (volts >= 0.9)
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("Batt V: ");
			lcd.setCursor(8, 0);
			lcd.print(volts);

			delay(1000);
			return 1;
		}
		// otherwise wait and check again
		else
		{
			//power down op-amp buffer
			digitalWrite(OPAMP_ENABLE_PIN, LOW);
			
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("No battery.");
		}

		delay(1000);
	}
}

/*************************************************************************
set up battery parameters
*************************************************************************/
int state_1()
{
	selection(2);
	/*
	int temp;
	//to do - implement UI
	testPW = discharge_current;
	temp = selection(testPW);
	// select battery chemistry
	uint8_t batt_type = 2;

	// select number of cells in a pack
	uint8_t num_of_cells = 1;

	//select desired discharge current in milliamps
	//testPW = discharge_current;
	//temp = selection(testPW, 3);
	double discharge_current = temp;
	discharge_current = discharge_current / 1000.0;

	// calculate end of discharge battery voltage
	batt_min_v = min_cell_v[batt_type] * num_of_cells;
	// calculate PWM timer compare value
	batt_disch_i = (discharge_current * PWM_RES) / V_SUP;
	*/
	return 3;
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
	uint32_t interval = 1000;

	// reset counters
	watt_hour = 0;
	millamp_hour = 0;

	lcd_discharge_labels();
	previous_time = millis();

	//enable op-amp
	digitalWrite(OPAMP_ENABLE_PIN, HIGH);
	delay(1);

	// turn on MOSFET
	MOSFET_PWM = (V_SET_PIN, batt_disch_i);
	volts = 3.0;

	while (volts > batt_min_v)
	{
		if ((previous_time + interval) <= millis())
		{
			previous_time += interval;
			seconds++;

			volts = adc_read(V_SNS_PIN);
			amps = adc_read(I_SNS_PIN);

			watt_hour = watt_hour + (volts * amps) / 3600;
			millamp_hour = millamp_hour + (amps * 1000.0) / 3600;

			lcd_discharge_update();
		}
	}

	// turn off MOSFET
	MOSFET_PWM = (V_SET_PIN, 0);
	//disable op-amp
	digitalWrite(OPAMP_ENABLE_PIN, LOW);

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

double adc_read(int pin)
{
	double temp;

	temp = analogRead(pin);
	//Serial.println(temp);
	temp = (temp * V_REF) / ADC_RES;

	return temp;
}

void lcd_discharge_labels()
{
	lcd.clear();
	lcd.print("V: ");
	lcd.setCursor(9, 0);
	lcd.print("A: ");
	lcd.setCursor(0, 1);
	lcd.print("Wh: ");

}

void lcd_discharge_update()
{
	lcd.setCursor(4, 0);
	lcd.print(volts);
	//Serial.print(volts);
	//Serial.print("	");
	lcd.setCursor(12, 0);
	lcd.print(amps);
	//Serial.println(amps, 4);
	lcd.setCursor(4, 1);
	lcd.print(watt_hour);
	lcd.setCursor(10, 1);
	lcd.print(millamp_hour);
}

void lcd_complete_message()
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Discharge");
	lcd.setCursor(0, 1);
	lcd.print("Completed");
}

void lcd_complete_summary()
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Wh: ");
	lcd.setCursor(5, 0);
	lcd.print(watt_hour);
	lcd.setCursor(0, 1);
	lcd.print("mAh: ");
	lcd.setCursor(5, 1);
	lcd.print(millamp_hour);
}

void lcd_clear_line(int line)
{
	lcd.setCursor(0, line);
	lcd.print("                ");

}

void lcd_print_string(String arr)
{
	//int num = sizeof arr;
	Serial.println(arr);
}

// work in progress
int selection(int prompt)
//int selection(int *point)
{
	int option_list;
	//String options;
	lcd.clear();
	lcd.setCursor(0, 0);
	
	switch (prompt)
	{
	case 1:
		lcd.print("Battery Type.");
		// loop will run until button press is detected
		while ((flags & ENC_BUTTON_PRESSED) == 0)
		{
			// check if rotary encoder has moved
			if (flags & ENC_POS_CHANGE)
			{
				Serial.println(enc_count);  // test message
				lcd_clear_line(1);
				lcd.setCursor(0, 1);
				//lcd.print(point[enc_count]); // prints array element
				flags &= ~ENC_POS_CHANGE;  // clear flag
			}
		}
		break;
	case 2:
		lcd.print("Number of cells.");
		
		break;
	case 3:
		lcd.print("Select current.");
		break;
	}
	
	// loop will run until button press is detected
	while ((flags & ENC_BUTTON_PRESSED) == 0)
	{
		// check if rotary encoder has moved
		if (flags & ENC_POS_CHANGE)
		{
			Serial.println(enc_count);  // test message
			lcd_clear_line(1);
			lcd.setCursor(0, 1);
			//lcd.print(point[enc_count]); // prints array element
			flags &= ~ENC_POS_CHANGE;  // clear flag
		}
	}
	//return point[enc_count];
	
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
		if (/*enc_current_state == 0b00001101 || enc_current_state == 0b00000100 ||*/ enc_current_state == 0b00000010 /*|| enc_current_state == 0b00001011*/)
		{
			enc_count++;
			flags |= ENC_POS_CHANGE; // set change flag
		}
		if (/*enc_current_state == 0b00001110 || enc_current_state == 0b00001000 ||*/ enc_current_state == 0b00000001 /*|| enc_current_state == 0b00000111*/)
		{
			enc_count--;
			flags |= ENC_POS_CHANGE; // set change flag
		}
	}
	else PCICR |= (1 << ENC_PCIE); // Re-enable interrupt
}