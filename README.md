#include "sam.h"
#include "timers.h"

//unsigned long volatile *pioc_sodr = (uint32_t volatile *) 0x400E1230;
//unsigned long volatile *pioc_codr = (uint32_t volatile *) 0x400E1234;
#define TIME_TO_STABILIZE (15)

#define SAMPLING_FREQ (16)
#define CLK_DIV_FACTOR (128)
#define TIMER1_TICKS (VARIANT_MCK / CLK_DIV_FACTOR / SAMPLING_FREQ)

volatile unsigned int ir_changes_counter = 0;

volatile unsigned long last_change = 0;

volatile int error = 0;
volatile int error_derivative = 0;
volatile int error_integral = 0;

volatile int speed = 0;
int speed_reference = 0;

unsigned long polishing_start = 0;
bool polishing = false;

void inc_ir_changes_counter() {
	unsigned long current_change = millis();
	if (current_change > last_change + TIME_TO_STABILIZE) {
		last_change = current_change;
		ir_changes_counter += 1;
	}
}


#define TIMER1_CHANNEL (0)
void TC3_Handler() {
	TC_GetStatus(TC1, 0);

	/* For accuracy we do operations that
	 * depend on time in this ISR */
	int error_previous = error;

	speed = ir_changes_counter;
	error = speed_reference - speed;
	error_derivative = error - error_previous;
	error_integral += error; 
  
  /* Home made anti-windup */
	if (error_integral < I_MIN)
		error_integral = I_MIN;
	else if (error_integral > I_MAX)
		error_integral = I_MAX;

	/* Speed is in half-rotations per sample */
	unsigned int u = saturate(Kp * error + Kd * error_derivative * SAMPLING_FREQ + Ki * error_integral / SAMPLING_FREQ); 

	if (polishing) {
		if (check_polishing_done()) {
			stop_polishing();
		}
	} else {
		if (error > POLISHING_TRESHOLD) {
			start_polishing();
		}
	}

	analogWrite(PWM_PIN, u);

	/* Reset counter rather than save previous count */
	ir_changes_counter = 0; 
}

#define LED_PIN (5)
#define IR_PIN (6)
#define PWM_PIN (7)
void setup() {
	pinMode(IR_PIN, INPUT);
	pinMode(PWM_PIN, OUTPUT);
	pinMode(LED_PIN, OUTPUT);
	Serial.begin(9600);
  analogWrite(PWM_PIN, u);
	attachInterrupt(IR_PIN, &inc_ir_changes_counter, FALLING);
	start_timer_TC1(TIMER1_TICKS, TIMER1_CHANNEL);
}

#define POLISHING_TIME (5000)
#define POLISHING_TRESHOLD (50)
#define POLISHING_SPEED (256)
#define IDLE_SPEED (128)
void start_polishing() {
	polishing_start = millis();
	speed_reference = POLISHING_SPEED;
	/* Turn led on */
	REG_PIOC_SODR = PIO_PC25;
	polishing = true;
}

void stop_polishing() {
	speed_reference = IDLE_SPEED;
	/* Turn led off */
	REG_PIOC_CODR = PIO_PC25;
	polishing = false;
}

int check_polishing_done() {
	unsigned long current_time = millis();
	return current_time - polishing_start > POLISHING_TIME;
}

#define U_MIN (0)
#define U_MAX (255)
unsigned int saturate(unsigned int u) {
	if (u < U_MIN)
		return U_MIN;
	else if (u > U_MAX)
		return U_MAX;
	return u;
}

#define Kp (1.0)
#define Kd (1.0)
#define Ki (1.0)
#define I_MIN (-50)
#define I_MAX (50)
void loop() {
	Serial.print("u = ");
	Serial.println(u);
	delay(300);
}
