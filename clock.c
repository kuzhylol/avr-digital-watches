#ifndef F_CPU
#define F_CPU   16000000UL
#endif

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

#include "segm.h"

uint8_t segm_time[4]; 
uint8_t segm_alarm[4];

struct clock{
	uint8_t seconds, min_l, min_r, hours_l, hours_r;
		/* [hours_l] [hours_r] : [min_l] [min_r] */
			      /* 15:47 */
};
 
struct clock time;	/* Real time */
struct clock alarm; 	/* Alarm */

	static void soft_delay(volatile uint8_t N)
{
	/* If volatile is not used, AVR-GCC will optimize this stuff out     */
        /* making our function completely empty                              */
	volatile uint8_t inner = 0x01;
	while (N--) {
		while (inner--);
	}
}

/** Timer2 Interrupt (on overflow), see datasheet
 * For vectors, refer to <avr/iom328p.h>
 * For more on interrupts handling with AVR-GCC see
 * https://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
 */

ISR(TIMER2_OVF_vect, ISR_BLOCK)
{
	TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20)); /* stop timer */
	/* It's often required to manually reset interrupt flag */
        /* to avoid infinite processing of it.                  */
        /* not on AVRs (unless OCB bit set)                     */
        /* 	TIFR2 &= ~TOV2;                                 */
}

void sleep_ms(uint16_t ms_val)
{
	/* Set Power-Save sleep mode */
	/* https://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html */
	//set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	set_sleep_mode(SLEEP_MODE_IDLE);
	cli();		/* Disable interrupts -- as memory barrier */
	sleep_enable();	/* Set SE (sleep enable bit) */
	sei();  	/* Enable interrupts. We want to wake up, don't we? */
	TIMSK2 |= (1 << TOIE2); /* Enable Timer2 Overflow interrupt by mask */
	while (ms_val--) {
		/* Count 1 ms from TCNT2 to 0xFF (up direction) */
		TCNT2 = (uint8_t)(0xFF - (F_CPU / 128) / 1000);

		/* Enable Timer2 */
		TCCR2B =  (1 << CS22) | (1 << CS20); /* f = Fclk_io / 128, start timer */

		//sleep_cpu();	/* Put MCU to sleep */

		/* This is executed after wakeup */
		
	}
	sleep_disable();	/* Disable sleeps for safety */		
}


static struct GPIO_port PB = {
	.DDR = &DDRB,
	.PIN = &PINB,
	.PORT = &PORTB,
};


static struct segm_Display display = {
	.SHCP = {.port = &PB, .pin = 0},
	.STCP = {.port = &PB, .pin = 1},
	.DS   = {.port = &PB, .pin = 2},
	.delay_func = &_delay_loop_1,	/* 3 cycles / loop, busy wait */
	.sleep_ms_func = &sleep_ms,	/* 3 cycles / loop, busy wait */
	.is_comm_anode = false		/* We have common cathode display */
};


static struct GPIO_port PD = {
	.DDR = &DDRD,
	.PIN = &PIND,
	.PORT = &PORTD,
};


	/* add buttons */
struct segm_Pin btn1 = {.port = &PD, .pin = 2},
		btn2 = {.port = &PD, .pin = 3},
		btn3 = {.port = &PD, .pin = 4};

bool poll_btn_low(struct segm_Pin *btn)
{
	if (!(*(btn->port->PIN) & (1 << btn->pin))) {
		sleep_ms(30);
		if (!(*(btn->port->PIN) & (1 << btn->pin)))
			return true;
	}
	return false;
}


void set_minutes(struct clock *main_minutes){

	/*XX:01, XX:02, XX:03 ....*/
	main_minutes->min_l++;
	/* XX:09 -> XX */
	if(main_minutes->min_l == 10){
		main_minutes->min_r++;

		/* XX:59 -> XX:00 */
		if(main_minutes->min_r == 6) 
		  main_minutes->min_r = 0;
		  
		 main_minutes->min_l = 0;	
	}
}


void set_hours(struct clock *main_hours){

	/* hour++ */
	main_hours->hours_r++; 
 	/* count hour with overflow to 23:59 */

	if(main_hours->hours_r == 10){
 	  	 main_hours->hours_l++;	
	  	 main_hours->hours_r = 0; /* 12:59 -> 13:00 */
	}
	/* 23:59 -> 00:00 */
	if(main_hours->hours_l == 2 && main_hours->hours_r == 4){
		main_hours->hours_l = 0;
		main_hours->hours_r = 0;
	}
}

void init_timer1_by_overflow(void)
{

	TCCR1B |= (1 << CS12) |  (1 << CS10);

	/* reset by timer overflow */
	TIMSK1 |= (1 << TOIE1);
	/* generate 1 second */
	TCNT1 = 0xFFFF - (F_CPU/1024);
}


void init_button_setup(void)
{

	/* Configure buttons for input with pullup R */
	*(btn1.port->DDR) &= ~(1 << btn1.pin);
	*(btn1.port->PORT) |= (1 << btn1.pin);
	*(btn2.port->DDR) &= ~(1 << btn2.pin);
	*(btn2.port->PORT) |= (1 << btn2.pin);

	/* alarm mode button */
	*(btn3.port->DDR) &= ~(1 << btn3.pin);
	*(btn3.port->PORT) |= (1 << btn3.pin);


	 /* switch on Output interrupts by falling edge for PD2 and PD3 */
	EICRA |= (1 << ISC10) | (1 << ISC00);
	EIMSK |= (1 << INT0) | (1 << INT1);
}


int main(void){


	DDRD |= (1 << 6);

	bool alarm_status;
	segm_init(&display);
	init_timer1_by_overflow();

	/* control */
	init_button_setup();
	

	sei(); 

	while(1){
	segm_indicate4(&display, segm_time); /* show real time */

		/* run alarm mode setting*/
		while(poll_btn_low(&btn3)){

			segm_indicate4(&display, segm_alarm);

			if(poll_btn_low(&btn1)){
				soft_delay(2000);
				if(poll_btn_low(&btn1)){			
					soft_delay(2000);
					set_hours(&alarm); 
					segm_alarm[0] = segm_sym_table[alarm.hours_l];
					segm_alarm[1] = segm_sym_table[alarm.hours_r];
				}
		
			}

			if(poll_btn_low(&btn2)){
				soft_delay(2000);
				if(poll_btn_low(&btn2)){			
					soft_delay(2000);
					set_minutes(&alarm); 
					
					segm_alarm[2] = segm_sym_table[alarm.min_r];
					segm_alarm[3] = segm_sym_table[alarm.min_l];
				}

			}		
		}

		if(alarm_status = (time.hours_l == alarm.hours_l) & 
		         	  (time.hours_r == alarm.hours_r) &
				  (time.min_l == alarm.min_l) 	  &
				  (time.min_r == alarm.min_r)){
					/* timer for switch off alarm */
					if(time.seconds < 20){
						  PORTD |= (1 << 6);
						  sleep_ms(500);
						  PORTD &= ~(1 << 6);	
					}
			}
	}
}


ISR(TIMER1_OVF_vect, ISR_BLOCK)
{	
 	if(time.seconds >= 60) {
		time.seconds = 0;
		/* set minutes */
		time.min_l++;
		if(time.min_l == 10){
		 	time.min_r++;	
			if(time.min_r == 6){
			  time.min_r = 0;
			  set_hours(&time);
			  }
			time.min_l = 0;	
		}

		/* set time of segments */
		segm_time[0] = segm_sym_table[time.hours_l];
		segm_time[1] = segm_sym_table[time.hours_r];
		segm_time[2] = segm_sym_table[time.min_r];
		segm_time[3] = segm_sym_table[time.min_l];

	}

	time.seconds++; 
	/* generate 1 second */
	TCNT1 = 0xFFFF - (F_CPU/1024);
}

/* hour counter interrupt */
ISR(INT0_vect)
{
	if(!(poll_btn_low(&btn3)))
	{
		if (poll_btn_low(&btn1))
			if (poll_btn_low(&btn1)){
				set_hours(&time);
				/* fill hours */
				segm_time[0] = segm_sym_table[time.hours_l];
				segm_time[1] = segm_sym_table[time.hours_r];
			}
		}
}

/* min counter interrupt */
ISR(INT1_vect)
{
	if(!(poll_btn_low(&btn3)))
	{
		if (poll_btn_low(&btn2))
			if (poll_btn_low(&btn2)){
				set_minutes(&time);
				/* fill mins */
				segm_time[2] = segm_sym_table[time.min_r];
				segm_time[3] = segm_sym_table[time.min_l];
			}
	}
}
