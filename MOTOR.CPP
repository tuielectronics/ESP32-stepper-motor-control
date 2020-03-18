
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

/*
 * motor speed = 0, 1, 2, 3, 4, 5, 6, 7
 * motor direction = 0, 1
 * motorSequence structure
     0                1               2               
      0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 
     +---------------+---------------+---------------+
     |   command     |   command 1   |L| L   |R| R   |
     |     group     |     steps     |E| S   |I| S   |
     |    length     |   (8 bit)     |F| P   |G| P   |
     |    (1~8)      |               |T| E   |T| E   |
     |    (8 bit)    |               |D| E   |D| E   |
     |               |               |I| D   |I| D   |
     |               |               |R| 3bit|R| 3bit|
     +---------------+---------------+
     3                4        
      0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 
     +---------------+---------------+
     |   command 2   |L| L   |R| R   |
     |     steps     |E| S   |I| S   |
     |   (8 bit)     |F| P   |G| P   |
     |               |T| E   |T| E   |
     |               |D| E   |D| E   |
     |               |I| D   |I| D   |
     |               |R| 3bit|R| 3bit|
     +---------------+---------------+
     5                6        
      0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 
     +---------------+---------------+
     |   command 3   |L| L   |R| R   |
     |     steps     |E| S   |I| S   |
     |   (8 bit)     |F| P   |G| P   |
     |               |T| E   |T| E   |
     |               |D| E   |D| E   |
     |               |I| D   |I| D   |
     |               |R| 3bit|R| 3bit|
     +---------------+---------------+
     7                8        
      0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 
     +---------------+---------------+
     |   command 4   |L| L   |R| R   |
     |     steps     |E| S   |I| S   |
     |   (8 bit)     |F| P   |G| P   |
     |               |T| E   |T| E   |
     |               |D| E   |D| E   |
     |               |I| D   |I| D   |
     |               |R| 3bit|R| 3bit|
     +---------------+---------------+
     9                10        
      0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 
     +---------------+---------------+
     |   command 5   |L| L   |R| R   |
     |     steps     |E| S   |I| S   |
     |   (8 bit)     |F| P   |G| P   |
     |               |T| E   |T| E   |
     |               |D| E   |D| E   |
     |               |I| D   |I| D   |
     |               |R| 3bit|R| 3bit|
     +---------------+---------------+
     11               12        
      0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 
     +---------------+---------------+
     |   command 6   |L| L   |R| R   |
     |     steps     |E| S   |I| S   |
     |   (8 bit)     |F| P   |G| P   |
     |               |T| E   |T| E   |
     |               |D| E   |D| E   |
     |               |I| D   |I| D   |
     |               |R| 3bit|R| 3bit|
     +---------------+---------------+
     13               14        
      0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 
     +---------------+---------------+
     |   command 7   |L| L   |R| R   |
     |     steps     |E| S   |I| S   |
     |   (8 bit)     |F| P   |G| P   |
     |               |T| E   |T| E   |
     |               |D| E   |D| E   |
     |               |I| D   |I| D   |
     |               |R| 3bit|R| 3bit|
     +---------------+---------------+
     15               16        
      0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 
     +---------------+---------------+
     |   command 8   |L| L   |R| R   |
     |     steps     |E| S   |I| S   |
     |   (8 bit)     |F| P   |G| P   |
     |               |T| E   |T| E   |
     |               |D| E   |D| E   |
     |               |I| D   |I| D   |
     |               |R| 3bit|R| 3bit|
     +---------------+---------------+
*/ 
volatile static uint16_t motorSequence[17];

volatile static uint16_t motorByteSequence[5];

volatile static uint16_t motorStatusFlag = 0;

volatile static uint16_t motorNextSteps = 0;
//volatile static uint32_t mcpwmInterruptChannel = 1;
volatile uint8_t commandStatusAsync = 0;
void mcpwmGpioInitialize()
{
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, STEPPER_A1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, STEPPER_A2);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, STEPPER_A3);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, STEPPER_A4);


  //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_SYNC_0, BATTERY_KEY_PIN);

  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, STEPPER_B1);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, STEPPER_B2);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, STEPPER_B3);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, STEPPER_B4);



  //mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_SYNC_0, BATTERY_KEY_PIN);
}
void mcpwmResetOperator() {
  MCPWM0.channel[0].generator[0].utez = 1;
  MCPWM0.channel[0].generator[0].utea = 1;
  MCPWM0.channel[0].generator[0].uteb = 1;
  MCPWM0.channel[0].generator[0].utep = 1;

  MCPWM0.channel[0].generator[1].utez = 1;
  MCPWM0.channel[0].generator[1].utea = 1;
  MCPWM0.channel[0].generator[1].uteb = 1;
  MCPWM0.channel[0].generator[1].utep = 1;

  MCPWM0.channel[1].generator[0].utez = 1;
  MCPWM0.channel[1].generator[0].utea = 1;
  MCPWM0.channel[1].generator[0].uteb = 1;
  MCPWM0.channel[1].generator[0].utep = 1;

  MCPWM0.channel[1].generator[1].utez = 1;
  MCPWM0.channel[1].generator[1].utea = 1;
  MCPWM0.channel[1].generator[1].uteb = 1;
  MCPWM0.channel[1].generator[1].utep = 1;

  MCPWM1.channel[0].generator[0].utez = 1;
  MCPWM1.channel[0].generator[0].utea = 1;
  MCPWM1.channel[0].generator[0].uteb = 1;
  MCPWM1.channel[0].generator[0].utep = 1;

  MCPWM1.channel[0].generator[1].utez = 1;
  MCPWM1.channel[0].generator[1].utea = 1;
  MCPWM1.channel[0].generator[1].uteb = 1;
  MCPWM1.channel[0].generator[1].utep = 1;

  MCPWM1.channel[1].generator[0].utez = 1;
  MCPWM1.channel[1].generator[0].utea = 1;
  MCPWM1.channel[1].generator[0].uteb = 1;
  MCPWM1.channel[1].generator[0].utep = 1;

  MCPWM1.channel[1].generator[1].utez = 1;
  MCPWM1.channel[1].generator[1].utea = 1;
  MCPWM1.channel[1].generator[1].uteb = 1;
  MCPWM1.channel[1].generator[1].utep = 1;
}


void mcpwmStart(uint32_t freq1, uint32_t freq2, uint8_t direction1, uint8_t direction2, uint16_t nextSteps) {


  if (freq1 > 0) {
    MCPWM0.timer[1].sync.timer_phase = freq1 * 200;
  }
  else{
    MCPWM0.timer[1].sync.timer_phase = freq2 * 200;
  }
  if (freq2 > 0) {
    MCPWM1.timer[1].sync.timer_phase = freq2 * 200; // the bigger the more left
  }
  else{
    MCPWM1.timer[1].sync.timer_phase = freq1 * 200;
  }

  MCPWM0.timer[1].sync.sync_sw = 1;
  MCPWM1.timer[1].sync.sync_sw = 1;
  _delay_ms(1);
  MCPWM0.timer[1].sync.sync_sw = 0;
  MCPWM1.timer[1].sync.sync_sw = 0;
  _delay_ms(100);

  if (freq1 > 0) {
    MCPWM0.timer[0].period.period = freq1 * 400;//will change pwm frequency 50->20000
    MCPWM0.timer[1].period.period = freq1 * 400;//
  }
  else{
    MCPWM0.timer[0].period.period = freq2 * 400;
    MCPWM0.timer[1].period.period = freq2 * 400;
  }
  if (freq2 > 0) {
    MCPWM1.timer[0].period.period = freq2 * 400;//will change pwm frequency
    MCPWM1.timer[1].period.period = freq2 * 400;//will change pwm frequency
  }
  else{
    MCPWM1.timer[0].period.period = freq1 * 400;
    MCPWM1.timer[1].period.period = freq1 * 400;
  }

  if (direction1 == 1 && freq1 > 0) {

    MCPWM0.channel[0].cmpr_value[0].cmpr_val = freq1 * 100;
    MCPWM0.channel[0].cmpr_value[1].cmpr_val = freq1 * 300;

    MCPWM0.channel[1].cmpr_value[0].cmpr_val = freq1 * 100;
    MCPWM0.channel[1].cmpr_value[1].cmpr_val = freq1 * 300;

    MCPWM0.channel[0].generator[0].utez = 2;
    MCPWM0.channel[0].generator[0].utea = 1;
    MCPWM0.channel[0].generator[0].uteb = 0;
    MCPWM0.channel[0].generator[0].utep = 0;

    MCPWM0.channel[0].generator[1].utez = 1;
    MCPWM0.channel[0].generator[1].utea = 0;
    MCPWM0.channel[0].generator[1].uteb = 2;
    MCPWM0.channel[0].generator[1].utep = 0;

    MCPWM0.channel[1].generator[0].utez = 2;
    MCPWM0.channel[1].generator[0].utea = 1;
    MCPWM0.channel[1].generator[0].uteb = 0;
    MCPWM0.channel[1].generator[0].utep = 0;

    MCPWM0.channel[1].generator[1].utez = 1;
    MCPWM0.channel[1].generator[1].utea = 0;
    MCPWM0.channel[1].generator[1].uteb = 2;
    MCPWM0.channel[1].generator[1].utep = 0;
  }
  else if (direction1 == 2  && freq1 > 0) {
    MCPWM0.channel[0].cmpr_value[0].cmpr_val = freq1 * 300;
    MCPWM0.channel[0].cmpr_value[1].cmpr_val = freq1 * 100;

    MCPWM0.channel[1].cmpr_value[0].cmpr_val = freq1 * 300;
    MCPWM0.channel[1].cmpr_value[1].cmpr_val = freq1 * 100;

    MCPWM0.channel[0].generator[0].utez = 1;
    MCPWM0.channel[0].generator[0].utea = 2;
    MCPWM0.channel[0].generator[0].uteb = 0;
    MCPWM0.channel[0].generator[0].utep = 0;

    MCPWM0.channel[0].generator[1].utez = 2;
    MCPWM0.channel[0].generator[1].utea = 0;
    MCPWM0.channel[0].generator[1].uteb = 1;
    MCPWM0.channel[0].generator[1].utep = 0;

    MCPWM0.channel[1].generator[0].utez = 1;
    MCPWM0.channel[1].generator[0].utea = 2;
    MCPWM0.channel[1].generator[0].uteb = 0;
    MCPWM0.channel[1].generator[0].utep = 0;

    MCPWM0.channel[1].generator[1].utez = 2;
    MCPWM0.channel[1].generator[1].utea = 0;
    MCPWM0.channel[1].generator[1].uteb = 1;
    MCPWM0.channel[1].generator[1].utep = 0;
  }
  if (direction2 == 1 && freq2 > 0) {

    MCPWM1.channel[0].cmpr_value[0].cmpr_val = freq2 * 100;
    MCPWM1.channel[0].cmpr_value[1].cmpr_val = freq2 * 300;

    MCPWM1.channel[1].cmpr_value[0].cmpr_val = freq2 * 100;
    MCPWM1.channel[1].cmpr_value[1].cmpr_val = freq2 * 300;

    MCPWM1.channel[0].generator[0].utez = 2;
    MCPWM1.channel[0].generator[0].utea = 1;
    MCPWM1.channel[0].generator[0].uteb = 0;
    MCPWM1.channel[0].generator[0].utep = 0;

    MCPWM1.channel[0].generator[1].utez = 1;
    MCPWM1.channel[0].generator[1].utea = 0;
    MCPWM1.channel[0].generator[1].uteb = 2;
    MCPWM1.channel[0].generator[1].utep = 0;

    MCPWM1.channel[1].generator[0].utez = 2;
    MCPWM1.channel[1].generator[0].utea = 1;
    MCPWM1.channel[1].generator[0].uteb = 0;
    MCPWM1.channel[1].generator[0].utep = 0;

    MCPWM1.channel[1].generator[1].utez = 1;
    MCPWM1.channel[1].generator[1].utea = 0;
    MCPWM1.channel[1].generator[1].uteb = 2;
    MCPWM1.channel[1].generator[1].utep = 0;
  }
  else if (direction2 == 2 && freq2 > 0) {
    MCPWM1.channel[0].cmpr_value[0].cmpr_val = freq2 * 300;
    MCPWM1.channel[0].cmpr_value[1].cmpr_val = freq2 * 100;

    MCPWM1.channel[1].cmpr_value[0].cmpr_val = freq2 * 300;
    MCPWM1.channel[1].cmpr_value[1].cmpr_val = freq2 * 100;

    MCPWM1.channel[0].generator[0].utez = 1;
    MCPWM1.channel[0].generator[0].utea = 2;
    MCPWM1.channel[0].generator[0].uteb = 0;
    MCPWM1.channel[0].generator[0].utep = 0;

    MCPWM1.channel[0].generator[1].utez = 2;
    MCPWM1.channel[0].generator[1].utea = 0;
    MCPWM1.channel[0].generator[1].uteb = 1;
    MCPWM1.channel[0].generator[1].utep = 0;

    MCPWM1.channel[1].generator[0].utez = 1;
    MCPWM1.channel[1].generator[0].utea = 2;
    MCPWM1.channel[1].generator[0].uteb = 0;
    MCPWM1.channel[1].generator[0].utep = 0;

    MCPWM1.channel[1].generator[1].utez = 2;
    MCPWM1.channel[1].generator[1].utea = 0;
    MCPWM1.channel[1].generator[1].uteb = 1;
    MCPWM1.channel[1].generator[1].utep = 0;
  }

  motorNextSteps = nextSteps;
  motorStatusFlag = 0;
  if (direction1 == 1) {
    MCPWM0.int_ena.timer1_tep_int_ena = 1;
  }
  else if (direction1 == 2) {
    MCPWM0.int_ena.timer0_tep_int_ena = 1;
  }


}
void mcpwmStop() {

  mcpwmResetOperator();
  if (MCPWM0.int_ena.timer1_tep_int_ena) {
    MCPWM0.int_ena.timer1_tep_int_ena = 0;
  }
  if (MCPWM0.int_ena.timer0_tep_int_ena) {
    MCPWM0.int_ena.timer0_tep_int_ena = 0;
  }
  motorNextSteps = 0;
}

static void IRAM_ATTR mcpwm_isr_handler(void *arg)
{
  if (MCPWM0.int_st.timer1_tep_int_st) {
    motorStatusFlag++;
    if (motorStatusFlag > motorNextSteps) {
      //SERIAL_DEBUG("[motor] isr_handler %d\n", motorStatusFlag);
      //motorStatusFlag = 0;
      mcpwmStop();

    }
    MCPWM0.int_clr.timer1_tep_int_clr = 1;
  }
  if (MCPWM0.int_st.timer0_tep_int_st) {
    motorStatusFlag++;
    if (motorStatusFlag > motorNextSteps) {

      //SERIAL_DEBUG("[motor] isr_handler %d\n", motorStatusFlag);
      //motorStatusFlag = 0;
      mcpwmStop();

    }
    MCPWM0.int_clr.timer0_tep_int_clr = 1;
  }
}

void mcpwmConfig() {
  uint32_t freq1 = 50;
  uint32_t freq2 = 50;

  mcpwmGpioInitialize();

  MCPWM0.clk_cfg.prescale = 15;
  MCPWM1.clk_cfg.prescale = 15;

  mcpwm_isr_register(MCPWM_UNIT_0, mcpwm_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

  MCPWM0.timer[0].period.prescale = 9;
  MCPWM0.timer[0].period.period = freq1 * 400;//will change pwm frequency 50->20000
  MCPWM0.timer[0].period.upmethod = 0;//immediate

  MCPWM0.timer[1].period.prescale = 9;
  MCPWM0.timer[1].period.period = freq1 * 400;//
  MCPWM0.timer[1].period.upmethod = 0;//immediate

  MCPWM1.timer[0].period.prescale = 9;
  MCPWM1.timer[0].period.period = freq2 * 400;//will change pwm frequency
  MCPWM1.timer[0].period.upmethod = 0;//immediate

  MCPWM1.timer[1].period.prescale = 9;
  MCPWM1.timer[1].period.period = freq2 * 400;//
  MCPWM1.timer[1].period.upmethod = 0;//immediate

  MCPWM0.timer_sel.operator0_sel = 0;
  MCPWM0.timer_sel.operator1_sel = 1;
  MCPWM0.timer_synci_cfg.t1_in_sel = 1;
  //MCPWM0.timer[1].sync.timer_phase = freq1 * 228;

  MCPWM0.timer[0].sync.out_sel = 1;
  MCPWM0.timer[1].sync.out_sel = 0;
  MCPWM0.timer[1].sync.in_en = 1;


  MCPWM1.timer_sel.operator0_sel = 0;
  MCPWM1.timer_sel.operator1_sel = 1;
  MCPWM1.timer_synci_cfg.t1_in_sel = 1;
  //MCPWM1.timer[1].sync.timer_phase = freq2 * 228; // the bigger the more left
  MCPWM1.timer[0].sync.out_sel = 1;
  MCPWM1.timer[1].sync.out_sel = 0;
  MCPWM1.timer[1].sync.in_en = 1;

  MCPWM0.channel[0].cmpr_value[0].cmpr_val = freq1 * 100;
  MCPWM0.channel[0].cmpr_value[1].cmpr_val = freq1 * 300;

  MCPWM0.channel[1].cmpr_value[0].cmpr_val = freq1 * 100;
  MCPWM0.channel[1].cmpr_value[1].cmpr_val = freq1 * 300;

  MCPWM1.channel[0].cmpr_value[0].cmpr_val = freq2 * 100;
  MCPWM1.channel[0].cmpr_value[1].cmpr_val = freq2 * 300;

  MCPWM1.channel[1].cmpr_value[0].cmpr_val = freq2 * 100;
  MCPWM1.channel[1].cmpr_value[1].cmpr_val = freq2 * 300;

  mcpwmResetOperator();


  MCPWM0.timer[0].mode.start = 2;//free run
  MCPWM0.timer[0].mode.mode = 1;//increase

  MCPWM0.timer[1].mode.start = 2;//free run
  MCPWM0.timer[1].mode.mode = 1;//increase
  //delay(1);



  MCPWM1.timer[0].mode.start = 2;//free run
  MCPWM1.timer[0].mode.mode = 1;//increase

  MCPWM1.timer[1].mode.start = 2;//free run
  MCPWM1.timer[1].mode.mode = 1;//increase

}

static void stepperTask(void *arg) {
  uint16_t freq1, freq2, dir1, dir2, duration;
  uint16_t len = motorByteSequence[0];
  uint16_t j = 0;
  do {
    if (bitRead(motorByteSequence[1 + j * 2 + 1], 7)) {
      dir1 = 1;
    }
    else {
      dir1 = 2;
    }
    // 2 3 4 5 -> 0 1 2 3
    uint16_t leftP = (motorByteSequence[1 + j * 2 + 1] & 0x70) >> 4;
    if (leftP == 1) {
      freq1 = 40;
    }
    else if (leftP == 2) {
      freq1 = 60;
    }
    else if (leftP == 3) {
      freq1 = 80;
    }
    else if (leftP == 4) {
      freq1 = 100;
    }
    else if (leftP == 5) {
      freq1 = 120;
    }
    else if (leftP == 6) {
      freq1 = 140;
    }
    else if (leftP == 7) {
      freq1 = 160;
    }
    else {
      freq1 = 0;
    }

    if (bitRead(motorByteSequence[1 + j * 2 + 1], 3)) {
      dir2 = 1;
    }
    else {
      dir2 = 2;
    }
    uint16_t rightP = motorByteSequence[1 + j * 2 + 1] & 0x07;
    if (rightP == 1) {
      freq2 = 40;
    } else if (rightP == 2) {
      freq2 = 60;
    }
    else if (rightP == 3) {
      freq2 = 80;
    }
    else if (rightP == 4) {
      freq2 = 100;
    }
    else if (rightP == 5) {
      freq2 = 120;
    }
    else if (rightP == 6) {
      freq2 = 140;
    }
    else if (rightP == 7) {
      freq2 = 160;
    }
    else {
      freq2 = 0;
    }

    if (freq1 == freq2 && freq2 > 0) {
      SERIAL_DEBUG("motor go straight, freq1 = %d, freq2 = %d, dir1 = %d, dir2 = %d, steps = %d, progress: %d / %d\n", freq1, freq2, dir1, dir2, motorByteSequence[1 + j * 2], j, len);
      
      mcpwmStart(freq1, freq2, dir1, dir2, motorByteSequence[1 + j * 2]);
      while (motorNextSteps != 0) {
        _delay_ms(1);
      }
      //
    }
    else if (freq1 != freq2 && (freq1 > 0 || freq2 > 0)) {
      SERIAL_DEBUG("motor go circle, freq1 = %d, freq2 = %d, dir1 = %d, dir2 = %d, steps = %d, progress: %d / %d\n", freq1, freq2, dir1, dir2, motorByteSequence[1 + j * 2], j, len);
      mcpwmStart(freq1, freq2, dir1, dir2, motorByteSequence[1 + j * 2]);
      while (motorNextSteps != 0) {
        _delay_ms(1);
      }
    }
    j++;
  } while (j < len);

  commandStatusAsync = 2;
  //SERIAL_DEBUG("[motor] finished\n");
  vTaskDelete(NULL);
}

void stepperRun() {
  xTaskCreate(stepperTask, "stepperTask", 4096, NULL, 5, NULL);
}
