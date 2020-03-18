/*
   define GPIO PIN for stepper motor A and stepper motor B
*/
#define STEPPER_A1 16
#define STEPPER_A2 4
#define STEPPER_A3 15
#define STEPPER_A4 13

#define STEPPER_B1 14
#define STEPPER_B2 27
#define STEPPER_B3 26
#define STEPPER_B4 25

#define _delay_ms(ms) vTaskDelay((ms) / portTICK_RATE_MS)
#define DEBUG_ESP_ON 1 // comment this line to disable serial log
#ifdef DEBUG_ESP_ON
#define SERIAL_DEBUG(...) Serial.printf( __VA_ARGS__ )

#else
#define SERIAL_DEBUG(...)
#endif
#include "MOTOR.CPP"
void setup() {
  Serial.begin(115200);
  SERIAL_DEBUG("Serial begin\n");
  mcpwmConfig();

  motorByteSequence[0] = 8;

  motorByteSequence[1] = 89;
  motorByteSequence[2] = 153;

  motorByteSequence[3] = 89;
  motorByteSequence[4] = 17;

  motorByteSequence[5] = 89;
  motorByteSequence[6] = 153;

  motorByteSequence[7] = 89;
  motorByteSequence[8] = 17;

  motorByteSequence[9] = 89;
  motorByteSequence[10] = 153;

  motorByteSequence[11] = 89;
  motorByteSequence[12] = 17;

  motorByteSequence[13] = 89;
  motorByteSequence[14] = 153;

  motorByteSequence[15] = 89;
  motorByteSequence[16] = 17;

  SERIAL_DEBUG("set motor motorByteSequence length = %d.\n", motorByteSequence[0]);

  stepperRun();
  //commandStatusAsync = 4;
}

void loop() {
  if (commandStatusAsync == 2) {
    commandStatusAsync = 0;
    SERIAL_DEBUG("motor finished\n");
  }

}
