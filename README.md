# ESP32-stepper-motor-control by MCPWM
ESP32 Arduino stepper motor accurately control 

![stepper motor](/motor.PNG)

![control sequence](/wave.PNG)

> https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/mcpwm.html

> ESP32 MCPWM module has not well implemented in the IDF api and Arduino core, here we access the MCPWM registers directly to generate a perfect waveform to control the 24BYJ48 or similar stepper motor accurately!

> ⚠ A new RTOS task using traditional GPIO control and delay() method is not accurate when the thread is getting busy.
