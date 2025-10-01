# INTRODUCTION
This project demonstrate the handling of external interupt (EXTI) event using
WeAct STM32F103C8T6 Bluepill plus board.

Once turned on, onboard LED (PB2) toggles slowly every one second. 

The KEY button of this board connects to pin A0. Once pressed, onboard LED
(PB2) will flash rapidly.

# Settings notes
PA0 is selected pull-down, GPIO_EXTI0, enabled in NVIC (EXTI line0 interrupt -> true)