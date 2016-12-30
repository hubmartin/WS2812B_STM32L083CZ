# WS2812B_STM32L083CZ
Updated driver for STM32 Low power line of MCUs

**See my other repositories for F3 and F4 port.**
There is also documentation how to use this lib. Other libraries are bit different and use DMA > GPIO writes.
This lib is using PWM because L0 dont have DMA->GPIO capability. So instead of writing DMA to the GPIO I write one bit to the compare register and I'm changing the duty cycle of the PWM each PWM cycle. The DMA is triggered by the same timer generating output bits by the UP event.
