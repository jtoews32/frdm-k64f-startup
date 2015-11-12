#!/bin/sh

arm-none-eabi-as -mcpu=cortex-m4 -mthumb -o blink-led.o blink-led.s
arm-none-eabi-ld -T blink-led.ld -o blink-led.elf blink-led.o
arm-none-eabi-objdump -d blink-led.elf > blink-led.elf.objdump.txt
arm-none-eabi-nm -n blink-led.elf
arm-none-eabi-objcopy -O binary blink-led.elf blink-led.bin