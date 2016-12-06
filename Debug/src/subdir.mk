################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/cr_startup_lpc82x.c \
../src/crp.c \
../src/oscillo_main.c \
../src/sysinit.c \
../src/uart.c 

OBJS += \
./src/cr_startup_lpc82x.o \
./src/crp.o \
./src/oscillo_main.o \
./src/sysinit.o \
./src/uart.o 

C_DEPS += \
./src/cr_startup_lpc82x.d \
./src/crp.d \
./src/oscillo_main.d \
./src/sysinit.d \
./src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DNO_BOARD_LIB -D__MTB_DISABLE -D__LPC82X__ -DDEBUG -D__CODE_RED -DCORE_M0PLUS -D__USE_LPCOPEN -I"/Users/masa/LPCXpresso/824/lpc_chip_82x/inc" -I/Users/masa/LPCXpresso/common/lib -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


