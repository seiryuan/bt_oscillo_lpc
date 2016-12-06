################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: bt_oscillo_lpc824.axf

# Tool invocations
bt_oscillo_lpc824.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/Users/masa/LPCXpresso/824/lpc_chip_82x/Release" -Xlinker -Map="bt_oscillo_lpc824.map" -Xlinker --gc-sections -mcpu=cortex-m0 -mthumb -T "oscillo_v008_Release.ld" -o "bt_oscillo_lpc824.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) bt_oscillo_lpc824.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "bt_oscillo_lpc824.axf"; # arm-none-eabi-objcopy -v -O binary "bt_oscillo_lpc824.axf" "bt_oscillo_lpc824.bin" ; # checksum -p LPC824 -d "bt_oscillo_lpc824.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
