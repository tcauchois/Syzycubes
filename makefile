# Add inputs and outputs from these tool invocations to the build variables
PROJECT := syzycube
###
LSS := syzycube.lss
FLASH_IMAGE := syzycube.hex
ELF := syzycube.elf
SIZEDUMMY := sizedummy

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS := ADXL345.c main.c console.c TLC5947DAP.c

S_UPPER_SRCS := 
#ffft.S 

OBJS := $(C_SRCS:%.c=%.o) $(S_UPPER_SRCS:%.S=%.o)

DEPS := $(OBJS:%.o=%.d)

LIBS := -lm

# All Target
all: $(PROJECT).elf secondary-outputs

# Tool invocations
$(PROJECT).elf: $(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,$(PROJECT).map -mmcu=atmega328p -o"$(PROJECT).elf" $(OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

$(PROJECT).lss: $(PROJECT).elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S $(PROJECT).elf  >"$(PROJECT).lss"
	@echo 'Finished building: $@'
	@echo ' '

$(PROJECT).hex: $(PROJECT).elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex $(PROJECT).elf  "$(PROJECT).hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: $(PROJECT).elf
	@echo 'Invoking: Print Size'
	-avr-size --format=berkeley -t $(PROJECT).elf
	@echo 'Finished building: $@'
	@echo ' '

# Each subdirectory must supply rules for building sources it contributes
%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -std=c99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: %.S
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Assembler'
	avr-gcc -x assembler-with-cpp -mmcu=atmega328p -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(PROJECT).elf $(PROJECT).lss $(PROJECT).hex $(PROJECT).map $(OBJS) $(DEPS)
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents sizedummy
.SECONDARY:

-include $(DEPS)
