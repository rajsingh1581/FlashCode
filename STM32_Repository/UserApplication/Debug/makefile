################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
UserApplication.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
UserApplication.list \


# All Target
all:
	+@make --no-print-directory main-build && make --no-print-directory post-build

# Main-build Target
main-build: UserApplication.elf secondary-outputs

# Tool invocations
UserApplication.elf: $(OBJS) $(USER_OBJS) D:\Code\STM32\STM32CUBEIDE\STM32_Repository\UserApplication\STM32F410RBTX_FLASH.ld
	arm-none-eabi-gcc -o "UserApplication.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Code\STM32\STM32CUBEIDE\STM32_Repository\UserApplication\STM32F410RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="UserApplication.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

UserApplication.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "UserApplication.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	arm-none-eabi-objcopy -O ihex UserApplication.elf UserApplication.hex
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
