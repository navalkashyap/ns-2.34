################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tcl/test/temp.c 

OBJS += \
./tcl/test/temp.o 

C_DEPS += \
./tcl/test/temp.d 


# Each subdirectory must supply rules for building sources it contributes
tcl/test/%.o: ../tcl/test/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


