################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../gaf/gaf.cc 

O_SRCS += \
../gaf/gaf.o 

CC_DEPS += \
./gaf/gaf.d 

OBJS += \
./gaf/gaf.o 


# Each subdirectory must supply rules for building sources it contributes
gaf/%.o: ../gaf/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


