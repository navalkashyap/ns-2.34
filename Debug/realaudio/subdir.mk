################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../realaudio/realaudio.cc 

O_SRCS += \
../realaudio/realaudio.o 

CC_DEPS += \
./realaudio/realaudio.d 

OBJS += \
./realaudio/realaudio.o 


# Each subdirectory must supply rules for building sources it contributes
realaudio/%.o: ../realaudio/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


