################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../chargingsys/ctrace.cc 

O_SRCS += \
../chargingsys/ctrace.o 

CC_DEPS += \
./chargingsys/ctrace.d 

OBJS += \
./chargingsys/ctrace.o 


# Each subdirectory must supply rules for building sources it contributes
chargingsys/%.o: ../chargingsys/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


