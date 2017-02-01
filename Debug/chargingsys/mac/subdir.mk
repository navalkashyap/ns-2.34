################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../chargingsys/mac/mac-sensor.cc \
../chargingsys/mac/rilmac.cc 

O_SRCS += \
../chargingsys/mac/rilmac.o 

CC_DEPS += \
./chargingsys/mac/mac-sensor.d \
./chargingsys/mac/rilmac.d 

OBJS += \
./chargingsys/mac/mac-sensor.o \
./chargingsys/mac/rilmac.o 


# Each subdirectory must supply rules for building sources it contributes
chargingsys/mac/%.o: ../chargingsys/mac/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


