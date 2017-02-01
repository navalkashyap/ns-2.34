################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../chargingsys/phy/charging-energy-model.cc \
../chargingsys/phy/wireless-chargingphy.cc 

O_SRCS += \
../chargingsys/phy/charging-energy-model.o \
../chargingsys/phy/wireless-chargingphy.o 

CC_DEPS += \
./chargingsys/phy/charging-energy-model.d \
./chargingsys/phy/wireless-chargingphy.d 

OBJS += \
./chargingsys/phy/charging-energy-model.o \
./chargingsys/phy/wireless-chargingphy.o 


# Each subdirectory must supply rules for building sources it contributes
chargingsys/phy/%.o: ../chargingsys/phy/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


