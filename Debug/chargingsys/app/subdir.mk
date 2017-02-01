################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../chargingsys/app/chargingagent.cc \
../chargingsys/app/chargingapp.cc \
../chargingsys/app/rilagent.cc \
../chargingsys/app/rilapp.cc 

O_SRCS += \
../chargingsys/app/rilagent.o \
../chargingsys/app/rilapp.o 

CC_DEPS += \
./chargingsys/app/chargingagent.d \
./chargingsys/app/chargingapp.d \
./chargingsys/app/rilagent.d \
./chargingsys/app/rilapp.d 

OBJS += \
./chargingsys/app/chargingagent.o \
./chargingsys/app/chargingapp.o \
./chargingsys/app/rilagent.o \
./chargingsys/app/rilapp.o 


# Each subdirectory must supply rules for building sources it contributes
chargingsys/app/%.o: ../chargingsys/app/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


