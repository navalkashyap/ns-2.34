################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include xcp/subdir.mk
-include wpan/subdir.mk
-include webcache/subdir.mk
-include trace/subdir.mk
-include tora/subdir.mk
-include tools/subdir.mk
-include tmix/subdir.mk
-include tcp/linux/src/subdir.mk
-include tcp/linux/subdir.mk
-include tcp/subdir.mk
-include tcl/test/subdir.mk
-include src_rtg/subdir.mk
-include sensor-nets/subdir.mk
-include sctp/subdir.mk
-include satellite/subdir.mk
-include routing/subdir.mk
-include routealgo/subdir.mk
-include realaudio/subdir.mk
-include rap/subdir.mk
-include queue/subdir.mk
-include qs/subdir.mk
-include pushback/subdir.mk
-include plm/subdir.mk
-include pgm/subdir.mk
-include packmime/subdir.mk
-include nix/subdir.mk
-include mpls/subdir.mk
-include mobile/subdir.mk
-include mcast/subdir.mk
-include mac/subdir.mk
-include linkstate/subdir.mk
-include link/subdir.mk
-include lib/subdir.mk
-include indep-utils/webtrace-conv/ucb/subdir.mk
-include indep-utils/webtrace-conv/nlanr/subdir.mk
-include indep-utils/webtrace-conv/epa/subdir.mk
-include indep-utils/webtrace-conv/dec/subdir.mk
-include indep-utils/propagation/subdir.mk
-include indep-utils/model-gen/subdir.mk
-include indep-utils/dosreduce/subdir.mk
-include indep-utils/dosdbell/subdir.mk
-include indep-utils/cmu-scen-gen/setdest/subdir.mk
-include imep/subdir.mk
-include gen/subdir.mk
-include gaf/subdir.mk
-include emulate/subdir.mk
-include empweb/subdir.mk
-include dsr/subdir.mk
-include dsdv/subdir.mk
-include doc/figures/subdir.mk
-include diffusion3/ns/subdir.mk
-include diffusion3/lib/nr/subdir.mk
-include diffusion3/lib/main/subdir.mk
-include diffusion3/lib/subdir.mk
-include diffusion3/filters/rmst/subdir.mk
-include diffusion3/filters/misc/subdir.mk
-include diffusion3/filters/gear/subdir.mk
-include diffusion3/filters/diffusion/subdir.mk
-include diffusion3/filter_core/subdir.mk
-include diffusion3/apps/rmst_examples/subdir.mk
-include diffusion3/apps/ping/subdir.mk
-include diffusion3/apps/gear_examples/subdir.mk
-include diffusion/subdir.mk
-include diffserv/subdir.mk
-include delaybox/subdir.mk
-include common/subdir.mk
-include classifier/subdir.mk
-include chargingsys/routing/subdir.mk
-include chargingsys/phy/subdir.mk
-include chargingsys/mac/subdir.mk
-include chargingsys/app/subdir.mk
-include chargingsys/subdir.mk
-include baytcp/subdir.mk
-include asim/subdir.mk
-include apps/subdir.mk
-include aomdv/subdir.mk
-include aodv/subdir.mk
-include adc/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: ns-2.34.exe

# Tool invocations
ns-2.34.exe: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cygwin C++ Linker'
	g++  -o "ns-2.34.exe" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(OBJS)$(C_UPPER_DEPS)$(CXX_DEPS)$(CPP_DEPS)$(C_DEPS) ns-2.34.exe
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
