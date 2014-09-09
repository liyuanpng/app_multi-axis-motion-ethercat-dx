TARGET = MultiNodeDXmotionControlPlatform

APP_NAME = demo_app

USED_MODULES = module_xscope-wrapper module_adc module_blocks module_commutation module_ctrl_loops module_hall module_motor module_motorcontrol_common module_profile module_pwm_symmetrical module_qei module_statemachine

XCC_FLAGS_Debug = -g -O0
XCC_FLAGS_Release = -g -O3

# The VERBOSE variable, if set to 1, enables verbose output from the make system.
VERBOSE = 0

XMOS_MAKE_PATH ?= ../..
-include $(XMOS_MAKE_PATH)/xcommon/module_xcommon/build/Makefile.common
