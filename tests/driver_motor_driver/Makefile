include ../Makefile.tests_common

BOARD ?= nucleo-f446re

USEMODULE += motor_driver
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += xtimer

FEATURES_REQUIRED += periph_pwm
FEATURES_REQUIRED += periph_qdec
FEATURES_REQUIRED += periph_gpio

CFLAGS += -DLOG_LEVEL=LOG_DEBUG
CFLAGS += -DDEBUG_ASSERT_VERBOSE

include $(RIOTBASE)/Makefile.include
