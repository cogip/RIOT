APPLICATION = cortex
RIOTBASE ?= $(CURDIR)/../../..
BOARD ?= native
USEMODULE += xtimer
USEMODULE += motor_driver

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += schedstatistics
USEMODULE += ps

QUIET ?= 1

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

FEATURES_REQUIRED += periph_timer
FEATURES_REQUIRED += periph_qdec
FEATURES_REQUIRED += periph_pwm

include $(RIOTBASE)/Makefile.include
