# Default RIOT bootloader
APPLICATION = riotboot_can

# Add RIOTBOOT CAN integration
USEMODULE += riotboot_can

# Include riotboot flash partition functionality
USEMODULE += riotboot_slot

# Use ztimer for conn_can timeouts
USEMODULE += ztimer
USEMODULE += ztimer_init
USEMODULE += ztimer_usec

# Increase bootloader size, CAN stack needs more than default 4KB
RIOTBOOT_LEN ?= 0x4000

include ../riotboot_common.mk

# Re-enable core_msg: the CAN stack (dll, device, router) uses RIOT IPC.
# riotboot_common.mk disables it, but we need it for conn_can. TODO: find good comment
USEMODULE += core_msg
