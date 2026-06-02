@defgroup    boards_stm32h573i-dk STM32H573I-DK Discovery Kit
@ingroup     boards
@brief       Support for the STM32H573I-DK Discovery Kit

## Overview

The STM32H573I-DK is a Discovery Kit built around the STM32H573II (Cortex-M33,
250 MHz, 2 MB Flash, 640 KB RAM, TrustZone-capable). It exposes:

- on-board ST-LINK V3E debugger / programmer with VCP
- LAN8742A Ethernet PHY (RMII, RJ45)
- two FDCAN transceivers
- USB Type-C with USB-PD
- 32 Mbit OctoSPI Flash, 64 Mbit OctoSPI HyperRAM
- 240x240 round TFT, capacitive touch
- microSD card slot
- 4x user LEDs, user button, joystick

## Currently supported in RIOT

- UART (USART3 wired to the ST-LINK VCP on PD8/PD9)
- GPIO, timer (TIM2)
- Power management (basic)

Ethernet, FDCAN, USB, OctoSPI and the on-board peripherals will be enabled in
subsequent contributions.

## Flashing the board

The Discovery Kit ships with an on-board ST-LINK V3E. With OpenOCD installed,
build and flash the default example with:

```
BOARD=stm32h573i-dk make -C examples/hello-world flash term
```

The default programmer is `openocd`.

## References

- ST product page: https://www.st.com/en/evaluation-tools/stm32h573i-dk.html
- User manual (UM3140)
- STM32H5 reference manual (RM0481)
