@defgroup    boards_nucleo-h755zi-q STM32 Nucleo-H755ZI-Q
@ingroup     boards_common_nucleo144
@brief       Support for the STM32 Nucleo-H755ZI-Q

## Overview

The Nucleo-H755ZI-Q is a board from ST's Nucleo family supporting the dual-core
STM32H755ZI microcontroller (ARM Cortex-M7 + ARM Cortex-M4) with 1 MiB of RAM
and 2 MiB of Flash. RIOT runs on the Cortex-M7 core; the Cortex-M4 stays idle.

## Pinout

The pinout follows the standard Nucleo-144 layout (MB1364, identical to
Nucleo-H753ZI) so the [STM user manual UM2407](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf) applies.

### MCU

| MCU          | STM32H755ZI-Q |
|:-------------|:--------------|
| Family       | ARM Cortex-M7 (480 MHz) + Cortex-M4 (240 MHz) |
| Vendor       | ST Microelectronics |
| RAM          | 1 MiB       |
| Flash        | 2 MiB       |
| FPU          | yes (M7 double precision, M4 single precision) |
| Ethernet     | 10/100 Mbps (driver port pending) |
| FDCAN        | 2 channels  |
| Timers       | 22 (2x watchdog, 1 SysTick, 2x 32-bit, 17x 16-bit) |
| ADCs         | 3x 16 bit (up to 36 channels) |
| Crypto       | AES, HASH, RNG |
| Datasheet    | [Datasheet](https://www.st.com/resource/en/datasheet/stm32h755zi.pdf) |
| Reference Manual | [RM0399](https://www.st.com/resource/en/reference_manual/rm0399-stm32h745755-and-stm32h747757-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) |
| Board Manual | [UM2407](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf)|

## Flashing the Board

A detailed description about the flashing process can be found on the
[guides page](https://guide.riot-os.org/board_specific/stm32/).
The board name is `nucleo-h755zi-q`.
