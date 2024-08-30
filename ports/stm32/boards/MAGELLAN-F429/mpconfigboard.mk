MCU_SERIES = f4
CMSIS_MCU = STM32F429xx
AF_FILE = boards/stm32f429_af.csv
LD_FILES = boards/MAGELLAN-F429/stm32f429.ld boards/MAGELLAN-F429/common_ifs.ld
TEXT0_ADDR = 0x08000000
TEXT1_ADDR = 0x08020000

#01studio
MICROPY_PY_PICLIB = 1
MICROPY_PY_MP3	  = 1
MICROPY_PY_VIDEO  = 1
# MicroPython settings
MICROPY_PY_LWIP = 1
MICROPY_PY_USSL = 1
MICROPY_SSL_MBEDTLS = 1