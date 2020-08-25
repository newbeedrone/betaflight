F411_TARGETS += $(TARGET)
FEATURES     = VCP ONBOARDFLASH

TARGET_SRC   = \
             drivers/accgyro/accgyro_mpu.c \
             drivers/accgyro/accgyro_mpu6500.c \
             drivers/accgyro/accgyro_spi_mpu6000.c \
             drivers/accgyro/accgyro_spi_mpu6500.c \
             drivers/max7456.c \
             drivers/vtx_rtc6705.c \
             drivers/vtx_rtc6705_soft_spi.c \
			 drivers/flash_m25p16.c
