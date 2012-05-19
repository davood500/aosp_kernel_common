#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_SENSORS_JZ_MMA8452)	+= jz_sensor_mma8452.o
obj-$(CONFIG_SENSORS_JZ_MMA8452_ORI)	+= jz_sensor_mma8452_ori.o
obj-$(CONFIG_SENSORS_MMA7455L)	+= mma7455l.o
obj-$(CONFIG_SENSORS_LIS33DE_35DE)	+= lis33de_35de.o
obj-$(CONFIG_SENSORS_LIS33DE_35DE_ORI) += lis33de_35de_ori.o
obj-$(CONFIG_SENSORS_LSM303DLH_ACC) += lsm303dlh_acc.o 
obj-$(CONFIG_SENSORS_LSM303DLH_ORI) += lsm303dlh_ori.o
obj-$(CONFIG_SENSORS_LSM303DLH_MAG) += lsm303dlh_mag.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

