#
#  Makefile for the NUC970 CAN controller drivers.
#

obj-$(CONFIG_NUC970_CAN) += nuc970_can.o
obj-$(CONFIG_NUC970_CAN0) += nuc970_can0_platform.o
obj-$(CONFIG_NUC970_CAN1) += nuc970_can1_platform.o

ccflags-$(CONFIG_CAN_DEBUG_DEVICES) := -DDEBUG
