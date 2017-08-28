#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y				:= irq.o clk-ccf.o clk-apll.o clk-upll.o dev.o cpu.o mfp.o mach-nuc970.o pm.o suspend.o
obj-$(CONFIG_NUC970_CLK_TIMER)	+= time.o
obj-$(CONFIG_NUC970_CLK_ETIMER)	+= etime.o
obj-$(CONFIG_NUC970_FIQ)	+= fiq.o
obj-$(CONFIG_OF)		+= dt-nuc972-evb.o

