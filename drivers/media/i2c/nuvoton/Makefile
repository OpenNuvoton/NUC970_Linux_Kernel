
obj-y += nuc970_cap.o
ifeq ($(CONFIG_OF),y)
obj-y += sensor_ov7725.o
obj-y += sensor_ov5640.o
obj-y += sensor_nt99141.o
obj-y += sensor_nt99050.o
obj-y += sensor_tw9912.o
else
obj-$(CONFIG_SENSOR_OV7725) += sensor_ov7725.o
obj-$(CONFIG_SENSOR_OV5640) += sensor_ov5640.o
obj-$(CONFIG_SENSOR_NT99141) += sensor_nt99141.o
obj-$(CONFIG_SENSOR_NT99050) += sensor_nt99050.o
obj-$(CONFIG_SENSOR_TW9912) += sensor_tw9912.o
endif

