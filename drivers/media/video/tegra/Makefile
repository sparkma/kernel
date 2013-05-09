GCOV_PROFILE := y
#
# Makefile for the video capture/playback device drivers.
#
obj-y				+= avp/
obj-$(CONFIG_TEGRA_MEDIASERVER)	+= mediaserver/
obj-$(CONFIG_TEGRA_NVAVP)	+= nvavp/
obj-$(CONFIG_TEGRA_DTV)		+= tegra_dtv.o
obj-$(CONFIG_TEGRA_CAMERA)	+= tegra_camera.o
obj-$(CONFIG_VIDEO_AR0832)	+= ar0832_main.o
obj-$(CONFIG_VIDEO_OV5650)	+= ov5650.o
obj-$(CONFIG_VIDEO_OV14810)	+= ov14810.o
obj-$(CONFIG_VIDEO_OV9726)	+= ov9726.o
obj-$(CONFIG_VIDEO_OV2710)	+= ov2710.o
obj-$(CONFIG_VIDEO_SOC380)	+= soc380.o
obj-$(CONFIG_TORCH_SSL3250A)	+= ssl3250a.o
obj-$(CONFIG_TORCH_TPS61050)	+= tps61050.o
obj-$(CONFIG_VIDEO_SH532U)	+= sh532u.o
obj-$(CONFIG_VIDEO_AD5820)	+= ad5820.o
# /* [ECID:0000]ZTEBSP: yuxin add for new camera 20110921 begin */
obj-$(CONFIG_VIDEO_OV5640)	+= ov5640.o
# /* [ECID:0000]ZTEBSP: yuxin add for new camera 20110921 end */
#[ECID:0000]ZTEBSP yuxin add for cam sensor ov7692 ++,2011.11.15
obj-$(CONFIG_VIDEO_OV7692)	+= ov7692.o
#[ECID:0000]ZTEBSP yuxin add for cam sensor ov7692 --,2011.11.15
#[ECID:0000]ZTEBSP yuxin add for flash chip zi2848 ++,2011.10.25
obj-$(CONFIG_FLASH_ZI2848)	+= zi2848.o
#[ECID:0000]ZTEBSP yuxin add for flash chip zi2848 --,2011.10.25
