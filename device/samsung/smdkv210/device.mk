# These is the hardware-specific overlay, which points to the location
# of hardware-specific resource overrides, typically the frameworks and
# application settings that are stored in resourced.
DEVICE_PACKAGE_OVERLAYS := device/samsung/smdkv210/overlay

PRODUCT_COMMON_DIR := device/samsung/common/s5p

PRODUCT_COPY_FILES := \
	device/samsung/smdkv210/init.smdkv210.rc:root/init.smdkv210.rc \
	device/samsung/smdkv210/init.smdkv210.usb.rc:root/init.smdkv210.usb.rc \
	device/samsung/smdkv210/ueventd.smdkv210.rc:root/ueventd.smdkv210.rc \
	device/samsung/smdkv210/s3c-keypad.kl:system/usr/keylayout/s3c-keypad.kl \
        device/samsung/smdkv210/s3c-keypad.kcm:system/usr/keychars/s3c-keypad.kcm \
        device/samsung/smdkv210/s3c_ts.idc:system/usr/idc/s3c_ts.idc \
        device/samsung/smdkv210/ft5x0x_ts.idc:system/usr/idc/ft5x0x_ts.idc \
	device/samsung/smdkv210/vold.fstab:system/etc/vold.fstab

PRODUCT_PROPERTY_OVERRIDES += \
	ro.sf.lcd_density=240 \
	ro.opengles.version=131072

# Set default USB interface
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += \
        persist.sys.usb.config=mass_storage

PRODUCT_PROPERTY_OVERRIDES += \
        hwui.render_dirty_regions=false

PRODUCT_TAGS += dalvik.gc.type-precise

PRODUCT_PACKAGES += \
	gralloc.smdkv210 

#audio
PRODUCT_PACKAGES += \
        audio_policy.smdkv210 \
        audio.primary.smdkv210 \
        audio.a2dp.default \
        lights.smdkv210 \
        hwcomposer.smdkv210 \
        libaudioutils

# GPS
PRODUCT_COPY_FILES += \
        device/samsung/smdkv210/gps.conf:system/etc/gps.conf

# These is the OpenMAX IL configuration files
PRODUCT_COPY_FILES += \
	$(PRODUCT_COMMON_DIR)/sec_mm/sec_omx/sec_omx_core/secomxregistry:system/etc/secomxregistry \
	$(PRODUCT_COMMON_DIR)/media_profiles.xml:system/etc/media_profiles.xml

#MFC Firmware
PRODUCT_COPY_FILES += \
        $(PRODUCT_COMMON_DIR)/samsung_mfc_fw.bin:system/vendor/firmware/samsung_mfc_fw.bin

#bin
PRODUCT_COPY_FILES += \
	device/samsung/smdkv210/usih4_arm:system/bin/usih4_arm \
	device/samsung/smdkv210/BCM4330.hcd:/system/etc/bluetooth/BCM4330.hcd

# These are the OpenMAX IL modules
PRODUCT_PACKAGES += \
        libSEC_OMX_Core \
        libOMX.SEC.AVC.Decoder \
        libOMX.SEC.M4V.Decoder \
        libOMX.SEC.M4V.Encoder \
        libOMX.SEC.AVC.Encoder

# Include libstagefright module
PRODUCT_PACKAGES += \
	libstagefrighthw
# Camera
PRODUCT_PACKAGES += \
	camera.smdkv210

# Filesystem management tools
PRODUCT_PACKAGES += \
	make_ext4fs \
	setup_fs

$(call inherit-product, frameworks/base/build/phone-xhdpi-1024-dalvik-heap.mk)
$(call inherit-product-if-exists, vendor/samsung/smdkv210/device-vendor.mk)
$(call inherit-product, hardware/broadcom/wlan/bcmdhd/firmware/bcmdhd/device-bcm.mk)
