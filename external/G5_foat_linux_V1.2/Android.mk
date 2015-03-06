LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := eng
LOCAL_MODULE := g510_upgrade
LOCAL_SRC_FILES := $(call all-subdir-c-files)
LOCAL_SHARED_LIBRARIES := \
	libcutils \
	libutils
include $(BUILD_EXECUTABLE)

###############################################################################
include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := eng
LOCAL_MODULE := G510_V0D.00.3C_T4_app.lod
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)
LOCAL_SRC_FILES := $(LOCAL_MODULE)
include $(BUILD_PREBUILT)
###############################################################################
