LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := eng
LOCAL_MODULE := gsmmux
LOCAL_SRC_FILES := $(call all-subdir-c-files)
LOCAL_SHARED_LIBRARIES := \
	libcutils \
	libutils
include $(BUILD_EXECUTABLE)
