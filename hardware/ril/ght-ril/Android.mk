# Copyright 2006 The Android Open Source Project

# XXX using libutils for simulator build only...
LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
    reference-ril.c \
    atchannel.c \
    misc.c \
    at_tok.c 

LOCAL_SHARED_LIBRARIES := \
    libcutils libutils libril


# for asprinf
LOCAL_CFLAGS := -D_GNU_SOURCE

LOCAL_CFLAGS += -DRIL_SHLIB

LOCAL_C_INCLUDES := $(KERNEL_HEADERS)


LOCAL_PRELINK_MODULE:= false
LOCAL_MODULE_TAGS := eng
#optional
	
LOCAL_LDLIBS += -lpthread

LOCAL_MODULE:= libght-ril


include $(BUILD_SHARED_LIBRARY)

