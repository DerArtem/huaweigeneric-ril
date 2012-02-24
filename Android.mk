# Copyright 2006 The Android Open Source Project

# XXX using libutils for simulator build only...
#
LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
    huaweigeneric-ril.c \
    atchannel.c \
    misc.c \
    at_tok.c \
    sms.c \
    sms_gsm.c \
    gsm.c \
	requestdatahandler.c

LOCAL_SHARED_LIBRARIES := \
	libcutils libutils libril

# for asprinf
LOCAL_CFLAGS := -D_GNU_SOURCE

LOCAL_C_INCLUDES := $(KERNEL_HEADERS) $(TOP)/hardware/ril/libril/

LOCAL_MODULE_TAGS := optional

#build shared library
LOCAL_SHARED_LIBRARIES += \
libcutils libutils
LOCAL_LDLIBS += -lpthread
LOCAL_CFLAGS += -DRIL_SHLIB 
LOCAL_MODULE:= libhuaweigeneric-ril
LOCAL_PRELINK_MODULE := false
include $(BUILD_SHARED_LIBRARY)
