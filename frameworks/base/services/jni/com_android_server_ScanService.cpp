#define LOG_TAG "ScanServiceJNI"

#include "jni.h"
#include "JNIHelp.h"
#include "android_runtime/AndroidRuntime.h"

#include <utils/misc.h>
#include <utils/Log.h>
#include <hardware/hardware.h>
#include <hardware/scan.h>

#include <stdio.h>
#include <stdlib.h>

namespace android
{
	static jstring scan_getBarcode(JNIEnv* env, jobject clazz, jint ptr) {
		scan_device_t* device = (scan_device_t*)ptr;
		struct barcode_t barcode;
		int ret = -1;
		char code_c[MAX_RX_BUF_LEN];
		jstring code_jni = NULL;

		if(!device) {
			LOGE("Device scan is not open.");
			return env->NewStringUTF("scan not init");
		}

		ret = device->get_barcode(device, &barcode);
		if(ret){
			LOGE("scan fail.ret=%d", ret);
			sprintf(code_c, "scan fail.ret=%d", ret);
			code_jni = env->NewStringUTF(code_c);
			return code_jni;
		}
		strcpy(code_c, (const char*)barcode.code);
		code_jni = env->NewStringUTF(code_c);
		LOGI("Get code_c %s from device scan.ret=%d", code_c, ret);
	
		return code_jni;
	}

	static inline int scan_device_open(const hw_module_t* module, struct scan_device_t** device) {
		return module->methods->open(module, SCAN_HARDWARE_DEVICE_ID, (struct hw_device_t**)device);
	}
	
	static jint scan_init(JNIEnv* env, jclass clazz) {
		scan_module_t* module;
		scan_device_t* device;
		
		LOGI("Initializing HAL stub scan......");

		if(hw_get_module(SCAN_HARDWARE_MODULE_ID, (const struct hw_module_t**)&module) == 0) {
			LOGI("Device scan found.");
			if(scan_device_open(&(module->common), &device) == 0) {
				LOGI("Device scan is open.");
				return (jint)device;
			}

			LOGE("Failed to open device scan.");
			return 0;
		}

		LOGE("Failed to get HAL stub scan.");

		return 0;		
	}

	static const JNINativeMethod method_table[] = {
		{"init_native", "()I", (void*)scan_init},
		{"getBarcode_native", "(I)Ljava/lang/String;", (void*)scan_getBarcode},
	};

	int register_android_server_ScanService(JNIEnv *env) {
    		return jniRegisterNativeMethods(env, "com/android/server/ScanService", method_table, NELEM(method_table));
	}
};

