hardware_modules := gralloc hwcomposer audio nfc scan
include $(call all-named-subdir-makefiles,$(hardware_modules))
