ifeq ($(findstring /xtensa-esp32-elf,$(PATH)),)
$(error Please add /path/to/xtensa-esp32-elf to your $$PATH)
endif
ifeq ($(IDF_PATH),)
$(error Please add IDF_PATH=/path/to/esp-idf to your environment)
endif

EXTRA_COMPONENT_DIRS := ../../

PROJECT_NAME := leddisplay_test
include $(IDF_PATH)/make/project.mk

.PHONY: distclean
distclean:
	@rm -rf build sdkconfig sdkconfig.old
