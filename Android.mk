LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE    := libz
LOCAL_SRC_FILES := zlib/adler32.c \
		zlib/compress.c \
		zlib/crc32.c \
		zlib/deflate.c \
		zlib/gzclose.c \
		zlib/gzlib.c \
		zlib/gzread.c \
		zlib/gzwrite.c \
		zlib/infback.c \
		zlib/inffast.c \
		zlib/inflate.c \
		zlib/inftrees.c \
		zlib/trees.c \
		zlib/uncompr.c \
		zlib/zutil.c

LOCAL_CFLAGS += -Izlib \
		-DHAVE_HIDDEN \
		-DZLIB_CONST \
		-Wall \
		-Werror \
		-Wno-unused \
		-Wno-unused-parameter

include $(BUILD_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE    := libexpat
LOCAL_SRC_FILES := expat/lib/xmlparse.c \
		expat/lib/xmlrole.c \
		expat/lib/xmltok.c

LOCAL_CFLAGS += -Iexpat \
		-Iexpat/lib \
		-Wall \
		-Werror \
		-Wmissing-prototypes \
		-Wstrict-prototypes \
		-Wno-unused-function \
		-Wno-unused-parameter \
		-Wno-missing-field-initializers \
		-DHAVE_EXPAT_CONFIG_H \
		-UWIN32_LEAN_AND_MEAN

include $(BUILD_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := newflasher.arm64_pie
LOCAL_CFLAGS += -Iexpat/lib -Izlib
LOCAL_STATIC_LIBRARIES += libz libexpat
LOCAL_SRC_FILES := newflasher.c

include $(BUILD_EXECUTABLE)
