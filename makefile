ARMCC=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-gcc
ARMSTRIP=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-strip

ARMCC64=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc
ARMSTRIP64=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-strip

NDK_BUILD := NDK_PROJECT_PATH=. /root/ndk/android-ndk-r21d/ndk-build NDK_APPLICATION_MK=./Application.mk

CCWIN=i686-w64-mingw32-gcc
CCWINSTRIP=i686-w64-mingw32-strip
WINDRES=i686-w64-mingw32-windres

OS := $(shell uname)
VERSION := $(shell sed 's/^.*VERSION //' version.h)

CC=gcc
STRIP=strip
INSTALL=install

DESTDIR=
LIBS=

CFLAGS=-Wall -g -O2
ifeq ($(OS),Darwin)
CFLAGS+= -I/usr/local/Cellar/libusb/1.0.23/include/libusb-1.0
LIBS+=-lusb-1.0
endif
CROSS_CFLAGS=${CFLAGS} -I include -L lib

.PHONY: default
default: newflasher

.PHONY: cross
cross: newflasher.exe newflasher.x64 newflasher.i386 newflasher.arm32 newflasher.arm64 newflasher.arm64_pie

newflasher: newflasher.c version.h
	${CC} ${CFLAGS} $< -o $@ -lz -lexpat ${LIBS}

newflasher.exe: newflasher.c version.h newflasher.rc.in
	sed "s/@VERSION@/$(VERSION)/" newflasher.rc.in >newflasher.rc
	${WINDRES} newflasher.rc -O coff -o newflasher.res
	${CCWIN} ${CROSS_CFLAGS} -static newflasher.c newflasher.res -o newflasher.exe -lsetupapi -lzwin -lexpat.win
	${CCWINSTRIP} newflasher.exe

newflasher.x64: newflasher.c version.h
	${CC} ${CROSS_CFLAGS} -static newflasher.c -o newflasher.x64 -lz64 -lexpat.x64
	${STRIP} newflasher.x64

newflasher.i386: newflasher.c version.h
	${CC} ${CROSS_CFLAGS} -m32 -static newflasher.c -o newflasher.i386 -lz32 -lexpat.i386
	${STRIP} newflasher.i386

newflasher.arm32: newflasher.c version.h
	${ARMCC} ${CROSS_CFLAGS} -static newflasher.c -o newflasher.arm32 -lzarm32 -lexpat.arm32
	${ARMSTRIP} newflasher.arm32

newflasher.arm64: newflasher.c version.h
	${ARMCC64} ${CROSS_CFLAGS} -static newflasher.c -o newflasher.arm64 -lzarm64 -lexpat.arm64

newflasher.arm64_pie:
	@echo "Building Android pie binary"
	@git clone https://android.googlesource.com/platform/external/zlib
	@git clone https://android.googlesource.com/platform/external/expat
	${NDK_BUILD}
	@cp -fr libs/arm64-v8a/newflasher.arm64_pie ./newflasher.arm64_pie

newflasher.1.gz: newflasher.1
	gzip -9fkn $<

newflasher-$(VERSION).txz: makefile newflasher.1 newflasher.c readme.md version.h
	rm -rf $(basename $@)
	$(INSTALL) -d $(basename $@)
	tar c $^ | tar xp -C $(basename $@)
	chmod -R u=rwX,go=rX $(basename $@)
	tar c --owner=0 --group=0 --numeric-owner $(basename $@) | xz -9v >$@
	rm -rf $(basename $@)

.PHONY: dist
dist: newflasher-$(VERSION).txz

.PHONY: install
install: newflasher newflasher.1.gz
	$(INSTALL) -o root -g root -d $(DESTDIR)/usr/bin
	$(INSTALL) -o root -g root -m 755 -s newflasher $(DESTDIR)/usr/bin/
	$(INSTALL) -o root -g root -d $(DESTDIR)/usr/share/man/man1
	$(INSTALL) -o root -g root -m 644 -s newflasher.1.gz $(DESTDIR)/usr/share/man/man1

.PHONY: clean
clean:
	rm -rf *.gz *.o *.rc *.res obj libs zlib expat newflasher-*.txz

.PHONY: distclean
distclean:
	rm -rf *.gz *.o *.rc *.res obj libs zlib expat newflasher.exe newflasher.x64 newflasher.i386
	rm -rf newflasher.arm32 newflasher.arm64 newflasher.arm64_pie newflasher newflasher-*.txz
