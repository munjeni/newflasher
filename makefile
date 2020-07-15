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
CROSS_CFLAGS=${CFLAGS} -I include -I zlib-1.2.11 -L zlib-1.2.11 -I expat-2.2.9/lib -L expat-2.2.9/lib/.libs

.PHONY: default
default: newflasher

.PHONY: cross
cross: newflasher.exe newflasher.x64 newflasher.i386 newflasher.arm32 newflasher.arm64 newflasher.arm64_pie

.PHONY: libs
libs:
	@mkdir -p include
	@test -d zlib-1.2.11 && echo "" || wget https://zlib.net/zlib-1.2.11.tar.gz
	@test -d zlib-1.2.11 && echo "" || tar xzf zlib-1.2.11.tar.gz
	@rm -rf zlib-1.2.11.tar.gz
	@test -d expat-2.2.9 && echo "" || wget https://github.com/libexpat/libexpat/releases/download/R_2_2_9/expat-2.2.9.tar.gz
	@test -d expat-2.2.9 && echo "" || tar xzf expat-2.2.9.tar.gz
	@rm -rf expat-2.2.9.tar.gz

newflasher: newflasher.c version.h
	${CC} ${CFLAGS} $< -o $@ -lz -lexpat ${LIBS}

newflasher.exe: libs newflasher.c version.h
	@cd zlib-1.2.11 && CC=${CCWIN} ./configure --static && make clean && make
	@cd expat-2.2.9 && CC="${CCWIN} -fPIC" ./configure --enable-static --disable-shared --host=i686-w64-mingw32 && make clean && make
	@test -f include/GordonGate.h && echo "" || wget https://github.com/Androxyde/Flashtool/blob/master/drivers/GordonGate/Sony_Mobile_Software_Update_Drivers_x64_Setup.msi?raw=true -O GordonGate
	@test -f include/GordonGate.h && echo "" || xxd --include GordonGate > include/GordonGate.h
	@rm -rf GordonGate
	sed "s/@VERSION@/$(VERSION)/" newflasher.rc.in >newflasher.rc
	${WINDRES} newflasher.rc -O coff -o newflasher.res
	${CCWIN} ${CROSS_CFLAGS} -static newflasher.c newflasher.res -o newflasher.exe -lsetupapi -lz -lexpat
	${CCWINSTRIP} newflasher.exe

newflasher.x64: libs newflasher.c version.h
	@cd zlib-1.2.11 && CC=gcc ./configure --static && make clean && make
	@cd expat-2.2.9 && CC="gcc -fPIC" ./configure --enable-static --disable-shared && make clean && make
	${CC} ${CROSS_CFLAGS} -static newflasher.c -o newflasher.x64 -lz -lexpat
	${STRIP} newflasher.x64

newflasher.i386: libs newflasher.c version.h
	@cd zlib-1.2.11 && CC="gcc -m32" ./configure --static && make clean && make
	@cd expat-2.2.9 && CC="gcc -m32 -fPIC" ./configure --enable-static --disable-shared && make clean && make
	${CC} ${CROSS_CFLAGS} -m32 -static newflasher.c -o newflasher.i386 -lz -lexpat
	${STRIP} newflasher.i386

newflasher.arm32: libs newflasher.c version.h
	@cd zlib-1.2.11 && CC=${ARMCC} ./configure --static && make clean && make
	@cd expat-2.2.9 && CC="${ARMCC} -fPIC" ./configure --enable-static --disable-shared --host=arm-linux-gnueabi && make clean && make
	${ARMCC} ${CROSS_CFLAGS} -static newflasher.c -o newflasher.arm32 -lz -lexpat
	${ARMSTRIP} newflasher.arm32

newflasher.arm64: libs newflasher.c version.h
	@cd zlib-1.2.11 && CC=${ARMCC64} ./configure --static && make clean && make
	@cd expat-2.2.9 && CC="${ARMCC64} -fPIC" ./configure --enable-static --disable-shared --host=aarch64-linux-gnu && make clean && make
	${ARMCC64} ${CROSS_CFLAGS} -static newflasher.c -o newflasher.arm64 -lz -lexpat
	${ARMSTRIP64} newflasher.arm64

newflasher.arm64_pie:
	@echo "Building Android pie binary"
	@test -d zlib && echo "" || git clone https://android.googlesource.com/platform/external/zlib
	@test -d expat && echo "" || git clone https://android.googlesource.com/platform/external/expat
	${NDK_BUILD}
	@cp -fr libs/arm64-v8a/newflasher.arm64_pie ./newflasher.arm64_pie

newflasher.1.gz: newflasher.1
	gzip -9fkn $<

.PHONY: install
install: newflasher newflasher.1.gz
	$(INSTALL) -o root -g root -d $(DESTDIR)/usr/bin
	$(INSTALL) -o root -g root -m 755 -s newflasher $(DESTDIR)/usr/bin/
	$(INSTALL) -o root -g root -d $(DESTDIR)/usr/share/man/man1
	$(INSTALL) -o root -g root -m 644 -s newflasher.1.gz $(DESTDIR)/usr/share/man/man1

.PHONY: clean
clean:
	rm -rf *.gz *.o *.rc *.res *.ico obj libs zlib expat newflasher-*.txz zlib-1.2.11 expat-2.2.9 include *.mk *.in

.PHONY: distclean
distclean:
	rm -rf *.gz *.o *.rc *.res *.ico obj libs zlib expat newflasher.exe newflasher.x64 newflasher.i386
	rm -rf newflasher.arm32 newflasher.arm64 newflasher.arm64_pie newflasher newflasher-*.txz
	rm -rf zlib-1.2.11 expat-2.2.9 include *.mk *.in
