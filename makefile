ARMCC=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-gcc
ARMSTRIP=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-strip

ARMCC64=/home/savan/Desktop/qemu/busybox-1.23.2/busybox-1.23.2/busybox-w32/buildroot-2017.02.2/output/host/usr/bin/aarch64-buildroot-linux-uclibc-gcc
ARMSTRIP64=/home/savan/Desktop/qemu/busybox-1.23.2/busybox-1.23.2/busybox-w32/buildroot-2017.02.2/output/host/usr/bin/aarch64-buildroot-linux-uclibc-strip

CCWIN=i586-mingw32msvc-gcc
CCWINSTRIP=i586-mingw32msvc-strip
WINDRES=i586-mingw32msvc-windres

CC=gcc
STRIP=strip

CFLAGS=-Wall -O3 -static -I include -I expat/include -L lib -L expat/lib

default:newflasher.exe newflasher.x64 newflasher.i386 newflasher.arm32 newflasher.arm64

newflasher.exe:
	${WINDRES} newflasher.rc -O coff -o newflasher.res
	${CCWIN} ${CFLAGS} sha256.c newflasher.c newflasher.res -o newflasher.exe -lsetupapi -lzwin -lexpat.win
	${CCWINSTRIP} newflasher.exe

newflasher.x64:
	${CC} ${CFLAGS} sha256.c newflasher.c -o newflasher.x64 -lz64 -lexpat.x64
	${STRIP} newflasher.x64

newflasher.i386:
	${CC} ${CFLAGS} -m32 sha256.c newflasher.c -o newflasher.i386 -lz32 -lexpat.i386
	${STRIP} newflasher.i386

newflasher.arm32:
	${ARMCC} ${CFLAGS} sha256.c newflasher.c -o newflasher.arm32 -lzarm32 -lexpat.arm32
	${ARMSTRIP} newflasher.arm32

newflasher.arm64:
	${ARMCC64} ${CFLAGS} sha256.c newflasher.c -o newflasher.arm64 -lzarm64 -lexpat.arm64
	${ARMSTRIP64} newflasher.arm64

clean:
	rm -rf *.o *.res

distclean:
	rm -rf *.o *.res newflasher.exe newflasher.x64 newflasher.i386 newflasher.arm32 newflasher.arm64
