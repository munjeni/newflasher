ARMCC=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-gcc
ARMSTRIP=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-strip

ARMCC64=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc
ARMSTRIP64=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-strip

CCWIN=i686-w64-mingw32-gcc
CCWINSTRIP=i686-w64-mingw32-strip
WINDRES=i686-w64-mingw32-windres

CC=gcc
STRIP=strip

CFLAGS=-Wall -O2
CROSS_CFLAGS=${CFLAGS} -static -I include -I expat/include -L /usr/local/lib -L /usr/lib -L lib -L expat/lib

default: newflasher

cross: newflasher.exe newflasher.x64 newflasher.i386 newflasher.arm32 newflasher.arm64

newflasher: newflasher.c GordonGate.h
	${CC} ${CFLAGS} $< -o $@ -lz -lexpat

newflasher.exe:
	${WINDRES} newflasher.rc -O coff -o newflasher.res
	${CCWIN} ${CROSS_CFLAGS} newflasher.c newflasher.res -o newflasher.exe -lsetupapi -lzwin -lexpat.win
	${CCWINSTRIP} newflasher.exe

newflasher.x64:
	${CC} ${CROSS_CFLAGS} newflasher.c -o newflasher.x64 -lz64 -lexpat.x64
	${STRIP} newflasher.x64

newflasher.i386:
	${CC} ${CROSS_CFLAGS} -m32 newflasher.c -o newflasher.i386 -lz32 -lexpat.i386
	${STRIP} newflasher.i386

newflasher.arm32:
	${ARMCC} ${CROSS_CFLAGS} newflasher.c -o newflasher.arm32 -lzarm32 -lexpat.arm32
	${ARMSTRIP} newflasher.arm32

newflasher.arm64:
	${ARMCC64} ${CROSS_CFLAGS} newflasher.c -o newflasher.arm64 -lzarm64 -lexpat.arm64
	${ARMSTRIP64} newflasher.arm64

clean:
	rm -rf *.o *.res

distclean:
	rm -rf *.o *.res newflasher.exe newflasher.x64 newflasher.i386 newflasher.arm32 newflasher.arm64
