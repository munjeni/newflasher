ARMCC=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-gcc
ARMSTRIP=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-strip

ARMCC64=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc
ARMSTRIP64=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-strip

CCWIN=i686-w64-mingw32-gcc
CCWINSTRIP=i686-w64-mingw32-strip
WINDRES=i686-w64-mingw32-windres

CCAPPLE64=/home/savan/Desktop/osxtoolchain/osxcross/target/bin/x86_64-apple-darwin11-cc
CCAPPLESTRIP64=/home/savan/Desktop/osxtoolchain/osxcross/target/bin/x86_64-apple-darwin11-strip

CCAPPLE=/home/savan/Desktop/osxtoolchain/osxcross/target/bin/i386-apple-darwin11-cc
CCAPPLESTRIP=/home/savan/Desktop/osxtoolchain/osxcross/target/bin/i386-apple-darwin11-strip

CC=gcc
STRIP=strip

CFLAGS=-Wall -O2
CROSS_CFLAGS=${CFLAGS} -I include -L lib

.PHONY: default
default: newflasher

.PHONY: cross
cross: newflasher.exe newflasher.x64 newflasher.i386 newflasher.arm32 newflasher.arm64 newflasher.x86_64-apple-darwin11 #newflasher.i386-apple-darwin11

newflasher: newflasher.c version.h
	${CC} ${CFLAGS} $< -o $@ -lz -lexpat

newflasher.exe: newflasher.c version.h newflasher.rc.in
	sed "s/@VERSION@/$$(sed 's/^.*VERSION //' version.h)/" newflasher.rc.in >newflasher.rc
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
	${ARMSTRIP64} newflasher.arm64

newflasher.i386-apple-darwin11: newflasher.c version.h
	${CCAPPLE} ${CROSS_CFLAGS} newflasher.c -o newflasher.i386-apple-darwin11 -lzdarwin11 -lexpatdarwin11
	${CCAPPLESTRIP} newflasher.i386-apple-darwin11

newflasher.x86_64-apple-darwin11: newflasher.c version.h
	${CCAPPLE64} ${CROSS_CFLAGS} newflasher.c -o newflasher.x86_64-apple-darwin11 -lzdarwin11_64 -lexpatdarwin11_64 -lusb-1.0 -lobjc -Wl,-framework,IOKit -Wl,-framework,CoreFoundation
	${CCAPPLESTRIP64} newflasher.x86_64-apple-darwin11

.PHONY: clean
clean:
	rm -rf *.o *.rc *.res

.PHONY: distclean
distclean:
	rm -rf *.o *.rc *.res newflasher.exe newflasher.x64 newflasher.i386 newflasher.arm32 newflasher.arm64 newflasher.i386-apple-darwin11 newflasher.x86_64-apple-darwin11 newflasher
