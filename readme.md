This *experimental* software allows you to flash firmwares acquired through [XperiFirm](https://forum.xda-developers.com/crossdevice-dev/sony/pc-xperifirm-xperia-firmware-downloader-t2834142) to Sony phones including and newer than the XZ  Premium. 

### Usage

- Place the newflasher executable into the firmware folder (usually created by XperiFirm)
- Run Newflasher. Everything inside the folder will be flashed, so remember to remove files you don't want on your device beforehand. 

### Build (native)

Install `gcc`, `make` and development versions of `zlib` and `expat`. On Debian-based systems such as Mint and Ubuntu, this means:

    sudo apt-get install gcc libexpat-dev libz-dev make

On Darwin you need to install `libusb-1.0`, this means:

    brew install libusb

Download, build and install, assuming that `~/bin` is in `$PATH`:

    git clone https://github.com/munjeni/newflasher.git ~/.local/share/newflasher
    make -C ~/.local/share/newflasher
    ln -s ~/.local/share/newflasher/newflasher ~/bin

### Build (cross, static)

`make newflasher.[exe/x64/i386/arm32/arm64/arm64_pie]`, exe referring to a Windows build, the rest being for Linux Android and Darwin.

### Discussion

[XDA Thread](https://forum.xda-developers.com/crossdevice-dev/sony/progress-newflasher-xperia-command-line-t3619426). 
