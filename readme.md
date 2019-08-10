This *experimental* software allows you to flash firmwares acquired through [XperiFirm](https://forum.xda-developers.com/crossdevice-dev/sony/pc-xperifirm-xperia-firmware-downloader-t2834142) to Sony phones including and newer than the XZ  Premium. 

### Usage

- Place the newflasher executable into the firmware folder (usually created by XperiFirm)
- Run Newflasher. Everything inside the folder will be flashed, so remember to remove files you don't want on your device beforehand. 

### Build (native)

Download, build and install, assuming that `~/bin` is in `$PATH`:

    git clone https://github.com/newflasher/newflasher.git ~/.local/share/newflasher
    make -C ~/.local/share/newflasher
    ln -s ../.local/share/newflasher/newflasher ~/bin

### Build (cross, static)

`make newflasher.[exe/x64/i386/arm32/arm64]`, exe referring to a Windows build, the rest being for Linux and Android. 

### Discussion

[XDA Thread](https://forum.xda-developers.com/crossdevice-dev/sony/progress-newflasher-xperia-command-line-t3619426). 
