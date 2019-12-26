# Heart Rate Monitor
This is a heart rate monitoring application developed for the FRDM KL03 evaluation board. It is built on a stripped-down version of the [Warp firmware](https://github.com/physical-computation/Warp-hardware) developed by the University of Cambridge's [Physical Computation Laboratory](http://physcomp.eng.cam.ac.uk). 

**Prerequisites:** You need an arm cross-compiler such as `arm-none-eabi-gcc` installed as well as a working `cmake` (installed, e.g., via `apt-get` on Linux or via [MacPorts](https://www.macports.org) or [Homebrew](https://brew.sh) on macOS). You will also need an installed copy of the SEGGER [JLink commander](https://www.segger.com/downloads/jlink/), `JlinkExe`, which is available for Linux, macOS, and Windows (here are direct links for downloading it for [macOS](https://www.segger.com/downloads/jlink/JLink_MacOSX.pkg), and [Linux tgz 64-bit](https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.tgz)).

## 1. Compiling the application
The steps here are drawn from the compilation of the [Warp firmware](https://github.com/physical-computation/Warp-hardware).

First, specify the full path to the `bin` directory of your arm cross-compiler in the environment variable `ARMGCC_DIR` by changing line 3 of `build/ksdk1.1/build.sh`:

	export ARMGCC_DIR=<full-path-to-arm-cross-compiler>

Second, specify the full path to the application directory so that JLink can correctly locate the binary output file by changing line 3 of `tools/scripts/jlink.commands`:

	loadfile <full-path-to-application>/build/ksdk1.1/work/demos/Warp/armgcc/Warp/release/Warp.srec

Then you should be able to compile the application with:

	cd build/ksdk1.1/
	./build.sh

This copies the files from `Warp/src/boot/ksdk1.1.0/` into the KSDK tree, builds, and converts the binary to SREC. _When editing source, edit the files in `Warp/src/boot/ksdk1.1.0/`, not the files in the build location, since the latter are overwritten during each build._

## 2. Pin configuration
The following pins on the FRDM KL03 evaluation board are connected to an [SSD1331 OLED display](https://www.adafruit.com/product/684) and a [PPG pulse sensor](https://pulsesensor.com):

PIN CONFIGURATION

## 3. Loading the application onto the evaluation board
Run the firmware downloader:

	JLinkExe -device MKL03Z32XXX4 -if SWD -speed 100000 -CommanderScript ../../tools/scripts/jlink.commands

### Acknowledgements
The Warp firmware is developed by Phillip Stanley-Marbell and the University of Cambridge's Physical Computation Laboratory. This application was developed as a final project for the 4B25 Embedded Systems course at the Cambridge University Engineering Department.