Instructions for building RepRapFirmware
========================================

**Important!**

RepRapFirmware is built from several Github projects. You need to use
compatible branches of these projects. Since RepRapFirmware v3.1.0, stable releases are tagged in github.

To build RepRapFirmware v3.1.1, use the following tags:

| project | tag | notes | 
| :-- | --: | --- |
|[RepRapFirmware](https://github.com/Duet3D/RepRapFirmware)| 3.1.1||
|[CANlib](https://github.com/Duet3D/CANlib)| 3.1.0 | _only needed for Duet 3_ |
|[CoreNG](https://github.com/Duet3D/CoreNG)| 3.1.0||
|[FreeRTOS](https://github.com/Duet3D/FreeRTOS)| 3.1.0||
|[RRFLibraries](https://github.com/Duet3D/RRFLibraries)| 3.1.0||
| [DuetWiFiSocketServer](https://github.com/Duet3D/DuetWifiSocketServer)|| _not tagged, use master branch_|

 >**Note**: CoreNG, FreeRTOS, RRFLibraries do not have v3.1.1 tags because the  3.1.0 tagged releases are the base for v3.1.1 (no changes in the dependencies, only in RepRapFirmware) 

To build RepRapFirmware from the latest development sources, use the following branches:

| project | branch | notes | 
| :-- | --: | --- |
|[RepRapFirmware](https://github.com/Duet3D/RepRapFirmware)| v3.02-dev||
|[CANlib](https://github.com/Duet3D/CANlib)| master | _only needed for Duet 3_ |
|[CoreNG](https://github.com/Duet3D/CoreNG)| dev | _not needed for Duet 3 Mini5+_ |
|[CoreN2G](https://github.com/Duet3D/CoreN2G)| master | _only needed for Duet 3 Mini5+_ |
|[FreeRTOS](https://github.com/Duet3D/FreeRTOS)| dev ||
|[RRFLibraries](https://github.com/Duet3D/RRFLibraries)| dev ||
|[DuetWiFiSocketServer](https://github.com/Duet3D/DuetWifiSocketServer)| dev ||

## Additional Tools

Building RepRapFirmware lately requires a tool called `crc32appender` to be in
the user's `PATH` as it will be called at the end of the compilating process.  It
can be found as Golang source code in `Tools/crc32appender` together with
pre-compiled binaries for Windows, Linux and MacOS x86-64.

## Instructions for building under Windows

1. Download and install the gcc cross-compiler from
   [the ARM developer site](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads): 
   - To build firmware version 2.03beta3 and later use version 2018-q4-major
   - To build firmware version 2.01beta2 and later, use version 2018-q2-update
   - To build firmware version 1.20alpha3 and later, use version 2017-q2-update
   - To build firmware version 1.20alpha2 and earlier, use version
     arm-none-eabi-4.8.3-2014q1. A simple way of doing this is to download
     Arduino version 1.5.8 and install it into folder `C:/Arduino-1.5.8`. The
     compiler and associated tools will then be in folder
     `C:\Arduino-1.5.8\hardware\tools\gcc-arm-none-eabi-4.8.3-2014q1\bin`.
     If you already have a later version of Arduino installed including
     the add-on for SAM processors, you will find the compiler and tools in a
     different folder, for example
     `C:\Users\<YOUR USER NAME>\AppData\Local\Arduino15\packages\arduino\tools\arm-none-eabi-gcc\4.8.3-2014q1\bin.`

2. Download and install [Eclipse IDE for C/C++ Developers version 2018-09](http://www.eclipse.org/downloads/eclipse-packages/).
   You do not need the Arduino add-on.

3. Download and install [GNU Arm Eclipse](https://sourceforge.net/projects/gnuarmeclipse/files/Build%20Tools/gnuarmeclipse-build-tools-win64-2.6-201507152002-setup.exe/download).
   This provides versions of make.exe, rm.exe and other tools without the
   8192-character command line limitation of some other versions.

4. Modify your PATH environment variable to include the `bin` folder of the GNU
   ARM Eclipse installation.

5. Run `which rm` and `which make` to make sure that rm and make will be
   fetched from that folder.

6. In Eclipse create new workspace `C:/Eclipse/Firmware`. Then exit Eclipse.

7. Download this github project as a zip file and unzip it into
   `C:/Eclipse/Firmware`. Then rename folder `ReprapFirmware-dev` in that folder
   to `RepRapFirmware`. Alternately, change into the directory
   `C:/Eclipse/Firmware` in a Terminal and run `git clone
   https://github.com/Duet3D/ReprapFirmware.git -branch dev`

8. Repeat the previous step for github project CoreNG. The folder name should
   be left as CoreNG (or renamed from CoreNG-dev to CoreNG if you downloaded a
   dev build).

9. If you want to build version 1.19 or later of the Duet WiFi build of
   RepRapFirmware then you also need to download and add project
   [DuetWiFiSocketServer](https://github.com/Duet3D/DuetWifiSocketServer). Alternatively, just download file
   [`src/include/MessageFormats.h`](https://github.com/Duet3D/DuetWiFiSocketServer/blob/master/src/include/MessageFormats.h) 
   from that project and put it somewhere on the
   include path for RepRapFirmware.

10. If you want to build a RTOS-enabled configuration of the v2-dev branch,
    also download project FreeRTOS from my github repo and add that project to
    the workspace.

11. If you want to build firmware versions later than 2.02RC1, also download
    and project RRFLibraries from my github repo and add that project to the
    workspace.

12. Load Eclipse and tell it to import the CoreNG and RepRapFirmware projects,
    also CANlib, FreeRTOS, DuetWiFiSocketServer and RRFLibraries if you have included
    them.

13. The build depends on the Eclipse workspace variable **ArmGccPath** being
    set to the directory where your arm-none-eabi-g++ compiler resides. For
    example `C:\Program Files (x86)\GNU Tools ARM Embedded\7 2018-q2-update\bin`
    on Windows. To set it, go to **Windows -> Preferences -> C/C++ -> Build ->
    Build Variables** and click "Add..."

14. Build dependencies:
    - Duet3 Mini builds depend on [CoreN2G](https://github.com/Duet3D/CoreN2G),
      [FreeRTOS](https://github.com/Duet3D/FreeRTOS), and
      [RRFLibraries](https://github.com/Duet3D/RRFLibraries). They also
      require [DuetWifiSocketServer](https://github.com/Duet3D/DuetWifiSocketServer),
      as outlined above.
    - Duet3 builds depend on [CANlib](https://github.com/Duet3D/CANlib), 
      [CoreNG](https://github.com/Duet3D/CoreNG), [FreeRTOS](https://github.com/Duet3D/FreeRTOS),
      and [RRFLibraries](https://github.com/Duet3D/RRFLibraries).
    - Duet2 builds depend on [CoreNG](https://github.com/Duet3D/CoreNG),
      [FreeRTOS](https://github.com/Duet3D/FreeRTOS), and [RRFLibraries](https://github.com/Duet3D/RRFLibraries).
      They also require [DuetWifiSocketServer](https://github.com/Duet3D/DuetWifiSocketServer),
      as outlined above.
    - Duet Maestro builds depend on [CoreNG](https://github.com/Duet3D/CoreNG),
      [FreeRTOS](https://github.com/Duet3D/FreeRTOS), and [RRFLibraries](https://github.com/Duet3D/RRFLibraries).
    - Duet085 builds (also running on Duet06) depend on [CoreNG](https://github.com/Duet3D/CoreNG) and [RRFLibraries](https://github.com/Duet3D/RRFLibraries). **Note:** Duet085 does not run RepRapFirmware 2 or later, the last official build is v1.26.

15. Build configurations  
    | Board | Project | Build Configuation|  
    | :-- | :-- | :-- |
    |**Duet3 Mini** (Wifi and Ethernet)||
    ||CoreN2G| SAME5X_RTOS|
    ||FreeRTOS | SAME51|
    ||RRFLibraries|SAME51_RTOS|
    ||RepRapFirmware|Duet_5LC|
    |**Duet3**||
    ||CANlib|SAME70_RTOS|
    ||CoreNG|SAME70|
    ||FreeRTOS|SAME70|
    ||RRFLibraries|SAME70_RTOS|
    ||RepRapFirmware|Duet3_V06|
    |**Duet2** (DuetWifi and Duet Ethernet)||
    ||CoreNG|SAM4E8E|
    ||FreeRTOS|SAM4E|
    ||RRFLibraries|SAM4E_RTOS|
    ||RepRapFirmware|Duet2_RTOS (or just Duet2 in version 3.2 and later)|
    |**Duet Maestro**||
    ||CoreNG|SAM4S|
    ||FreeRTOS|SAM4S|
    ||RRFLibraries|SAM4S_RTOS|
    ||RepRapFirmware|DuetMaestro|
    |**Duet085** (Duet0.85 and Duet0.6)|**Note:** _Needs v1.20 tags_||
    ||CoreNG|SAM3X8E|
    ||RRFLibraries|SAM3X|
    ||RepRapFirmware|Duet085| 

16. Build order:  
      1. CoreNG
      2. CoreN2G (if needed)
      2. FreeRTOS  
      3. RRFLibraries  
      4. CANlib (if needed)  
      5. Build RepRapFirmware  

  >**Note:** you do not need to build the DuetWiFiSocketServer project, but it does need to be in the workspace because the RepRapFirmware project uses one of its include fies.

## Instructions for building under macOS

Using Homebrew-Cask makes it very easy to install new software on macOS:
https://caskroom.github.io/

1. Download and install the gcc-arm-embedded: brew cask install
   gcc-arm-embedded
2. Download and install Eclipse for C++ : brew cask install eclipse-cpp
3. Download or clone the RepRapFirmware, CoreNG, FreeRTOS, RRFLibraries and
   DuetWiFiSocketServer projects into your workspace. Keep the folder names as
   is.
4. Open Eclipse and import RepRapFirmware, FreeRTOS, RRFLibraries and CoreNG
   projects using File -> Open Projects from File System.
5. The build depends on the Eclipse workspace variable **ArmGccPath** being set
   to the directory where your `arm-none-eabi-g++` compiler resides. To set it,
   go to Windows -> Preferences -> C/C++ -> Build -> Build Variables and
   click "Add..."
6. Build CoreNG, FreeRTOS and RRFLibraries first, then RepRapFirmware. See the
   instructions for Windows (above), step 14ff, for the dependencies and configurations needed.

## Building under Debian Linux

See [this forum post](https://forum.duet3d.com/topic/11703/building-reprap-firmware-on-debian-buster).

## Related projects

- [DuetWebControl](https://github.com/Duet3D/DuetWebControl)
- [unofficial LPC17xx port](https://github.com/gloomyandy/RepRapFirmware)
