Instructions for building dc42 fork of RepRapFirmware
=====================================================

**Important!**

RepRapFirmware is built from several Github projects. You need to use compatible branches of these projects. As at 27 December 2018, the current (version 2.02) source code is on these branches:

- ReprapFirmware: main
- CoreNG: main
- FreeRTOS: main
- RRFLibraries: main
- DuetWiFiSocketServer: dev

**Instructions for building under Windows**

1. Download and install the gcc cross-compiler:

- To build firmware version 2.01beta2 and later, you need version 2018-q2-update. You can download an installer for this version from https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads.

- To build firmware version 1.20alpha3 and later, you need version 2017-q2-update. You may still be able to download an installer for this version from https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads.

- To build firmware version 1.20alpha2 and earlier you need version arm-none-eabi-4.8.3-2014q1. A simple way of doing this is to download Arduino version 1.5.8 and install it into folder C:/Arduino-1.5.8. The compiler and associated tools will then be in folder C:\Arduino-1.5.8\hardware\tools\gcc-arm-none-eabi-4.8.3-2014q1\bin. If you already have a later version of Arduino installed including the add-on for SAM processors, you will find the compiler and tools in a different folder, for example C:\Users\<YOUR USER NAME>\AppData\Local\Arduino15\packages\arduino\tools\arm-none-eabi-gcc\4.8.3-2014q1\bin.

2. Download and install Eclipse IDE for C/C++ Developers version 2018-09, from http://www.eclipse.org/downloads/eclipse-packages/. You do not need the Arduino add-on.

3. Download and install GNU Arm Eclipse from https://sourceforge.net/projects/gnuarmeclipse/files/Build%20Tools/gnuarmeclipse-build-tools-win64-2.6-201507152002-setup.exe/download. This provides versions of make.exe, rm.exe and other tools without the 8192-character command line limitation of some other versions.

4. Modify your PATH environment variable to include the 'bin' folder of the GNU ARM Eclipse installation.

5. Run "which rm" and "which make" to make sure that rm and make will be fetched from that folder.

6. In Eclipse create new workspace C:/Eclipse/Firmware. Then exit Eclipse.

7. Download this github project as a zip file and unzip it into C:/Eclipse/Firmware. Then rename folder ReprapFirmware-dev in that folder to RepRapFirmware.

8. Repeat the previous step for github project CoreNG. The folder name should be left as CoreNG (or renamed from CoreNG-dev to CoreNG if you downloaded a dev build).

9. If you want to build version 1.19 or later of the Duet WiFi build of RepRapFirmware then you also need to download and add project DuetWiFiSocketServer. Alternatively, just download file src/include/MessageFormats.h from that project and put it somewhere on the include path for RepRapFirmware.

10. If you want to build a RTOS-enabled configuration of the v2-dev branch, also download project FreeRTOS from my github repo and add that project to the workspace.

11. If you want to build firmware versions later than 2.02RC1, also download and project RRFLibraries from my github repo and add that project to the workspace.

12. Load Eclipse and tell it to import the CoreNG and RepRapFirmware projects (and DuetWiFiSocketServer and RRFLibraries if you have included them).

13. If your compiler and tools are in a folder other than C:\Program Files (x86)\GNU Tools ARM Embedded\7 2018-q2-update\bin, configure the path to the tools in both projects by changing the GccPath build variable.

14. Build CoreNG first, also build FreeRTOS and RRFLibraries if needed. Then clean and build RepRapFirmware (the clean step is needed to make Eclipse notice that the output library files in the other projects have been built). The Duet WiFi and Duet Ethernet builds of RRF use the SAM4E_RTOS builds of CoreNG and RRFLibraries and the SAM4E build of FreeRTOS. The Duet Maestro uses the SAM4S_RTOS build of CoreNG and RRFLibraries, and the SAM4S build of FreeRTOS. The Duet085 build of RRF (which also runs on the Duet06) uses the SAM3X build of CoreNG and RRFLibraries. The RADDS build of RRF uses the RADDS_RTOS build of CoreNG and the SAM3X_RTOS build of RRFLibraries.

**Instructions for building under macOS**

Using Homebrew-Cask makes it very easy to install new software on macOS: https://caskroom.github.io/

1. Download and install the gcc-arm-embedded: brew cask install gcc-arm-embedded

3. Download and install Eclipse for C++ : brew cask install eclipse-cpp

4. Download or clone the RepRapFirmware, CoreNG, FreeRTOS, RRFLibraries and DuetWiFiSocketServer projects into your workspace. Keep the folder names as is.

5. Open Eclipse and import RepRapFirmware, FreeRTOS, RRFLibraries and CoreNG projects.

6. Right click "RepRapFirmware" project and select Properties. Go To "C/C++ Build -> Settings" and change the path: /usr/local/bin/ (or some other other path where arm-none-eabi-g++ and other binaries are located)

7. Do the same for CoreNG project.

8. Build CoreNG, FreeRTOS and RRFLibraries first, then RepRapFirmware. See the instructions for Windows (above) for the configurations needed.
