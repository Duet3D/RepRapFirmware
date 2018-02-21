Instructions for building dc42 fork of RepRapFirmware under Windows
===================================================================

1. Download and install the gcc cross-compiler:

- To build firmware version 1.20alpha3 and later, you need version 2017-q2-update. You can download an installer for this version from https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads.

- To build firmware version 1.20alpha2 and earlier you need version arm-none-eabi-4.8.3-2014q1. A simple way of doing this is to download Arduino version 1.5.8 and install it into folder C:/Arduino-1.5.8. The compiler and associated tools will then be in folder C:\Arduino-1.5.8\hardware\tools\gcc-arm-none-eabi-4.8.3-2014q1\bin. If you already have a later version of Arduino installed including the add-on for SAM processors, you will find the compiler and tools in a different folder, for example C:\Users\<YOUR USER NAME>\AppData\Local\Arduino15\packages\arduino\tools\arm-none-eabi-gcc\4.8.3-2014q1\bin.

2. Download and install Eclipse Oxygen for C/C++ Developers, from http://www.eclipse.org/downloads/eclipse-packages/. You do not need the Arduino add-on.

3. In Eclipse create new workspace C:/Eclipse/Firmware. Then exit Eclipse.

4. Download this github project as a zip file and unzip it into C:/Eclipse/Firmware. Then rename folder ReprapFirmware-dev in that folder to RepRapFirmware.

5. Repeat for github project CoreNG. The folder name should be left as CoreNG (or renamed from CoreNG-dev to CoreNG if you downloaded a dev build).

5a. If you want to build version 1.19 or later of the Duet WiFi build of RepRapFirmware then you also need to download and add project DuetWiFiSocketServer. Alternatively, just download file src/include/MessageFormats.h from that project and put it somewhere on the include path for RepRapFirmware.

6. Load Eclipse and tell it to import the CoreNG and ReprapFirmware projects (and DuetWiFiSocketServer if you have included it).

7. If your compiler and tools are in a folder other than C:\Program Files (x86)\GNU Tools ARM Embedded\6 2017-q2-update\bin (if building 1.20alpha3 or later) or C:\Arduino-1.5.8\hardware\tools\gcc-arm-none-eabi-4.8.3-2014q1\bin (if building earlier versions), configure the path to the tools in both projects. You will find this in the project settings under C/C++ Build -> Settings -> Cross Settings.

8. Ensure there is a copy of make.exe on your PATH. If you installed Arduino 1.5.8 into C:/Arduino-1.5.8 then there will be one in C:\Arduino-1.5.8\hardware\arduino\sam\system\CMSIS\Examples\cmsis_example\gcc_arm.

9. Build CoreNG first. Then clean and build RepRapFirmware (the clean step is needed to make Eclipse notice that the CoreNG library has been built). The Duet WiFi and Duet Ethernet builds of RRF use the SAM4E build of CoreNG. The Duet085 build of RRF (which also runs on the Duet06) uses the SAM3X build of CoreNG. The RADDS and Alligator builds of RRF have their own builds of CoreNG.

Instructions for building dc42 fork of RepRapFirmware under macOS
=================================================================

Using Homebrew-Cask makes it very easy to install new software on macOS: https://caskroom.github.io/

1. Download and install the gcc-arm-embedded: brew cask install gcc-arm-embedded

3. Download and install Eclipse for C++ : brew cask install eclipse-cpp

4. Download or clone the RepRapFirmware, CoreNG and DuetWiFiSocketServer projects into your workspace. Keep the folder names as is.

5. Open Eclipse and import RepRapFirmware and CoreNG projects.

6. Right click "RepRapFirmware" project and select Properties. Go To "C/C++ Build -> Settings" and change the path: /usr/local/bin/ (or some other other path where arm-none-eabi-g++ and other binaries are located)

7. Do the same for CoreNG project.

8. Build CoreNG first, then RepRapFirmware. The Duet WiFi and Duet Ethernet builds of RRF use the SAM4E build of CoreNG. The Duet085 build of RRF (which also runs on the Duet06) uses the SAM3X build of CoreNG. The RADDS build of RRF has its own build of CoreNG.
