Instructions for building dc42 fork of RepRapFirmware under Windows
===================================================================

1. Download and install the gcc cross-compiler. A simple way of doing this is to download Arduino version 1.5.8 and install it into folder C:/Arduino-1.5.8. The compiler and associated tools will then be in folder C:\Arduino-1.5.8\hardware\tools\gcc-arm-none-eabi-4.8.3-2014q1\bin. If you already have a later version of Arduino installed including the add-on for SAM processors, you will find the compiler and tools in a different folder, for example C:\Users\<YOUR USER NAME>\AppData\Local\Arduino15\packages\arduino\tools\arm-none-eabi-gcc\4.8.3-2014q1\bin.

2. Download and install Eclipse Neon 2. You no longer need the Arduino add-on.

3. In Eclipse create new workspace C:/Eclipse/Firmware. Then exit Eclipse.

4. Download this github project as a zip file and unzip it into C:/Eclipse/Firmware. Then rename folder ReprapFirmware-dev in that folder to RepRapFirmware.

5. Repeat for github project CoreNG. The folder name should be left as CoreNG (or renamed from CoreNG-dev to CoreNG if you downloaded a dev build).

5a. If you want to build version 1.19 or later of the Duet WiFi build of RepRapFirmware then you also need to download and add project DuetWiFiSocketServer. Alternatively, just download file src/include/MessageFormats.h from that project and put it somewhere on the include path for RepRapFirmware.

6. Load Eclipse and tell it to import the CoreNG and ReprapFirmware projects (and DuetWiFiSocketServer if you have nicluded it).

7. If your compiler and tools are in a folder other than C:\Arduino-1.5.8\hardware\tools\gcc-arm-none-eabi-4.8.3-2014q1\bin, configure the path to the tools in both projects. You will find this in the project settings under C/C++ Build -> Settings -> Cross Settings.

8. Ensure there is a copy of make.exe on your PATH. If you installed Arduino 1.5.8 into C:/Arduino-1.5.8 then there will be one in C:\Arduino-1.5.8\hardware\arduino\sam\system\CMSIS\Examples\cmsis_example\gcc_arm.

9. Build CoreNG first, then RepRapFirmware. The Duet WiFi and Duet Ethernet builds of RRF use the SAM4E build of CoreNG. The Duet085 build of RRF (which also runs on the Duet06) uses the SAM3X build of CoreNG. The RADDS build of RRF has its own build of CoreNG.

Instructions for building dc42 fork of RepRapFirmware under macOS
===================================================================

1. Download and install Arduino IDE, and start it.

2. Go to: Tools -> Board -> Boards Manager... and install "Arduino SAM Boards"

3. Download and install Eclipse. Oxygen Release (4.7.0) exists while writing these instructions.

4. Download or clone the RepRapFirmware, CoreNG and DuetWiFiSocketServer projects into your workspace. Keep the folder names as is.

5. Open Eclipse and import RepRapFirmware and CoreNG projects.

6. Right click "RepRapFirmware" project and select Properties. Go To "C/C++ Build -> Settings" and change the path: `/Users/<YOUR USER NAME>/Library/Arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin`. You better visit that folder before changing on Eclipse to check whether it exists. If it does not, go to `/Users/<YOUR USER NAME>/Library/Arduino15/packages/arduino/tools/arm-none-eabi-gcc` and find the toolchain path.

7. Do the same for CoreNG project.

8. Again Right click "RepRapFirmware" project and select Properties. Go To "C/C++ Build -> Settings". Under "Cross GCC Compiler" and "Cross G++ Compiler" you will see "Includes". Add DuetWiFiSocketServer's `src/includes` folder to the "Include paths (-I)" of both "Cross GCC Compiler" and "Cross G++ Compiler".

9. Build CoreNG first, then RepRapFirmware. The Duet WiFi and Duet Ethernet builds of RRF use the SAM4E build of CoreNG. The Duet085 build of RRF (which also runs on the Duet06) uses the SAM3X build of CoreNG. The RADDS build of RRF has its own build of CoreNG.

D Crocker, updated 2017-07-12.
