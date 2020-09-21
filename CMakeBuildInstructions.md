Instructions for building dc42 RepRapFirmware fork with CMake
=============================================================

### Requirements

- CMake 3.10 (or newer)
- Ninja
- cmake-gui (optional)
- GNU Arm Embedded Toolchain

CMake 3.10 is the minimum supported version to accomodate those still on Ubuntu 18.04 LTS.

It should be possible to use other build systems other than Ninja (https://ninja-build.org/). It is used in this guide since Ninja is small and cross-platform. For a list of other supported build systems, see https://cmake.org/cmake/help/v3.10/manual/cmake-generators.7.html.

`cmake-gui` should be installed together with CMake in Windows. On Linux/MacOS this is usually in a separate `cmake-gui` package.

The GNU Arm Embedded Toolchain can be downloaded from https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads.
- To build firmware version 2.03beta3 and later use version 2018-q4-major
- To build firmware version 2.01beta2 and later, use version 2018-q2-update
- To build firmware version 1.20alpha3 and later, use version 2017-q2-update
- To build firmware version 1.20alpha2 and earlier, use version arm-none-eabi-4.8.3-2014q1. A simple way of doing this is to download Arduino version 1.5.8 and install it into folder C:/Arduino-1.5.8. The compiler and associated tools will then be in folder C:\Arduino-1.5.8\hardware\tools\gcc-arm-none-eabi-4.8.3-2014q1\bin. If you already have a later version of Arduino installed including the add-on for SAM processors, you will find the compiler and tools in a different folder, for example C:\Users\<YOUR USER NAME>\AppData\Local\Arduino15\packages\arduino\tools\arm-none-eabi-gcc\4.8.3-2014q1\bin.

The 'bin' directory of the GNU Arm Embedded Toolchain should be in the system/user PATH variable. To check, you should be able to execute `arm-none-eabi-gcc --version` on a terminal/command line.

Users should also download/clone the following libraries and switch to the indicated branch.

- [CoreNG](https://github.com/likha3d/CoreNG): v3-cmake-build
- [FreeRTOS](https://github.com/likha3d/FreeRTOS): v3-cmake-build
- [RRFLibraries](https://github.com/likha3d/FreeRTOS): v3-cmake-build
- [DuetWiFiSocketServer](https://github.com/likha3d/FreeRTOS): dev

<!-- The branches above are temporary until the changes can get merged upstream. -->

The build will assume that these libraries are cloned/downloaded in the same parent directory as `RepRapFirmware` (though this can be changed). For example,

```
RepRapFirmwareBuild
    |- RepRapFirmware
    |- FreeRTOS
    |- CoreNG
    |- RRFLibraries
    |- DuetWiFiSocketServer
```

### Instructions

**Command Line/Terminal**

1. Open a terminal/command line in `RepRapFirmware` directory.
2. (Optional) Create 'build' directory and `cd` into it. This is an optional step to prevent build files from polluting the source directory -- if done, all build files will be put into the 'build' directory.

3. Execute CMake.

    If step 2 was done:

    ```
    cmake -DBOARD=[DuetNG|DuetM|RADDS|Duet3_V05|Duet3_v06|Alligator|SAME70xpld] -DCMAKE_TOOLCHAIN_FILE=../Tools/cmake/arm-gcc-toolchain.cmake -G Ninja ..
    ```

    If not:


    ```
    cmake -DBOARD=[DuetNG|DuetM|RADDS|Duet3_V05|Duet3_v06|Alligator|SAME70xpld] -DCMAKE_TOOLCHAIN_FILE=cmake/arm-gcc-toolchain.cmake -G Ninja .
    ```

    Choose one of the supported values for `BOARD` (case-sensitive). Note, only `DuetNG` is supported for now.

4. Execute `ninja` in the terminal/command line.

    ```
    ninja
    ```

**GUI**

`cmake-gui` allows users to configure `RepRapFirmware` build using a GUI.

1. Open `cmake-gui`.
2. Click `Browse Source...` and point it to `RepRapFirmware`.
3. Specify a build directory using `Browse Build...`.
4. Click `Configure`. A wizard window will appear.
    - Choose `Ninja`, `Specify toolchain file for cross-compiling`. Click 'Next'.
    - Locate `Tools/cmake/arm-gcc-toolchain.cmake`. Click 'Finish'.
5. You can change the `BOARD`, the paths to the external libraries, and other build settings. Once done, click `Configure` again.
6. Click `Generate`. The build system files should now be in the build directory specified in step 3.
7. Open a terminal and `cd` into the build directory. You should be able to execute the `ninja` command in this directory.