This is firmware for controlling 3D printers and related devices using electronics based on ATSAM main processors. The current processors supported are the ATSAM4E, ATSAM4S and SAME70. The legacy SAM3X processor is also supported. There is a fork of this firmware that supports LPC1768/1769 processors. The SAME5x processor is supported in the related project Duet3Expansion.

Documentation
=============
For documentation on supported gcodes and configuration, see https://duet3d.com/wiki. There is a tool for generating the config.g file and homing files at https://configurator.reprapfirmware.org/.

Support, issues and suggestions
===============================
If you wish to obtain support, report suspected firmware issues or make suggestions for improvements, please use the forum at https://forum.duet3d.com/ so that other users of RepRapFirmware can help you or comment on your suggestions. **I am not accepting any new Issues via github** due to repeated inappropriate use of this facility.

Pull requests
=============
If you wish to submit a pull request that changes the existing behaviour of RepRapFirmware, please discuss the change with me and other users first via the forum at https://forum.duet3d.com/. RepRapFirmware is used in a wide variery of different machines and a change that may suit you may cause difficulties for other users. **I will reject any PRs that have not been discussed with me unless they are trivial bug fixes.**

Overview and history
====================
RepRapFirmware was developed originally by Adrian Bowyer and his collaborators at RepRapPro Ltd. It has been in use since late 2013, first in the open-source Ormerod, Mendel and Huxley 3D printer kits supplied by RepRapPro, and subsequently in a wide variery of 3D printers. Several current kit suppliers and many manufacturers of commercial 3D printers use RepRapFirmware.

Unlike most other 3D printer firmwares, RepRapFirmware is not intended to be run on outdated 8-bit processors with limited CPU power and RAM. So it makes good use of the power of modern inexpensive ARM processors to implement advanced features.

RepRapFirmware has pioneered a number of advances in 3D printing including:

* Support for mixing extruders (July 2014)
* Precise timing of step pulses, even during acceleration (January 2015)
* Accurate extruder pressure advance, including retraction before the end of a move when needed (January 2015)
* Segmentation-free delta motion (January 2015)
* Simulation mode, for establishing an accurate print time before committing to a print (January 2015)
* Least-squares auto calibration of delta printers (April 2015)
* Support for SPI-configured stepper drivers (August 2016)
* Resume-after-power fail as a standard feature of the firmware (October 2017)
* Compensation for the variation in extruder steps/mm with speed that is a feature of common types of extruder (January 2018)
* Compensation of heater power for changes in power supply voltage
* Dynamic acceleration control to control ringing

Supported hardware
==================
Version 2 of this fork of RepRapFirmware supports electronics based on SAM4E, SAM4S and SAME70 processors; in particular the Duet WiFi, Duet Ethernet, Duet Maestro, the forthcoming Duet 3 range, and some specialist OEM boards. There is another fork that supports electronics that use the LPC1768/1769 processors. There is also an untested (by me) build for Arduino Due/RADDS

Version 1 of this fork also supports hardware built around the SAM3X8E processor, such as Duet 06, Duet 085, Arduino Due/RADDS and Alligator. I no longer develop version 1 but if someone else wants to continue development of version 1 or add support for these processors in version 2, I will consider accepting related pull requests. Please note, version 2 uses FreeRTOS, and the additional memory requirement makes it difficult to fit both RTOS and LWIP network support into the 96Kb RAM limit of the SAM3X8E. One way to resolve this might be to use the TCP/IP stack that is available with FreeRTOS.

General design principles
=========================
* "One binary to rule them all". For a given electronics board, all features of interest to most 3D printer owners are supported by a single binary. Users do not need to recompile the firmware unless they have unusual requirements.
* "G-code everywhere". All firmware configuration is done using gcodes in the config.g file. Most types of changes can be done on the fly so that you can experiment with different configurations without even having to restart the printer.
* Use high-integrity coding standards to help ensure that the firmware is reliable. In particular, don't use dynamic memory allocation during normal operation, use it only during the initialisation and configuration phases. The MISRA-C++ 2008 coding standard serves as a guide as to what is acceptable practice, but compliance to it is not rigidly enforced in order to allow features from later versions of the C++ language to be used.
* Use an appropriate degree of modularity. Too little modularity makes the code hard to understand and maintain. Too much makes it hard to introduce new features when the existing module boundaries turn out to be inappropriate.
* Use object-based and object-oriented design principles where it is appropriate. In particular, class data should normally be private. Use 'friend' sparingly or not at all.

Compiling
=========
For compiling from source, see separate file BuildInstructions.md.

Licence
=======
The source files in this project (RepRapFirmware) are licensed under GPLv3, see http://www.gnu.org/licenses/gpl-3.0.en.html. The associated CoreNG project, which provides a partial hardware abstraction layer, includes files derived from the Advanced Software Framework (formerly Atmel Software Framework) from Microchip. Those files have a more restrictive license, in particular they may only be used for code that targets Atmel/Microchip processors.
