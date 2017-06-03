This is firmware for controlling 3D printers and related devices using electronics with Atmel ARM main processors. The minimum specification processor it is intended for is ARM Cortex M3 with 96Kb RAM such as the SAM3X8E.

General design principles:

* Unlike most other 3D printer firmwares, it is not intended to be portable to outdated processors with limited CPU power. So make good use of the power of modern inexpensive ARM processors to implement advanced features.
* "One binary to rule them all". For a given electronics board, all features of interest to many 3D printer owners are supported by a single binary. Users do not need to recompile the firmware unless they have unusual requirements.
* "G-code everywhere". All firmware configuration is done using gcodes in the config.g file. Most type of change can be done on the fly so that you can experiment with different configurations without even having to restart the printer.
* Use high-integrity coding standards to help ensure that the firmware is reliable. In particular, don't use dynamic memory allocation after the initialisation phase. The MISRA-C++ 2008 coding standard serves as a guide as to what is acceptable practice, but compliance to it is not rigidly enforced in order to allow features from later versions of the C++ language to be used.
* Use an appropriate degree of modularity. Too little modularity makes the code hard to understand and maintain. Too much makes it hard to introduce new features when the existing module boundaries turn out to be inappropriate.
* Use object-based and object-oriented design principles where it is appropriate. In particular, class data should normally be private. Use 'friend' sparingly or not at all.

RepRapFirmware has pioneered a number of advances in 3D printing including:

* Support for mixing extruders (July 2014)
* Precise timing of step pulses, even during acceleration (Janury 2015)
* Segmentation-free delta motion (January 2015)
* Least-squares auto calibration of delta printers (April 2015)
* Accurate extruder pressure advance, including retraction before the end of a move when needed (January 2015)

For documentation on supported gcodes and configuration, see https://duet3d.com/wiki. There is a tool for generating the config.g file and homing files at https://configurator.reprapfirmware.org/.

For compiling from source, see separate file BuildInstructions.md.

Licence: GPLv3, see http://www.gnu.org/licenses/gpl-3.0.en.html
