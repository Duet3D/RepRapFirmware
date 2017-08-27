Summary of important changes in recent versions
===============================================

Version 1.19.1
==============

Upgrade notes:
- If your printer is a SCARA then you now need to set appropriate M208 lower and upper limits for X and Y
- **Important!** See also the upgrade notes for version 1.19 if you are upgrading from 1.18.2 or earlier
- Recommended DuetWebControl version is 1.19
- Recommended DuetWiFiServer version is 1.19

New and changed features:
- M37 P parameter is supported. If you send M37 P"file.g" then printing file.g will be simulated and the expected print time reported.
- The bed adjusting wizard now leaves one of the screws alone when suggesting corrections
- The machine name and the password in the M550 and M551 commands may now be quoted, optionally
- If M104 T# is used and tool # is currently in standby, then its standby temperature will be set as well as its active temperature
- SCARA kinematic calculations have been speeded up
- SCARA segments/second now defaults to 100
- SCARA printers now apply the M208 axis limits to X and Y as well as to other axes. Minimum and maximum radius limits are still applied too.
- SCARA M669 S and T parameters can be changed on the fly, i.e. re-homing is no longer required when they are changed
- Minimum limits are applied to the parameters of M92, M201 and M203 commands to avoid firmware crashes if bad values are supplied

Bug fixes:
- Incorrect, very large height errors were sometimes shown in the G29 height map when the number of probe points used approached the maximum supported
- M669 was supposed to report the current kinematics, but didn't except for SCARA printers
- SCARA kinematics didn't apply limits to the Z axis or to any additional axes. Delta printers didn't apply limits to any additional axes.
- SCARA forward kinematics were wrong
- SCARA minimum radius limit was wrong
- If the number of commands in the deferred command queue exceeded 8, commands could be lost
- If a G1 command was used with a coordinate for an axis that X or Y was mapped, that coordinate was ignored unless the S1 or S2 command modifier was used. This affected tool change files on IDEX machines.
- If a reset was forced by the software watchdog because it got stuck in a spin loop, the reset reason in M122 was displayed as 'User'

Version 1.19
============

Upgrade notes:
- Recommended DuetWebControl version is 1.19
- Recommended DuetWiFiServer version is 1.19
- **Important!** If you use an IR Z probe or some other type that does not need to be deployed, delete the files sys/deployprobe and sys/retractprobe.g if they exist, because they are now called automatically. You can do this in the System Files Editor of the web interface.
- **Important!** On a CoreXY machine, if upgrading from a version prior to 1.19beta9, you need to reverse the Y motor direction in the M569 command. Similarly for CoreXYU machines.
- **Important!**  When upgrading a Duet WiFi from 1.18.2 or earlier firmware, see the important notes at https://duet3d.com/wiki/Upgrading_to_DuetWiFiFirmware_1.19.
- Height map files created with firmware 1.18 or earlier cannot be read by firmware 1.19, so you will need to run G29 S0 again to generate a new heightmap.csv file
- Height map filenames in G29, M374 and M375 commands must now be enclosed in double quotes
- Every heater that you use must now be configured using a M305 command with a P parameter that identifies the heater. Previously, if a heater used default thermistor parameters, you could omit the M305 command for that heater.
- If you are using a Duet Ethernet and you are letting your router allocate an IP address automatically, the IP address will change, because the default MAC address now depends on the board unique ID
- If you have more than 32 probe points in your bed.g file, you will have to reduce the number to 32
- If you send a G1 command with multiple E parameters (e.g. G1 X10 Y20 E1.0:1.5:2.0) then this is now only allowed when in relative extrusion mode (M83). If you try doing this in absolute extrusion mode, no extrusion will take place and an error message will be generated.
- The behaviour of the R parameter in G1 commands has changed. Previously, G1 R1 with no additional parameters restored the head position for all axes to the values just prior to the last pause command, and G1 R2 with no additional parameters restored the head position for all axes to the values just prior to the last tool change. The behaviour now is that only axes that are mentioned in the G1 command have their values restored. So instead of using G1 R1 in resume.g to restore the head position, you must now use G1 R1 X0 Y0 Z0. As before, any non-zero axis parameters will be added to the saved position of that axis. This change has been made so that additional axes used as substitute X and Y axis are not automatically restored.

Known issues:
- Although the WiFi module can now be put into access point mode using M589 and M552 S1, WiFi access does not work properly in access point mode.
- When a power fail occurs and power fail action has been configured, the firmware has to wait for some or all moves in the print queue to complete before the powerfail.g command can be run. This means that you may run out of power before powerfail.g is run, which could result in a few moves being skipped and the nozzle becoming stuck to the print.

New features since 1.18.2:
- Resume-after-power-failure support on Duet WiFi/Ethernet. See https://duet3d.com/wiki/Setting_up_to_resume_a_print_after_a_power_failure.
- Bed levelling using multiple independent Z motors (M671). See https://duet3d.com/wiki/Bed_levelling_using_multiple_independent_Z_motors. In Cartesian/CoreXY printers without multiple independent Z motors, this facility can be used to determine what adjustments to make to bed leveling screws instead.
- Support 2 additional external drivers connected to the CONN_LCD socket
- FTP and Telnet are now supported on the Duet WiFi but are disabled by default. Use M586 to enable them.
- Added support for M591 command to configure filament sensors (but filament sensor support still incomplete)
- Added support for M672 command to program the Duet3D delta effector sensitivity
- SCARA kinematics is believed to be fully working
- Added support for binary file upload in M559 and M560
- Added support for an additional SX1509B port expander
- Pause commands issued while a macro is being executed are deferred until the macro has completed, and they can be resumed
- M568 command to enable/disable mixing no longer does anything because mixing is always enabled. Mixing is not used if the E parameter in the G1 command has multiple values.
- Message boxes can now have other axis jog buttons as well as Z (thanks chrishamm)
- Probe deployment and retraction for G30 and G29 commands is now handled automatically. You should still include a M401 command before the first G30 command in bed.g and a M402 command after the last one, so that the probe deploys and retracts once for the entire sequence instead of once per G30 command.
- M577 now allows separate X and Y spacings, use Sxxx:yyy
- Volumetric extrusion is now supported (M200)
- Additional tool/heater data is provided to DWC (thanks chrishamm). Using Heater 0 as a tool heater should now work.
- The '(' character in a gcode file now introduces a comment, just as the ';' character does. The comment is terminated at end-of-line. This is not the same as in some CNC gcodes, where a comment introduced by '(' is terminated by ')'.
- Heater tuning peak detection algorithm changed. This may fix some "no peak detected" reports during auto tuning.
- The heater dead time is how taken as 60% of the peak delay time intead of 100%, which results in more aggressive PID parameters.
- TEMPERATURE_CLOSE_ENOUGH reduced from 2.5C to 1.0C
- Reduced the maximum number of random probe points on Duet WiFi/Ethernet to 32 to avoid running out of memory during delta auto calibration
- DriveMovement structures are now allocated dynamically from a freelist, to allow more moves to be queued in typical cases. The number free and minimum ever free is included in the M122 report.
- If the G10 command is used to set the standby temperature of a tool that is on standby, the live temperature is adjusted accordingly
- SCARA parameters configured using M669 now include X and Y bed origin offsets
- Baby stepping is no longer cleared when you home the printer or probe the bed
- The Y axis can now be mapped in a similar way to the X axis
- The meaning of the first M669 crosstalk parameter for a SCARA printer has changed. A zero value now means that the proximal motor does not affect the proximal-to-distal arm angle.
- The CoreXY kinematics calculations have been changed to conform to the way they are defined in other firmwares and at CoreXY.com. See the important upgrade notes. The CoreXZ and CoreXYU kinematics have been changed similarly.
- The maximum allowed target temperature for auto tuning now depends on the configured maximum temperature for that heater
- The heater gain that provokes a warning when setting heater model parameters with M307 or after auto tuning now depends on the configured maximum temperature for that heater
- If a PT100 interface reports an error when executing a M105 command with X parameter, the firmware now reports the nature of the error if possible
- When M305 is used with an X parameter to select a PT100 sensor, a new optional F parameter configures rejection of 50Hz (F50) or 60Hz (F60) interference
- Added support for prototype thermocouple adapter for type J etc. thermocouples (temperature sensor channels 150 to 157)
- M408 S0 response now includes message box details for PanelDue (needs new PanelDue firmware)
- M105 temperature reports are now in tool heater order instead of heater order
- M105 temperature reports now report the setpoint temperatures as well as the current temperatures, to keep Repetier Host happy
- Adjustment of head position to account for a changed tool offset is now deferred until the next move that includes axis movement, instead of the next move of any sort
- When grid probing, if a point cannot be reached by the Z probe, a message is emitted
- Command and parameter letters in gcode commands are now case-insensitive as per the NIST specification
- On the Duet WiFi and Duet Ethernet up to 9 axes are now supported, named XYZUVWABC
- In height map files the mean height error and deviation from the mean are now saved to 3 decimal places instead of 2
- Z probing moves now use 250mm/sec^2 acceleration unless a lower Z acceleration limit has been configured. This is to avoid triggering nozzle-contact sensors at the start of a probing move.
- M291 command is provided to display a message box with options for timeout, acknowledgement and axis jog buttons. This will require additional changes to DWC and PanelDue before it is fully usable.
- M292 command is provided to acknowledge M291 messages
- Manual delta calibration and bed compensation is supported (use P0 in the M558 command to indicate that there is no Z probe)
- Minimum value for S parameter (maximum heater PWM) in M307 command is reduced from 20% to 1%
- Core XYU kinematics are now supported (thanks Lars)
- RADDS build now supports 9 motors (thanks Tom)
- If a homing move uses parameter S3 instead of S1 then the axis minimum or maximum value is set to the current position instead of vice versa
- M589 with no parameters now reports the Duet's own SSID
- M589 S"*" now deletes the Duet WiFi's own access point details
- G1 command now take an optional P parameter which is a bitmap of output ports to turn on and off for the duration of the move. The mapping of bits to ports and the port switching advance time is configured using M670.
- M42 command now supports F parameter to select the PWM frequency
- Up to 10 virtual heaters can be defined using M305 commands, numbered 100-109. Virtual heater 100 defaults to sensing the MCU temperature (sensor channel 1000), and on the Duet WiFi/Ethernet virtual heaters 101-102 default to sensing the TMC2660 temperature warning/overheat sensors on the Duet and the DueX expansion board respectively (sensor channels 1001-1002).
- Heaters can be named by adding parameter H"name" in the M305 command. The quote-marks are compulsory. The temperatures of named virtual heaters are made available to DWC for display.
- Fans can be thermostatically controlled based on the temperatures of any real or virtual heaters.
- Fans can be thermostatically controlled in proportional mode by specifying a temperature range e.g. T40:50. If you specify only one temperature, or the second temperature is not greater than the first, bang-band mode will be used as before. In proportional mode the S parameter is not used but the L value is honoured.
- Current loop temperature sensors are now supported (sensor channels 300-307). M305 parameters L and H set the temperatures corresponding to 4mA and 20mA current respectively.
- M305 with just a P parameter now reports the sensor type along with the other parameters.
- It is now possible to create additional axes that are not visible in the user interface. To do this, add parameter P# to your M584 command where # is the number of axes you want to be visible (e.g. 3 = just X,Y,Z).
- Thermostatic fans now have a 1C hysteresis to reduce the effects of noise
- M204 S parameter is supported for backwards compatibility e.g. with Cura
- Some other exceptions now record a stack trace, as Hard Fault exceptions already did
- M122 now displays additional information: firmware name and version, hardware type, last software reset reason, unique board ID if available, and filament sensor status
- On the Duet Ethernet, the default MAC address is generated from the board ID
- The pause.g, resume.g and cancel.g files are not run unless all axes have been homed.
- The default maximum hot end temperature has been increased to 288C
- String parameters in some gcode commands, such as filenames, can now be enclosed in double quotation marks to avoid ambiguity
- When tuning a bed or chamber heater, more time is allowed for the temperature to start rising
- On the Duet WiFi the network code has been rewritten. The web server now runs on the Duet instead of on the WiFi module. FTP and Telnet are supported if enabled using M586. New commands M587, M588 and M589 are supported. The meaning of the M552 S parameter has changed: S-1 holds the WiFi module in the reset state, S0 holds it in the Idle state allowing it to process M587/M588/M589 commands, S1 starts it in client mode and S2 starts it in access point mode. The M122 diagnostic report includes WiFi module parameters unless the WiFi module is being held in the reset state.
- Added support for simple switch-based filament sensors and the Duet3D filament sensor.

Bug fixes:
- XYZ coordinates could be reported as NaN in DWC status responses, causing AJAX errors
- Axis compensation now takes account of X and Y axis mapping
- Messages rejected by webserver now generate a generic message only if debug is enabled
- Heater 0 output can now be used for general output after its model is disabled
- Fixed issue with homing axes using endstop connectors on DueXn expansion board
- M669 with no parameters now reports the bed offset on a SCARA machine as well as the other parameters
- Heater model max PWM is now set to tuning PWM after auto tuning (thanks cangelis)
- If an HTML file uploaded over USB contained an embedded leading substring of the EOF string, incorrect data was written to file (thanks cangelis)
- G2 and G3 arc movement commands didn't work when the X axis was mapped
- On a SCARA machine, sending G92 X0 Y0 caused the Duet to return position data containing NaNs to Duet Web Control, which caused it to disconnect
- When an extruding move had a lot more acceleration than deceleration, too many extruder steps were scheduled. A check threw the additional steps away so that printing was not affected, but a step error was logged.
- Except when delta kinematics were being used, speeds and accelerations were limited independently for the X and Y axes. This is correct for Cartesian printers, but not for CoreXY, Scara etc. The speed and acceleration of XY movement is now always limited to the lower of the specified maximum X and Y speed and acceleration unless Cartesian kinematics are being used.
- When a print was paused and resumed, it didn't always resume at the correct move
- When G92 or M374 was used to save a height map using the default filename, sometimes it would save to a random filename instead
- When G92 S1 or M375 was used to load a height map using the default filename, sometimes it would fail to load
- When the current tool offsets change because of a tool change or a G10 command, the new offsets are applied to the endpoint of the next move even if it has no movement along axes with changed offets
- The tool change restore point coordinates now take account of X axis mapping
- On a delta printer the nozzle height is now limited to reachable values, to avoid the motors trying to move the carriages past the physical endstops
- M552 with no parameters now reports the current IP address as well as the status
- Some Duets would restart immediately after initial power up and then run normally
- Thermostatic fans can now depend on heater numbers 100 (CPU temperature), 101 (TMC2660 drivers on Duet WiFi/Ethernet) and 102 (TMC2660 drivers on DueXn expansion board)
- MCU temperature measurement now includes an averaging filter to reduce noise
- On the Duet WiFi, when you download a file it no longer opens it in the browser but gives you a "Save file as" prompt instead.
- FTP is now working on the Duet Ethernet
- Filament consumed during tool change macros is no longer added to the total filament consumed for the purpose of estimated the percentage completion of the print

Version 1.18.2
==============

New features:
- On the Duet WiFi and Duet Ethernet, recognise the latest production DueX2 and DueX5 boards
- Support fan 8 on the latest DueX5 boards

Bug fixes:
- M42 gcode commands were not synchronised with movement
- M21 did not full reset the SD card state, which could lead to errors if the SD card was modified outside the Duet and then remounted using M21
- The M106 command accepted a fan number (P parameter) one higher than it should

Version 1.18.1
==============

Bug fixes:
- Corrected USB VID/PID

Version 1.18
============

New features:
- First official release for Duet Ethernet
- M204 is now implemented (P and T parameters only)
- M997 command now checks that the start of the main firmware file looks sensible
- The rate at which "No tool selected" and "Attempt to move before homing a delta printer" messages are generated is now limited
- Added VSSA fault detection if the hardware supports it
- If there are too many probe points implied by a M557 grid definition command, the firmware displays a suggested increased spacing
- On the wired Duets, M586 can be used to set which network protocols are supported and which port numbers are used. By default, http is enabled, ftp and telnet are disabled.
- Baby stepping is now implemented using the M290 command. The accumulated baby stepping amount is reported in M408 replies.
- Faster and easier-to-use auto tune algorithm with more consistent dead time measurement
- M109, M190 and M191 commands now send the temperatures once a second if the command came from the USB port and Marlin emulation is chosen
- The name of the firmware file to load is now passed to IAP, so that iap4e.bin can be used on both the Duet WiFi and the Duet Ethernet
- Added code queue so that fan commands etc. are synchronised to movement (thanks chrishamm)
- Added chrishamm's input buffering code (thanks chrishamm)
- Reduced the Duet WiFi VIN over-voltage detection threshold from 29.5V to 29.0V
- Live coordinates are now reported to 3 decimal places instead of 2
- When using a Z probe type other than 2, the probe output is sampled every 1ms instead of every 2ms for faster response
- PanelDue status responses continue to be sent while executing M109/116/190/191 commands
- Increased maximum permitted motor current on TMC2660 drivers to 2.4A
- Improved error messages when a M303 command has an out-of-range parameter
- We now use a USB VID/PID allocated to us instead oif Atmel's CDC example ones
- The Windows device driver now supports the Bossa Program Port, so you can now use bossac even if you don't have the Atmel or Arduino device driver for it installed

Bug fixes:
- Fixed issue with loading height map file when the number of probe points along the X axis is large
- Interpolation near the edges of the bed was incorrect when mesh bed compensation was used (thanks ChristophPech)
- On the Duet WiFi, if you sent command M122 while the machine was printing then occasionally it would stop and reset due to a watchdog timeout
- If multiple input sources sent overlapping G4 (dwell) commands, either or both of them would not be executed correctly

Other changes
- M552 no longer includes the option to set the HTTP number. Use M586 instead.
- M557 P parameter to set probing points is no longer supported. Use a bed.g file instead.
- Temperatures default to 0C instead of -273C

Known issues
- If you enable tool mixing, you should use relative extrusion only. If you use absolute extrusion, then if you pause and resume the print, the extruder is likely to extrude the wrong amount of filament in the first move after resuming.
- If you use M586 to disable FTP or Telnet on the Duet 085 or 06 after you have previously enabled them, the firmware refuses new connections but does not terminate any existing connections.
- FTP on the Duet Ethernet cannot be used to do file transfers

Upgrade notes
- If you connect via USB from a Windows PC you should install the updated device driver in the Drivers folder of this repository
- If you use the M552 R parameter to change the HTTP port number on a wired Duet, you will need to use M586 instead
- If you use FTP or Telnet on a wired Duet, you will need to enable them using M586
- It is recommended that you re-run heater auto tuning when upgrading to 1.18 from an earlier release
- You may find that your Z probe trigger height is slightly higher than before, so you should re-measure it
- When upgrading to this firmware, also upgrade iap.bin or iap4e.bin to latest version

Version 1.17e
=============

Bug fixes:
- Fixed an integer divide-by-zero bug in the LWP library that could cause the Duet 0.8.5/0.6 build of RepRapFirmware to crash with a hard fault

For known issues, see version 1.17d.

Version 1.17d
=============

New features:
- G2 and G3 arc movement commands are implemented.
- If the controller is reset because of a Hard Fault exception, additional debugging information is stored in the Software Reset Data and displayed by M122
- Faster step pulse generation - now up to 180kHz simultaneously on 3 motors on the Duet WiFi
- The M102 command (generated by S3D when using firmware retraction) is ignored instead of provoking a warning message
- Firmware retraction/unretraction now defaults to 2.0/2.0 instead of 1.0/2.0 mm

Bug fixes:
- If tool mixing was used with absolute extruder coordinates, the extrusion amount was calculated incorrectly.
- Second SD card didn't work on Duet085 and RADDS builds
- The deprecated M206 command rtakes account of whether the units are mm or inches
- Also fixed bug with absolute extruder coordinates introduced in temporary versions 1.17c+1 and +2

Known issues
- If you enable tool mixing, you should use relative extrusion only. If you use absolute extrusion, then if you pause and resume the print, the extruder is likely to extrude the wrong amount of filament in the first move after resuming.

Version 1.17c
=============

New features:
- The layer height comment in gcode files produced by Matter Control is now recognised
- M101 and M103 (firmware retraction commands generated by Simplify3D) are now recognised
- G20 bed probing now prints the mean height error and RMS deviation to 3 decimal places

Bug fixes:
- G30 with no parameters now sets the machine position to the Z probe trigger height instead of the user position. This means that running G28 (home) and G29 (bed probe) alternately no longer causes the bed height map to creep up or down in average height when Z homing uses the Z probe. However, it means that the Z height displayed after a G30 command is no longer always the same as the Z probe trigger height.
- In firmware 1.17 the speed change command by the M220 command got delayed until the next G1 or G0 command with an F parameter was read.
- In firmware 1.16 and 1.17 the acceleration used in G11 un-retraction commands was sometimes incorrectly reduced if pressure advance was configured on a different extruder.
- In firmware 1.16 and 1.17 when pressure advance was configured, the extruder acceleration limit implied by pressure advance used the pressure advance value for the wrong extruder.
- Firmware retraction did not work if additional axes had been configured e.g. for an IDEX printer.
- G29 commands using the P parameter with an upper case M in the filename were recognised as M commands instead

Version 1.17b
=============

Printer status:
- New printer status "Tool change" implemented. This is recognised by DWC 1.14 and the forthcoming PanelDue firmware 1.16.
Bug fix:
- M226 (gcode-initiated pause) was hanging the printer (thanks chrishamm for the fix)
Z probe:
- For Z probe types 4 and higher, the MOD signal on the Z probe connector is driven high at the start of a probing move and low at the end. This is to help with certain types of Z probe, in particular accelerometer-based Z probes.
Other:
- Upgraded DWC files to 1.14 release (thanks chrishamm)

For upgrade notes, see version 1.17.

Version 1.17a
=============

Homing speeds:
- The speed factor (M220 command, or speed control in DWC and PanelDue) no longer affects the speed of homing moves or other special moves (i.e. G1 commands with S1 or S2 parameter)

Bug fixes:
- Fixed the M120 (Push) command
- Setting relative extruder movement (M83) or a feed rate (G1 Fxxx) in config.g now works again
- The F (feed rate) parameter is now processed correctly on a G1 command with the R parameter
- If you used M106 to change a fan PWM frequency, the change did not take place until you next changed the speed
- On boards with SAM3X processors, due to a chip bug the fan speed was sometimes incorrect if you changed the PWM frequency
- If an http client requested a non-existent file that was not an HTML file, the server incorrectly returned a 404 HTML page

For upgrade notes, see version 1.17.

Version 1.17
============

Implemented grid bed compensation:
- M557 defines the grid
- G29 probes the grid, and saves the height map to file and activates it
- G29 S1 loads and activates a height map
- G29 S2 clears the height map (so does M561)
- M374 also saves the height map, and M375 loads a height map
- M376 sets the compensation taper height
- Long moves are segmented when grid compensation is in use so as to follow the contours of the bed
- Duet Web Control 1.14 can display the height map graphically (thanks chrishamm)

Changes to information extraction from gcode files:
- Recognise generated-with comment written by newer versions of Cura
- Recognise filament usage info in kisslicer-generated gcode files (thanks chrishamm)

 M500, M501 and M502 now use config_override.g instead of flash memory. The parameters saved and restored are:
- M307 auto tune results
- PID parameters, if you used M301 to override the auto tune PID settings
- Delta printer M665 and M666 settings
- G31 trigger height, trigger value and X and Y offsets
- The M501 auto save option has been removed

 Duet 0.8.5/0.6 web server changes:
- Support gzipped files, and look for gzipped versions of files before looking for regular ones

 Changes to Z probe configuration and usage
- Z probe offsets are now applied during G30 probing with specified XY coordinates, including during delta auto calibration
- Z probe recovery time can be defined (R parameter in M558) and adds a delay between the travel move and the probing move
- Added T parameter to the G31 command to specify Z probe type. This allows you to view the parameters for the Z probe(s) and to set parameters for a particular Z probe type without selecting that type. G31 P or G31 P0 prints the parameters of the currently-selected Z probe.

 Changes to heater management:
- You can now specify a Steinhart-Hart C coefficient in the M305 command, for better thermistor accuracy over a wide temperature range. If you do use a non-zero C coefficient then you will need to change the B (beta) parameter as well. The B parameter should be the reciprocal of the Steinhart-Hart B coefficient.
- The thermistor disconnected detection now takes account of the thermistor parameters configured with M305. This should allow the Dyze thermistor to be used without getting so many 'thermistor disconnected' reports. You may need to use a small negative H parameter in your M305 command to make it reliable.
- M143 now takes an H parameter to specify the heater whose temperature limit you are setting. If it is not provided then heater 1 is assumed.
- M109 and M190 commands now support both R and S parameters in the same way as Marlin. If you specify temperature using the S parameter, the firmware will wait for the heater to heat up but not to cool down. If you specify temperature using the R parameter, the firmware will wait both when heating up and when cooling down.
- M104 and M109 default to tool 0 if no tool is selected and no T parameter provided
- M109 now selects the tool after setting the active temperature if it was not already selected
- M191 (set chamber temperature and wait) is now supported, with both R and S parameters as for M190. However you may wish instead to use M141 followed by M116 later to wait for all temperatures.
- Removed S and T parameters from M301 command. Use the M307 command instead.
- M301 with negative P parameter no longer sets bang-bang mode. Use M307 instead.
- Increased the default max temperature excursion to 15C
- Setting the temperature of a tool heater no longer sets the heater temperature unless the tool is currently selected or no tool is selected

Changes to M571 command:
- M571 now accepts a P parameter to select the output pin
- Added F parameter to M571 command to set PWM frequency

Bug fixes:
- Firmware retraction with Z hop now works when using 'retract on layer change' in slic3r
- Fixed bad JSON message during printing when there were no active extruders
- Software reset code storage/retrieval now works on Duet WiFi
- Fixed reset reason text because on the Duet WiFi a watchdog reset can look like an external reset
- Fix for adjusting the mix ratio during printing when using absolute extruder coordinates with mixing extruders, except for pause/resume
- Workaround for DWC 1.13 including a volume ID in the new file path when renaming files across directories
- Bug fix: M300 now causes a beep on PanelDue again (was broken in 1.16).
- Bug fix: when a move was aborted, the head position was incorrectly calculated if the move has a direction reversal scheduled later on. In practice this situation did not arise.

Miscellaneous changes:
- Multiple commands from different sources that do not interfere are executed concurrently. Previously, only a few status reporting commands could be executed concurrently with other commands. 
- If the M569 command is used with the 'R' parameter to set the enable polarity then the corresponding driver is disabled after the polarity is set.
- Experimental code has been added to log Z probe transitions during a move, for use when calibrating nozzle offsets.
- Removed the undocumented M201 max average printing acceleration parameter
- Added exception handlers and store a software reset code when an exception occurs
- Removed Duet WiFi prototype 1 build configuration and added RADDS build configuration
- Tool offset is no longer applied to G1 moves with S1 or S2 modifiers
- A 2nd controlled fan using an external mosfet driven by expansion connector pin 25 on a Duet 0.6 is no longer inverted

Upgrade notes for 1.17 - VERY IMPORTANT TO AVOID DAMAGE!!!
----------------------------------------------------------
- On the Duet 0.6 and 0.85 the default direction for the X motor is now forward (as it is for all other motors and on the Duet WiFi). If you do not have a M569 P0 command in your config.g file then you will need to add M569 P0 S0 in order to keep the previous behaviour.
- On the Duet 0.6 and 0.85 the default bed thermistor resistance at 25C is now 100K (as it is for the Duet WiFi). If you have an Ormerod, Huxley Duo or RepRapPro Mendel printer with a 10K bed thermistor, you will need to add parameter T10000 to the M305 P0 command in config.g if you don't have that already.
- On the Duet 0.6 and 0.85 the default hot end heater thermistor parameters are changed to match the Semitec thermistor used in the E3DV6 and other popular hot ends. If you have a printer built from a kit supplied by RepRapPro then you should use the following parameters in your M305 P1 command to restore the previous behaviour: B4138 C0
- The default bed temperature limit is 125C. Use M143 H0 S### if you need to increase it.
- If your printer has multiple hot end heaters and you use the M143 command to change the temperature limit, you will need to use one M143 command with an appropriate H parameter for each heater.
- If you limit the maximum heater PWM using the S parameter in a M301 command, you will now have to use M307 to do this instead. Similarly the M307 A parameter takes the place of the M301 T parameter (divide the T parameter by 255 and take the reciprocal to get the A parameter).
- On the Duet 0.8.5 any parameters you saved to flash memory will be lost when upgrading to RC1 or later.
- You will need to add M501 in your config.g file, at the end or just before the T0 command if you have one, if you want to load saved values automatically at startup.
- If you use auto delta calibration and you have Z probe X and/or Y offsets defined, you should adjust your probe points to allow for the fact that the firmware will now place the probe over the points you specify instead of the nozzle
- If your pause.g and resume.g contain any extruder movement commands to retract or prime filament, make sure these files have the M83 command at the start to select relative extruder coordinates
- The recommended web interface is Duet Web Control 1.14. To use DWC 1.14 on the Duet WiFi, you must be using version 1.02 or later of DuetWiFiServer. Version 1.03-ch is recommended.

Known issues for 1.17
---------------------
- The M120 (Push) command doesn't work in macro or gcode files, because it causes execution of the file to be terminated
- Setting relative extruder movement (M83) or a feed rate (G1 Fxxx) in config.g has no effect once config.g has completed
- The F (feed rate) parameter is not processed correctly on a G1 command with the R parameter. In the resume.g file, you can use a G1 Fxxx command to set the feed rate, then leave the F parameter off the G1 R1 command.
- If you enable tool mixing, you should use relative extrusion only. If you use absolute extrusion, then if you pause and resume the print, the extruder is likely to extrude the wrong amount of filament in the first move after resuming.
- Firmware retraction won't work properly if you have created additional axes, for example the U axis on an IDEX machine.

Version 1.16
============

- Support the DueX2 and DueX5 expansion boards for the Duet WiFi
- Add support for up to 3 additional axes U, V and W. The number of axes is reported to DWC and to PanelDue.
- Add support for X axis remapping in the M563 tool creation command
- Add support for default fan remapping in the M563 tool creation command
- Support dual material prints and dual simultaneous prints on IDEX (independent dual X carriage) printers. See [https://duet3d.com/wiki/Configuring_multiple_independent_X-carriages_on_a_Cartesian_printer].
- Support minimum fan speeds and fan PWM blipping when starting fans from standstill
- Files uploaded to SD card are now time-stamped (thanks chrishamm)
- Sending M307 A-1 C-1 D-1 disables the PID for a heater channel, allowing its pin and driver to be used for other purposes
- Sending M106 P# I-1 disables the specified fan so that its control pin can be used as for general purpose output (M42) or a servo (M280). Caution: the polarity may not be what you expect.
- Changed M42 pin numbering, see [https://duet3d.com/wiki/Using_servos_and_controlling_unused_I/O_pins]
- Added M280 servo support, see [https://duet3d.com/wiki/Using_servos_and_controlling_unused_I/O_pins]
- Allow separate firmware un-retract speed to be configured in M207
- Allow negative extra un-retraction in M207
- Support expansion connector pin PB6 on the Duet WiFi and use it by default for a cooling fan tacho input
- Added Z probe type 6 (switch on E1 endstop connector)
- Added optional I1 parameter to the M558 command to reverse the sense of the Z probe reading. This replaces M574 E0 S0 when using Z probe type 4 and also works with other types of Z probe.
- Support heater 6 on Duet 0.8.5 (untested and probably incomplete). You need to send a M307 H6 command with valid model parameters to enable heater 6, and Fan 1 will be disabled (they share the same control signal).
- On the Duet 0.8.5, Fan 1 no longer defaults to thermostatic but instead defaults to fully on. This is in case you have heater 6 connected.
- Sending a T command to select a tool no longer runs the tool change macros if the specified tool is already selected
- M122 command now includes the status of all TMC2660 drivers on the Duet WiFi and expansion boards
- Bug fix: if there were very many files in the /gcodes folder of the SD card then DWC would give and Ajax error when trying to load the files list (thanks chrishamm)

Upgrade notes:

- If you are using M42 then you will need to adjust the pin numbers in your M42 commands
- If you are using a type 4 Z probe (i.e. switch connected to E0 endstop input) and you are using M574 E0 S0 to invert the polarity, you will need to use the I1 parameter on the M558 command instead
- If you are using a Duet 0.8.5 and you were relying on Fan 1 being thermostatic by default, you will need to configure it yourself by adding command M106 P1 T45 H1:2:3:4:5:6 to config.g. 
- The recommended web interface is DWC 1.13
- You can use either DuetWebServer-1.03-ch or DuetWebServer-1.02 with this release. You may find that the -1.03-ch version is faster and provides a more reliable connection.

Version 1.15
============

- Implemented automatic heating model calibration and PID tuning. See [https://duet3d.com/wiki/Tuning_the_heater_temperature_control] for details and instructions.
- Implemented much better heater safety monitoring, based on a model (first-order-plus-time-delay) of each heater/sensor system, using default models if none have been configured
- Implemented M38 (thanks Chrishamm).
- Added support for an SD card socket on the SPI bus, and implemented M21/M22 to support this. This was done primarily for the Duet WiFi and has not yet been tested on the wired Duets.
- Added support for new Duet Web Control functions, in particular the sys file editor (thanks chrishamm)
- Increased the PWM resolution
- Included the status of the SD card-detect signal in the M122 response
- Further improvements to the speed of step pulse generation
- Improved motion smoothness when printing circles and complex shapes at high speed
- Bug fix: PID parameters entered using M301 had the I parameter doubled and the D parameter halved, and vice versa for PID parameters reported by M301
- Bug fix: the object height was sometimes incorrectly extracted from gcode files
- Bug fix: slicer comments in gcode files that included certain characters (e.g. '\') would give ajax errors when the file information was returned
- Bug fix (1.15e): using some nonzero values of M572 pressure advance could cause incorrect extruder movement and layer shifts
- Bug fix (1.15e): step errors were not always recorded
- Further changes specific to the Duet WiFi firmware build, see [https://www.duet3d.com/forum/thread.php?pid=1104#p1104] for details

Upgrade notes:

- You may get heating faults reported if you do not tune the heaters
- If you want to continur using tour own PID parameters instead of auto-tuning, you must double your I parameters and halve your D parameters in your M301 commands
- The recommended web interface is DWC 1.12

Version 1.14
============

- When multiple motors are due to step, generate all the step pulses simultaneously
- Added support for multiple drivers for a single axis (M584)
- Added support for extended step pulse width when using external drivers (T parameter on M569 command)
- Removed XYZE parameters from M569 (use M584 instead)
- Added M913 command (set motor % of normal current), allowing the motor current to be temporarily reduced e.g. for homing and loading filament
- Added Z probe type 5, which is a normally-closed switch or active-high 3.3V digital signal connected to the IN pin of the Z probe connector on the Duet 0.8.5 and Duet WiFi
- Duet WiFi default Z probe threshold is now 500 (the recommende dvalue for most types of Z probe)
- Duet WiFi defaults to using a low end homing switch on the X axis (Ormerod and Huxley, users take note!)

Upgrade notes:

- If you are using M569 commands to remap axes and extruders to different drives, that will no longer work. Use the new M584 command instead.
- If you use the M584 command, it should come before any M350 and M906 commands in your config.g file.

Version 1.13
============

* First firmware release for the new Duet WiFi
* Added support for M581 and M582 commands, including 'only if printing a file' condition in M581 command
* Fixed M117 for PanelDue (needs PanelDue firmware version 1.14)
* Firmware update messages are now sent to USB and PanelDue (needs PanelDue version 1.14 firmware to display them)
* M122 responses are now sent only to the comms channel that requested them
* Added gcode queue underrun counter, displayed in M122 Move diagnostic info
* SD card interface speed is included in M122 Platform diagnostic info
* Added support in M997 command to update additional firmware modules on Duet WiFi
* Corrected a possible problem with multiple incompatible gcodes being executed concurrently
* Support H parameter on M0 and M1 commands
* Treat M25 within the file being printed the same as M226
* Added additional M37 simulation modes to help identify bottlenecks
* Process M0 and M1 when in simulation mode
* When executing M0/M1 commands and no print is paused, execute stop.g/sleep.g if they exist
* Reduced interrupt latency, by not disabling all interrupts when starting a new move, to avoid losing characters sent by PanelDue
* Temporary fix for RTD temperature spikes causing failed prints
* Don't print extruder positions in M114 because they are always zero
* Allow setting of fan PWM from 50% upwards when a fan is in thermostatic mode
* Reduced default extruder heater PWM frequency to 500Hz
* Axes are flagged as not homed after using M350 to set microstepping

Upgrade notes:

The recommended web interface is DWC 1.11 for the Duet, and 1.11a-dc42 for the Duet WiFi.

Version 1.12
============

* PT100 and other RTD sensors are now supported
* When a print is paused and then cancelled, the firmware now attempts to run file sys/cancel.g. It only turns the heaters off if that file is not found.
* The file upload speed over the web interface has been increased. Those who were getting slow file upload speeds are likely to see the most benefit.
* If a chamber heater is configured, it is now exempted from the heater timeout, just like the bed heater
* Two changes have been made to improve print quality when printing at high speeds. First, enabling extruder pressure advance is less likely to result in sequences of short moves having sawtooth velocity profiles. Second, a "Maximum average printing acceleration" can now be configured (M201 P parameter). Using this to restrict average acceleration will flatten out any remaining sawtooth velocity profiles.
* The firmware now reports itself to the USB subsystem as "Duet 3D printer control electronics",instead of as an Arduino Due. A Windows driver file is provided.
* The default maximum hot end temperature is reduced to 260C.
* Bug fix: the USB interface exposed by firmware version 1.11 was not recognised by some versions of Windows on some computers, resulting in a driver installation error
* Bug fix: simulation mode (M37) did not work
* Bug fix: setting the motor current on the non-existent 9th driver on a Duet 0.6 might have undesirable side-effects

Upgrade notes:

- If you are using Windows, please install the driver at [https://github.com/dc42/RepRapFirmware/tree/dev/Driver]. This driver does not install any new binaries, it simply tells Windows to use its usbser.sys driver to communicate with a Duet. If you are using Windows 10 then you don't need to install the driver if you don't want to, but then the Duet will show up as "USB Serial Device".
- If you print with hot end temperatures above 260C then you will need to add a M143 command in config.g, for example M143 S280 will increase the temperature limit to 280C.
- The recommended web interface is DWC 1.11.
