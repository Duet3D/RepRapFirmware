RepRapFirmware 3.2-beta1 (in preparation
========================

Upgrade notes:
- In GCode commands, numeric parameters of the form "0xdddd" where dddd are hex digits are no longer supported. Use {0xdddd} instead.
- If you are using the M453 command to configure spindle motors and switch the firmware into CNC mode, you will need to change this command because the parameters have changed.
- [Duet 3] if using an attached DotStar LED strip then you now need to use M150 with the X parameter to specify the LED strip type. This is because the default type is now Neopixel.
- If you are using object model field move.workspaceNumber in conditional GCode, you should preferably replace it by (move.workplaceNumber - 1). Note the different name, and that the new workplaceNumber is 0-based (so it can be used to index the workplaceOffsets arrays directly) whereas workspaceNumber was 1-based. We plan to remove workspaceNumber in a future release.
- Default thermistor parameters for all builds of RRF3 are now: T100000 B4725 C7.06e-8. These match the thermistor used by E3D better than the old default, which had B4388 C0. In the unlikely event that your M308 line had a C parameter but no B parameter, you will need to add B4388 to get the prevous behaviour.

New features/changed behaviour:
- Support for connecting the Ethernet adapter socket of Duet Ethernet to SBC instead, using separate firmware build
- A default filename is no longer provided in the M559 and M560 commands, so the P parameter must always be used
- Numeric literals in GCode meta commands and in expressions enclosed by { } can now be in hex (0x prefix) or binary (0b prefix)
- The minimum value for the P parameter of M584 is reduced from 3 to 2 so that the Z axis can be hidden
- M906, M913 and M918 commands now wait for movement to stop, except when they are used in the power fail script
- G29 with no S parameter now runs file sys/mesh.g if it exists; otherwise it behaves like G29 S0 as before
- Drivers number of the form 0.# where # represents one or more decimal digits are now supported even on board that don't support CAN
- The resurrect.g file now records which objects on the build plate have been cancelled
- Duet 3 Mini 5+ WiFi and Ethernet prototype boards v0.2 are now supported
- Added move.workplaceNumber to the object model. This is intended to repace move.workspaceNumber, but is 0-based instead of 1-based.
- Added L (calibration factor) parameter to laser filament monitor configuration
- Increased the number of stack words displayed in the software reset data. The number of wear-levelling slots stored is reduced from 4 to 3.
- Added M584 R parameter to indicate whether newly created axes are continuous rotation axes or not
- M486 now confirms when an object is cancelled or resumed
- Default thermistor parameters for all builds of RRF3 are now T100000 B4725 C7.06e-8. These match the thermistor used by E3D better than the old defaults.
- The parameters for M453 have changed. The frequency parameter is now Q (to match M950) instead of F. You can configure up to 3 ports to control each spindle. See https://duet3d.dozuki.com/Wiki/Gcode#Section_M453_in_RepRapFirmware_3_2_and_later.
- [Duet Maestro] M308 L and H parameters are now supported.
- [Duet Maestro and Duet 3] Added M308 S# H999 for open-circuit thermistor input calibration, and M308 S# L999 for short-circuit calibration. The calibration values are stored in non-volatile memory. See https://duet3d.dozuki.com/Wiki/Calibrating_thermistor_and_PT1000_readings.
- [Duet 3] Added support for second UART (using the IO_1 pins) on Duet 3 MB6HC. New message type (P5) added to M118 command.
- [Duet 3] Default LED strip type is now Neopixel not DotStar
- [Duet 3] M915 with just P and/or axis parameters now reports the belt speed in mm/sec that corresponds to the coolstep threshold
- [Duet 3] Z probing is now abandoned if the probe is remote and cannot be contacted
- [in progress] Support for ST7567-based 12864 displays on Duet Maestro
- [in progress] Support for ST7567-based 12864 displays on Duet WiFi/Ethernet

Object model and expression evaluation changes:
- Spindle current/configured/max RPM were being output to 7 decimal places in object model queries. Now they are reported as integers.
- Added object model variable spindles[].minRpm 
- Support comparing a value of any type that has no literals (DateTime, IPAddress, MAC address, DriverID) with a string
- Support DateTime - DateTime, DateTime + int, DateTime - int
- Support T{expression} commands
- Added random(nn) function

Bug fixes:
- Fixed issue with G29 S1 on Duet 3 with attached SBC causing the print to fail if any if the probe points had been unreachable when the height map was probed
- Fixed a buffer overflow when the number of filaments reported by PrusaSlic3r exceeds the maximum number of supported extruders
- Fixed bug in GetProportionDone that might have caused an incorrect extrusion amount for the first move after restarting a print following a power failure
- The output from M207 without parameters was truncated when there were 4 or more tools
- If a G31 command defined new values in terms of existing G31 values from the object model, then incorrect values could be set due to the new values being computed and stored multiple times
- The M409 response didn't end in newline and was invalid JSON if RRF ran out of output buffers. Now RRF returns {"err":1} if it runs out of buffers, and the response is always terminated by newline to help clients recover from errors.
- Object model variable seqs.spindles was not updated when the configuredRpm of a spindle was changed
- Loading IAP during a firmware upgrade might fail on Duet 2 if a filament monitor or fan tacho was active
- [Duet 3 with attached SBC] When an array parameter (e.g. M92 E value) had more than one element but less than the maximum number, the last element was replicated to fill the array. This was inconsistent with non-SBC behaviour, which only pads the array when a single element is privided.
- [Duet 3] Fixed a bug that caused strange behaviour during homing in some configurations when axis motors were connected to expansion boards
- [Duet 3] When attached to a SBC, M29 commands received locally are now sent to the SBC for processing
- [Duet 3] M915 with just P and/or axis parameters did not report the coolstep threshold (T parameter) correctly

RepRapFirmware 3.1.1
====================

Recommended compatible firmware:
- DuetWebControl 3.1.1
- DuetWiFiServer 1.23 (same as for previous RC)
- Duet Software Framework version 3.1.1 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.1.0
- PanelDueFirmware 1.24

Upgrade notes:
- Available RAM is slightly reduced in this version. This won't affect most users, however if you are running a Duet WiFi or Duet Ethernet with a large number of tools/heaters/sensors etc. then check that the Never Used RAM in the M122 report is comfortably above zero.
- If you are upgrading from a version earlier than 3.1.0, see also the upgrade notes for version 3.1.0.

New features/changed behaviour:
- Maximum GCode line length is increased from 160 to 200 characters
- The number of decimal places reported for axis minimum and maximum limits from the object model is increased from 1 to 2
- Duet 3: M122 now says whether the Duet is running in SBC or standalone mode
- Added object model field 'state.time'. This will be null if the date/time hasn't been set, otherwise a string in ISO format such as 2020-05-10T07:59:31.
- Object model variables which represent a date and time (e.g. file last modified time) are now in ISO format, i.e. "2020-05-10T07:59:31" instead of "2020-05-10 07:59:31". This is for compatibility with DSF and with the way they are returned by M409.

Bug fixes:
- In some configurations the firmware crashed occasionally because the NETWORK task stack overflowed
- Changes to Z probe parameters made by G31, M558 and M851 were not reported to DCS or DWC
- Duet 3 with SBC: G29 (optionally with S1 parameter) rewrote the old height map to the virtual SD card instead of writing the new one
- Duet 3 with SBC: certain commands completed in the wrong order
- G92 commands did not updated seqs.move

RepRapFirmware 3.1.0
====================

Recommended compatible firmware:
- DuetWebControl 3.1.0
- DuetWiFiServer 1.23 (same as for previous RC)
- Duet Software Framework version 3.1.0 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.1.0
- PanelDueFirmware 1.24

Upgrade notes:
- **If you are upgrading from RepRapFirmware 2.x then you will need to make substantial changes to your config.g file.** See https://duet3d.dozuki.com/Wiki/RepRapFirmware_3_overview#Section_Summary_of_what_you_need_to_do_to_convert_your_configuration_and_other_files for details. Also you must upgrade to the 3.0 release first.
- All PanelDue users: the PanelDue connector (or IO_0 on Duet 3) is no longer dedicated to PanelDue, therefore if you connect a PanelDue to this port you must use the following command in config.g to enable it: M575 P1 S1 B57600. You can use baud rates other than 57600, however the IAP files all assume 57600 baud; therefore if you use another baud rate then PanelDue will not display firmware update progress.
- If you use M581 commands you must replace the C parameter by R
- Tool change files are now run even if the axes have not been homed. If you don't want certain parts to run when axes have not been homed, use conditional GCode in the tool change file.
- After a tool change, the user Z coordinate is restored immediately instead of by the end of the next G0 or G1 command. 
- Duet WiFi, Ethernet and Maestro: a default bed heater is no longer assigned, so you need to use M140 H0 in config.g if you want to replicate the previous behaviour. The online configurator already generates this command automatically when you configure a bed heater. Any M143 H0 command must come later in config.g than the M140 H0 command, because M140 resets the temperature limit for the heater to the default for bed heaters.
- The I (invert) parameter of M558 has been removed. If you were using I1 then you will need to invert the pin name instead.
- Legacy 5-point bed compensation is no longer supported
- The tool number (P parameter) in a M563 command must now be in the range 0-49
- Duet 3: If you were using M308 H or L parameters for thermistors attached to a Duet 3 main board, you will need to adjust those values
- The parameters to M577 have changed. See  https://duet3d.dozuki.com/Wiki/Gcode#Section_M577_RepRapFirmware_3_01RC2_and_later.
- The parameters to M581 have changed. See https://duet3d.dozuki.com/Wiki/Gcode#Section_M581_RepRapFirmware_3_01RC2_and_later.
- The P parameter to M143 now has a different meaning. Also the X (sensor) parameter has been replaced by T. See https://duet3d.dozuki.com/Wiki/Gcode#Section_M143_Maximum_heater_temperature.

Known issues and limitations:
- If you execute a tool change at a Z value close to maximum then this could result in at attempt to exceed maximum Z depending on the difference between the old and new Z tool offsets.
- Z probe types 1, 2 and 5 are only supported for Z probe 0, and if using Duet 3 only for a probe connected to the main board. All other Z probes must be of type 8 or 9.
- Duet 3: an endstop switch or Z probe connected to the main board will not stop movement of a motor on an expansion board unless a motor on the main board is also moving
- Duet 3: when updating the firmware on one or more tool boards or expansion boards, after the updates have completed you must reset the main board or at least run config.g in order to reconfigure the expansion or tool boards
- Duet 3: additional limitations apply to systems with expansion and/or tool boards. See https://duet3d.dozuki.com/Wiki/Duet_3_firmware_configuration_limitations.

New features:
- The **if**, **elif**, **else**, **while**, **break**, **continue**, **echo** and **abort** GCode meta commands are implemented, along with expression evaluation. See https://duet3d.dozuki.com/Wiki/GCode_Meta_Commands.
- The RepRapFirmware Object Model has been implemented. See https://duet3d.dozuki.com/Wiki/Object_Model_of_RepRapFirmware.
- M409 and a corresponding HTTP call rr_model have been added, to allow parts of the object model to be queried
- The parameters to G- and M-commands can now be expressions and can use values retrieved from the object model
- M486 object cancellation is now implemented. Object labels are read from comments in the file if provided. When printing from SD card the object model includes subtree "job.build" to describe the known objects, so that a future version of Duet Web Control will be able to offer an object cancellation interface. Caution: although the code has been designed to handle multi-tool machines, it has not yet been tested with prints that use more than one tool.
- The PanelDue connector (or IO_0 on Duet 3) is no longer dedicated to PanelDue. When not used for PanelDue, its two pins are available for use by GPIO, endstops etc. On Duet WiFi/Ethernet/Maestro they are called "urx0" and "utx0".
- The PanelDue port (or IO_0 on Duet 3) can now be configure to operate in raw (non-PanelDue) serial mode using command M575 P1 S2 B### where ### is the required baud rate. In this mode it will default to Marlin-type responses.
- There is now a daemon GCode channel, which looks for and executes file sys/daemon.g. This can be used to execute regular tasks. If the end of the file is reached, or the file is not found, it delays for 1 second and starts again.
- Experimental support for NeoPixel LED strips has been added for Duet 3. See the M150 X parameter.
- Multiple Z probes are supported. M558, G29 and G30 now accept an optional K parameter to specify which Z probe to use. Each Z probe can now have its own deploy and retract files. Z probe number # (where # counts up from zero) looks first for deployprobe#.g and if that is not found it falls back to deployprobe.g. Similarly it uses retractprobe#.g in preference to retractprobe.g. The M401 (deploy probe) and M402 (retract probe) commands now accept an optional P parameter which is the Z probe number to deploy or retract, default 0.
- M207 retraction parameters are now settable on a per-tool basis. The P parameter selects which tool to set. M207 with no P parameter applies the parameters provided to all existing tools. Retraction settings in the object model are moved from extruders[].retraction to tools[].retraction.
- General purpose inputs may now be configured using M950 J# C"pin-name". These are used by M577 and M581. Their states may be queried in the object model. On Duet 3 they may refer to pins on expansion boards and tool boards as well as pins on the main board.
- File runonce.g is now supported. If this file is present at startup, it is run after running config.g and activating the network, and then deleted.
- The MakeDirectory and RenameFile local SD card functions now create the full path recursively if necessary
- Duet 3: M143 now works for heaters on expansion boards and tool boards
- Implemented M952, which is used to set the CAN addresses of tool boards, and exceptionally to alter the CAN bus timing

Changed behaviour (major):
- The M581 C parameter (condition) is now replaced by R, in order to allow triggering from a C endstop
- Duet 3: firmware updates initiated from SBC now send the same USB and PanelDue messages as other types of firmware update
- Tool change files are now run even if the axes have not been homed. If you don't want certain parts to run when axes have not been homed, use conditional GCode in the tool change file.
- After a tool change, the user Z coordinate is restored immediately instead of by the end of the next G0 or G1 command. 
- If a HTTP request is received but insufficient output buffers are available, a HTTP 503 error code is now returned immediately instead of waiting to see if buffers become available and/or discarding response messages. A client receiving a 503 request should call rr_reply to get and flush any pending responses before making any other type of request.
- The "laser" field in the rr_status and M408 responses and in state.laserPwm in the object model are now the power that will be used for the next G1, G2 or G3 move instead of the current laser power. This is to allow user interfaces to warn that the laser will turn on as soon as movement starts. New object model field move.current.laserPwm gives the current laser PWM.
- A default bed heater is never assigned. In previous 3.01 release candidates, heater 0 was the default bed heater in the Duet WiFi/Ethernet and Maestro builds.
- When pausing a job in CNC mode, the spindle speed is now saved in restore point 1
- The M308 thermistor H and L parameters on Duet 3 main boards have been re-scaled to match the scaling used on Duet 3 expansion and tool boards.
- If the macro stack depth is exceeded, the current macros in the stack are abandoned; and if the macro was called from a GCode print file, that file is abandoned too
- Round brackets in GCode lines are no longer treated as enclosing comments if the machine is not in CNC mode
- A G4 command will no longer wait for all movement to complete if the input channel executing the G4 has not commanded any motion since it last waited for motion to stop. This is to allow G4 to be used to introduced delays in trigger and daemon GCode files, without causing motion to stop. M400 can still be used to wait for motion to stop.
- The I (invert) parameter of M558 has been removed
- The parameters to M577 have changed. See  https://duet3d.dozuki.com/Wiki/Gcode#Section_M577_RepRapFirmware_3_01RC2_and_later.
- The parameters to M581 have changed. See https://duet3d.dozuki.com/Wiki/Gcode#Section_M581_RepRapFirmware_3_01RC2_and_later.
- The P parameter to M143 now has a different meaning. Also the X (sensor) parameter has been replaced by T. See https://duet3d.dozuki.com/Wiki/Gcode#Section_M143_Maximum_heater_temperature.
Duet 3: Z probes of types 8 (unfiltered digital) and 9 (BLTouch) connected to expansion boards and tool boards are supported
- When tuning a heater using M303 H# the S parameter is now mandatory
- The rr_connect message returns additional field "apiLevel":1 if it succeeds. This can be used as an indication that the rr_model command is supported.
- In Laser mode, GCode lines with coordinates etc. but no G or M command are now accepted if the most recent command was G0, G1, G2, or G3. This was already the case in CNC mode.
- The speed factor (M220) is no longer applied to extruder-only moves or to movement commands in system or user macro files

Changed behaviour (minor):
- Previously, if a response for an over-long filename was received by the HTTP server, a "Filename too long" error message was generated and no response to the HTTP command was returned. Now, a 404 response is returned, and a message warning about a possible virus attack is generated unless the filename looks like an OCSP request.
- M575 P0 indicates whether the USB interface is in the connected state
- M575 P1 says if the aux channel is disabled
- M122 Telnet responder line now gives the number of Telnet sessions
- On Duet WiFi/Ethernet the stepper driver microstep counters are checked and cleared whenever VIN power is applied or reapplied. This is to combat the phantom stepping that sometimes occurs.
- HTTP command rr_gcode with no gcode parameter now returns the buffer space, and rr_gcode with an empty gcode parameter no longer adds an empty command to the buffer
- Duet WiFi/Ethernet: added I2C transaction count and transactions/minute to M122 diagnostics
- Added longest SD card read time (since last M122) to diagnostics
- Longest SD card write time in diagnostics now excludes delays inserted by RRF between retries and the CRC calculation time
- When in laser mode, at the end of a SD file print the laser power for the next move is set to zero automatically even if the job file didn't request it
- The heater fault messages have been improved (thanks gtj0)
- Recent versions of S3D changed the print time comment when the print time is at least 1 hour but less than 2 hours. RRF now recognises the new format.
- M308 thermistor H and L parameters are now constrained to the range -128 to +127.
- M915 now reports the axis or extruder speed that corresponds to the fullsteps/second value of the H parameter
- Increased maximum number of axes on Duet WiFi/Ethernet from 9 to 10
- Increased maximum stack/macro file depth from 5 to 7
- Duet 3: an emergency stop now tries to stop all CAN-connected expansion boards and tool boards
- When a GCode channel locks movement and waits for movement to stop, if there is no movement but moves have been queued, those moves are now executed immediately. Previously there could be a short delay before they were executed.
- On Duet 3, increased maximum heaters per tool from 4 to 8, maximum extruders per tool from 6 to 8, maximum bed heaters from 9 to 12, maximum total heaters to 32 and maximum extra heater protection instances to 32

Bug fixes:
- Duet 3: the default value written to the TMC5160 PWMCONF register did not match the default value for that chip
- Duet 3 with SBC: Fixed a very small memory leak when DCS sent a height map to RRF
- Duet 3 with SBC: Fixed loss of output buffers when the USB interface became disconnected and generic messages were generated
- Duet 3 with SBC: Array parameters in G- and M-commands were not bounds checked
- Duet 3 with SBC: File-related commands sent from USB ior from a PanelDue connected to the Duet now work (if sending from a PanelDue then it must have firmware 1.24 or later)
- Using the M591 (configure height following mode) command caused a firmware crash and reset in some configurations
- Duet WiFi/Ethernet + DueX5: depending on the configuration, the response to a M122 command sent from DWC 2.1.5 might be discarded due to insufficient output buffers
- Duet 3 + SBC: when sending commands from USB or a raw aux port on the Duet and in Marlin response mode, certain commands (e.g. G28, G32) did not return the "ok" response
- Duet 3 + SBC: pauses commanded by filament monitors sometimes caused a system hang or other undesirable behaviour
- Pause and resume sometimes caused a small Z shift if bed compensation was in use and the tool had an X or Y offset
- If you used the M581 C parameter and you had a C axis, it would trigger on changes to the state of the C endstop
- Disabling a trigger response to an input or endstop using the C-1 parameter didn't work
- Duet 3 with SBC: SPI timeout errors were sometimes reported when a command used a CAN address for which no board was present
- When the minimum extrusion or retraction temperature was changed using M302, the updated values were not reported in the rr_status and M308 responses (this is long-standing bug)
- Non-movement commands that needed to be synchronised to the movement queue were sometimes executed too early
- M3 and M5 commands in laser mode were sometimes executed too early, typically by 1 move
- Under some conditions a PanelDue or similar client might not fetch all the waiting responses, leading to responses being delayed or lost, or temporarily running out of output buffers
- Empty responses to commands were being sent to PanelDue instead of being suppressed
- If nested config.g files were used (e.g. because M505 was used to change the system folder) then the effect of M83, G1 Fxxx etc. commands in the nested file were lost
- If an extruder-only move specified a feed rate, and the following printing move didn't specify a feed rate because it happened to be the same as the feed rate of the extruder-only move, then the speed factor wouldn't get applied to that move. Likewise if a GCode file used a G1 Fxxx line with no movement in order to dset the feed rate of the following moves, the speed factor would not be applied to those moves.
- M409 incorrectly allowed the '.' to be omitted between the closing square bracket of an index and the following field name
- On Duet 3 in standalone mode, on the Ethernet interface the limit on the number of MDNS services was set too low, so only the 'echo' service was created
- The seconds in the last-modified times of files were reported incorrectly (this was a long-standing bug)
- In five-bar SCARA kinematics, the X and Y motors were not treated as continuous rotation axes (thanks bondus)
- When M32 was run a second time, the line numbering was not reset
- Round-robin scheduling of GCode input sources has been restored so that no channel can monopolise the motion system
- On some Duet 3 boards, axes were not flagged as homed when VIN power was lost but 5V power remained
- When using the M109 command, the firmware did not prevent you from setting temperatures that exceeded the limit set by M143
- The RPM of non-thermostatic fans with tachos wasn't reported continuously
- When an under-voltage event occurs, all axes are now flagged as not homed
- The maximum step rate possible was reduced in earlier RRF3 releases. Some of that loss has been restored.
- When the C (temperature coefficient) parameter was used in the G31 command, if the temperature could not be read from the sensor specified in the H parameter then the error message was not clear; and it didn't allow time for the sensor to become ready in case it had only just been configured.
- The M917 command didn't work on Duet 3 and Duet Maestro.
- Fixed two instances of possible 1-character buffer overflow in class OutputBuffer
- If no heaters were configured, one spurious heater was reported in the status response
- On delta printers, M564 S0 didn't allow movement outside the print radius defined in M665
- On Duet 3 with attached SBC, when a job was paused and then cancelled, a spurious move sometimes occurred

RepRapFirmware 3.0
==================

Recommended compatible firmware:
- Duet Web Control 2.0.4
- DuetWiFiServer 1.23
- DuetSoftwareFramework 1.2.2.0
- Duet3Firmware_EXP3HC 3.0

Upgrade notes:
- **If you are upgrading from RepRapFirmware 2.x then you will need to make substantial changes to your config.g file.** See https://duet3d.dozuki.com/Wiki/RepRapFirmware_3_overview#Section_Summary_of_what_you_need_to_do_to_convert_your_configuration_and_other_files for details.
- Duet Maestro users: the M917 command does not work in this release. If you use M917 in config.g or any other files, we suggest you comment it out all M917 commands until you can upgrade to version 3.01.

The remaining items affect users of RepRapFirmware 3.0 beta (but not RC) versions:
- Endstop type S0 (active low switch) is no longer supported in M574 commands. Instead, use type S1 and invert the input by prefixing the pin name with '!'.
- If you are using Duet 3 expansion or tool boards, you must upgrade those to 3.0 too
- You should also upload the new IAP file for your system. You will need it when upgrading firmware in future. These files are called Duet2CombinedIAP.bin, DuetMaestroIAP.bin, Duet3_SBCiap_MB6HC.bin (for Duet 3+SBC) and Duet3_SDiap.bin (for Duet 3 standalone systems). Leave the old IAP files on your system, they have different names and you will need them again if you downgrade to older firmware.
- Duet 3 users: there is no longer a default bed heater. To use Heater 0 as the bed heater, put M140 H0 in config.g (later in config.g than your M950 H0 command).

Known limitations:
- Duet 3 users: connector IO_0 of the MB6HC board is currently reserved for PanelDue. We recommend that you do not use it for any other purpose.
- Duet 3 users: support for expansion boards has some limitations. See https://duet3d.dozuki.com/Wiki/Duet_3_firmware_configuration_limitations for details.
- Duet Maestro and Duet 3 users: the M917 command does not work properly and must not be used.

Bug fixes since 3.0RC2:
- Fixed issue where a missing start.g could disrupt the G-code flow with an attached SBC
- When RRF requested a macro file but had to wait for the SBC to connect, the final code replies to DSF were omitted

RepRapFirmware 3.0RC2
=====================

Recommended compatible firmware:
- Duet Web Control 2.0.4
- DuetWiFiServer 1.23
- DuetSoftwareFramework 1.2.1.0
- Duet3Firmware_EXP3HC 3.0RC2
- Duet3Firmware_Tool1LC 3.0RC2

Known issues:
- As for RRF 3.0RC1

Upgrade notes:
- As for RRF 3.0RC1

Feature changes:
- Increased maximum number of triggers on Duet 3 from 16 to 32
- Increased maximum number of heaters on Duet 3 to 16
- Increased maximum number of extra heater protections on Duet 3 to 16
- Increased maximum number of fans on Duet 3 to 16

Bug fixes:
- Fix for M584 sent via SPI to Duet 3 when the first parameter is an array
- Fix for analog Z probing when Z probe reports "near" from the start of the move
- Fix for M25 while tool changing is is progress
- Fix for unexpected diagonal moves during tool changes
- Use of M950 to configure heater numbers greater than 5 on expansion boards caused sensors to disappear
- The response to M308 with just a S parameter didn't include the sensor name or last reading if the sensor was connected to a Duet 3 expansion board
- The G31 response now displays trigger height to 3 decimal places instead of 2
- When printing from file on a Duet 3 with attached Raspberry Pi, under some conditions an attempt to retrieve the file position caused RRF to crash

RepRapFirmware 3.0RC1
=====================

Recommended compatible firmware:
- Duet Web Control 2.0.4
- DuetWiFiServer 1.23
- DuetSoftwareFramework 1.2
- Duet3Firmware_EXP3HC 3.0RC1
- Duet3Firmware_Tool1LC 3.0RC1

Known issues:
- An endstop switch connected to a Duet 3 main board may only be used to home an axis if at least one motor controlling that axis is driven by the main board
- Various Duet 3 expansion board features are not fully implemented. See the RRF3 Limitations document on the wiki.
- After installing new expansion or tool board firmware or resetting an expansion or tool board, you must reset the main board or at least run config.g using M98. Otherwise the expansion/tool board configuration will not match the settings expected by the main board.

Upgrade notes:
- Endstop type S0 (active low switch) is no longer supported in M574 commands. Instead, use type S1 and invert the input by prefixing the pin name with '!'.
- If you are using Duet 3 expansion or tool boards, you must upgrade those to 3.0RC1 too
- Duet 3+SBC users must use DSF 1.1.0.5 or a compatible later version with this version of RRF
- You should also upload the new IAP file for your system. You will need it when upgrading firmware in future. These files are called Duet2CombinedIAP.bin, DuetMaestroIAP.bin, Duet3_SBCiap_MB6HC.bin (for Duet 3+SBC) and Duet3_SDiap.bin (for Duet 3 standalone systems). You can leave the old IAP files on your system, they have different names and you will need them if you downgrade to earlier firmware.
- Duet 3 users: there is no longer a default bed heater. To use Heater 0 as the bed heater, put M140 H0 in config.g (later in config.g than your M950 H0 command).

Feature changes since beta 12:
- Duet 3 only: Switch-type endstops connected to expansion boards are supported
- Current position is no longer shown for pulse-type filament monitors, because it was meaningless and nearly always zero
- Calibration data for pulse-type filament monitors is no longer displayed by M122 (same as for laser and magnetic filament monitors). Use M591 to report the calibration data.
- Max bed heaters increased to 9 on Duet 3, 2 on Duet Meastro (still 4 on Duet WiFi/Ethernet)
- Max chamber heaters increased to 4 on Duet 3 and on Duet WiFi/Ethernet
- CRC calculation has been speeded up, which improves the speed of file uploads in standalone mode when CRC checking is enabled in DWC
- G1 H1 E moves (stopping on motor stall) are now implemented
- rr_config and M408 S5 responses now include field "sysdir" which is the system files folder set using M505
- M950 P, M950 S, M42 and M280 are implemented on expansion boards
- B parameter added to M408 to query expansion boards (for expansion board ATE)
- M122 P parameter is passed to the expansion board if the B parameter is present and selects an expansion board (for ATE)

Bug fixes:
- Duet 3 only: Files uploaded in standalone modes were frequently corruption during uploading, resulting in CRC mismatches reported
- If a print that was sliced using absolute extrusion mode was resurrected, unwanted extrusion occurred just before the print was resumed
- Bed compensation did not take account of the XY offset of the printing nozzle from the head reference point
- When using SCARA kinematics the calculation of the minimum achievable radius was incorrect. Depending on the B parameter of the M667 command, this could result in spurious "Intermediate position unreachable" errors, or non-extruding G1 moves being turned into G0 moves.
- A badly-formed GCode file that returned the layer height or object height as nan or inf caused DWC to disconnect because of a JSON parse failure
- M579 scale factors were not applied correctly to G2 and G3 arc moves
- M119 crashed if an axis had no endstop
- Filament handling didn't work on Duet 3+SBC
- If a homing move involved only motors connected to Duet 3 expansion boards and the corresponding endstop was already triggered, the homing move started anyway and didn't stop
- Stall detect homing works properly
- If an attempt to create a switch-type endstop using M574 failed because the specified pin wasn't available, this resulted in a partically-configured switch endstop

RepRapFirmware 3.0beta12
========================
Recommended compatible firmware:
- Duet Web Control 2.0.4
- DuetWiFiServer 1.23
- DuetSoftwareFramework 1.1.0.5

Feature changes since beta 11:
- Duet 3 0.6 and 1.0: pin io8.out is no longer PWM-capable because of a resource clash. If you have connected a BLTouch to IO8, please move it to IO7 and adjust config.g accordingly.
- Duet 3 all revisions: improved the temperature reading accuracy at low temperatures for thermistors connected to the main board (typically it used to read several degrees low)
- Duet 3 all revisions: M308 L and H parameters are supported for thermistors and PT1000 sensors connected to the main board. They should only be used if you have suitable fixed resistors to use for calibration.
- DHT sensors are now supported (thanks wilriker)
- M115 P parameter is now only implemented on those builds that support multiple board types, and only when running config.g at startup
- M500 now accepts optional P10 parameter to force it to save the G10 tool offsets (thanks wilriker). It can be combined with P31 by using M500 P10:31.

Bug fixes:
- Duet 3 0.6 and 1.0: PWM output on io4.out and io5.out now works
- Duet 3 0.6 and 1.0: pin io6.out was incorrectly marked as PWM-capable
- Duet 3 all revisions: Driver 5 on main board didn't generate temperature warnings
- Duet 3 all revisions: updating expansion boards didn't work when running in standalone mode if the USB port was connected to a PC but no terminal emulator was running
- Duet 3 all revisions: the drivers are no longer shut down when VIN exceeds 29V
- In beta 11, control of BLTouch probes was unreliable because of glitches on the control output pin
- Homing sometimes didn't work, especially when endstops switches were already triggered at the start of the homing move
- M308 S# with no other parameters didn't report the sensor details
Known issues:

- Duet 3 all revisions: file uploading via the local Ethernet port is unreliable (this is the case in previous firmware versions too). To guard against this, always enable CRC checking in DWC 2.0.4.
- Extruder stall detection (G1 H1 E moves) is not implemented
- Stall detection endstops don't work properly if you home multiple aes at the same time
