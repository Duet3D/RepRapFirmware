RepRapFirmware 3.01-RC12
========================

Recommended compatible firmware:
- DuetWebControl 2.1.7
- DuetWiFiServer 1.23 (same as for previous RC)
- Duet Software Framework version 2.2.0 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.01-RC11
- PanelDueFirmware 1.24

Upgrade notes:
- All PanelDue users: the PanelDue connector (or IO_0 on Duet 3) is no longer dedicated to PanelDue, therefore if you connect a PanelDue to this port you must use the following command in config.g to enable it: M575 P1 S1 B57600. You can use baud rates other than 57600, however the IAP files all assume 57600 baud; therefore if you use another baud rate then PanelDue will not display firmware update progress.

Known issues and limitations:
- All boards: Z probe types 1, 2 and 5 are only supported for Z probe 0, and if using Duet 3 only for a probe connected to the main board. All other Z probes must be of type 8 or 9.
- Duet 3: an endstop switch on the main board will not stop movement of a motor on an expansion board unless a motor on the main board is also moving
- Duet 3: when updating the firmware on one or more tool boards or expansion boards, after the updates have completed you must reset the main board or at least run config.g in order to reconfigure the expansion or tool boards
- Duet 3: additional limitations apply to systems with expansion and/or tool boards. See https://duet3d.dozuki.com/Wiki/Duet_3_firmware_configuration_limitations.

New features/changed behaviour:
- The PanelDue connector (or IO_0 on Duet 3) is no longer dedicated to PanelDue. When not used for PanelDue, its two pins are available for use by GPIO, endstops etc. On Duet WiFi/Ethernet/Maestro they are called "urx0" and "utx0".
- Added 'move.virtualEPos' and 'boards[0].uniqueId' to object model
- M486 without parameters now lists the known objects on the build plate
- Previously, if a response for an over-long filename was received by the HTTP server, a "Filename too long" error message was generated and no response to the HTTP command was returned. Now, a 404 response is returned, and a message warning about a possible virus attack is generated unless the filename looks like an OCSP request.
- If a HTTP request is received but insufficient output buffers are available, a HTTP 503 error code is now returned immediately instead of waiting to see if buffers become available and/or discarding response messages. A client receiving a 503 request should call rr_reply to get and flush any pending responses before making any other type of request.

Bug fixes:
- Using the M591 (configure height following mode) command caused a firmware crash and reset in some configurations
- When changes were made to the filament monitor configuration, DCS and DWC were not notified that the corresponding object mode subtree had changed
- Duet WiFi/Ethernet/Maestro: the first time a M575 P1 command was used, the B parameter (baud rate) baud rate in that command was ignored, so the default baud rate of 57600 continued to be used
- Duet WiFi/Ethernet + DueX5: depending on the configuration, the response to a M122 command sent from DWC 2.1.5 might be discarded due to insufficient output buffers
- Duet 3 + SBC: when sending commands from USB or a raw aux port on the Duet and in Marlin response mode, certain commands (e.g. G28, G32) did not return the "ok" response
- Duet 3 + SBC: pauses commanded by filament monitors sometimes caused a system hang or other undesirable behaviour

RepRapFirmware 3.01-RC11
========================

Recommended compatible firmware:
- DuetWebControl 2.1.6
- DuetWiFiServer 1.23 (same as for previous RC)
- Duet Software Framework version 2.1.3 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.01-RC11
- PanelDueFirmware 1.24

Upgrade notes:
- If you use M581 commands you must replace the C parameter by R
- Tool change files are now run even if the axes have not been homed. If you don't want certain parts to run when axes have not been homed, use conditional GCode in the tool change file.
- Legacy 5-point bed compensation is no longer supported
- Duet 3 users: Connector IO_0 is no longer dedicated to PanelDue, therefore if you connect a PanelDue to this port you must use the following command in config.g to enable it: M575 P1 S1 B57600
- Duet 2 users: if using a PanelDue with a baud rate other than 57600, see under Known Issues below.

Known issues and limitations:
- Duet WiFi/Ethernet/Maestro with PanelDue: if you use a PanelDue baud rate other than 57600 then the first M575 command to change the baud rate doesn't work. Workaround: use two identical M575 commands in config.g.
- See also known issues and limitations for 3.01-RC9

New features/changed behaviour:
- The M581 C parameter (condition) is now replaced by R, in order to allow triggering from a C endstop
- Duet 3 only: the IO_0 connector is no longer dedicated to PanelDue
- The PanelDue port (or IO_0 on Duet 3) can now be configure to operate in raw (non-PanelDue) serial mode using command M575 P1 S2 B### where ### is the required baud rate. In this mode it will default to Marlin-type responses.
- HTTP command rr_gcode with no gcode parameter now returns the buffer space, and rr_gcode with an empty gcode parameter no longer adds an empty command to the buffer
- Skew compensation parameters have been added to the object model, in move.compensation.skew
- Duet WiFi/Ethernet: added I2C transaction count and transactions/minute to M122 diagnostics
- Added longest SD card read time (since last M122) to diagnostics
- Longest SD card write time in diagnostics now excludes delays inserted by RRF between retries and the CRC calculation time
- The "unknown value" message when looking up object model values now includes the name of the unknown value
- The object model now reports invisible axes as well as visible ones

Bug fixes:
- Pause and resume sometimes caused a small Z shift if bed compensation was in use and the tool had an X or Y offset
- When an error message occurred in a GCode meta command, the error message included the command number/letter of the previous normal GCode command, which was confusing
- When M584 was used to change the visibility of axes, seqs.move was not updated in the object model
- It was not possible to set up a M581 trigger to trigger on both low->high and high->low transitions of an endstop or input pin
- If you used the M581 C parameter and you had a C axis, it would trigger on changes to the state of the C endstop
- Disabling a trigger response to an input or endstop using the C-1 parameter didn't work
- Duet 3 with SBC only: SPI timeout errors were sometimes reported when a command used a CAN address for which no board was present
- Duet 3 only: Object model value fans[].actualValue was always reported as zero for fans on expansion and tool boards

RepRapFirmware 3.01-RC10
========================

Recommended compatible firmware:
- DuetWebControl 2.1.5
- DuetWiFiServer 1.23 (same as for previous RC)
- Duet Software Framework version 2.1.1 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.01-RC7 (same as for previous RC)
- PanelDueFirmware 1.24

Upgrade notes:
- Duet WiFi, Ethernet and Maestro: a default bed heater is no longer assigned, so you need to use M140 H0 in config.g if you want to replicate the previous behaviour. The online configurator already generates this command automatically when you configure a bed heater. Any M143 H0 command must come later in config.g than the M140 H0 command, because M140 resets the temperature limit for the heater to the default for bed heaters.

Known issues and limitations: as for 3.01-RC9

New features and changed behaviour:
- A default bed heater is never assigned. In previous 3.01 release candidates, heater 0 was the default bed heater in the Duet WiFi/Ethernet and Maestro builds.
- The "laser" field in the rr_status and M408 responses and in state.laserPwm in the object model are now the power that will be used for the next G1, G2 or G3 move instead of the current laser power. This is to allow user interfaces to warn that the laser will turn on as soon as movement starts. New object model field move.current.laserPwm gives the current laser PWM.
- When in laser mode, at the end of a SD file print the laser power for the next move is set to zero automatically even if the job file didn't request it
- When parsing numbers in conditional GCode expressions, excessive numbers of digits no longer give rise to error messages

Bug fixes:
- The minimum extrusion and retraction temperatures in the object model were not reported as zero when cold extrusion was enabled, so the DWC extrude and retract buttons remained greyed-out
- When the minimum extrusion or retraction temperature was changed using M302, the updated values were not reported in the rr_status and M308 responses (this is long-standing bug)
- When the minimum extrusion or retraction temperature was changed using M302, the appropriate object model sequence number was not upated, so DWC and DSF were not aware of the change
- Non-movement commands that needed to be synchronised to the movement queue were sometimes executed too early
- M3 and M5 commands in laser mode were sometimes executed too early, typically by 1 move
- Error messages from G1 and G2/G3 commands were lost
- Literals in conditional GCode were sometimes printed with too few decimal places
- After filament load/unload operations, the filament loaded status was sometimes reported incorrectly to DSF and DWC

RepRapFirmware 3.01-RC9
=======================

Recommended compatible firmware:
- DuetWebControl 2.1.4
- DuetWiFiServer 1.23 (same as for previous RC)
- Duet Software Framework version 2.1.0 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.01-RC7 (same as for previous RC)
- PanelDueFirmware 1.24

Upgrade notes:
- If you were using object model variable 'sensors.inputs' then see the note under "Changed behaviour" below.
- See also the notes for 3.01-RC8 if upgrading from an earlier version

Known issues and limitations:
- All boards: Z probe types 1, 2 and 5 are only supported for Z probe 0, and if using Duet 3 only for a probe connected to the main board. All other Z probes must be of type 8 or 9.
- All boards: if a print is cancelled automatically because of a serious error, the cancellation message is shown on the DWC console but the original error message reporting the problem is often missing
- Duet 3: an endstop switch on the main board will not stop movement of a motor on an expansion board unless a motor on the main board is also moving
- Duet 3: when updating the firmware on one or more tool boards or expansion boards, after the updates have completed you must reset the main board or at least run config.g in order to reconfigure the expansion or tool boards
- Duet 3: additional limitations apply to systems with expansion and/or tool boards. See https://duet3d.dozuki.com/Wiki/Duet_3_firmware_configuration_limitations.

New features/changed behaviour:
- Object model property 'sensors.inputs' is renamed to 'sensors.gpIn'. The 'configured' field of each array element is gone, instead the whole array element is null if that GPIn port has not been configured. The type of the 'value' field has been changed from Boolean to numeric so that we can return analog values in future firmware versions.
- Object model properties 'state.gpOut', 'move.axes[].stepsPerMm', 'move.axes[].microstepping', 'move.extruders[].stepsPerMm' and 'move.extruders[].microstepping' have been added

Bug fixes:
- Fan RPM reporting latency was sometimes greater than 1 second which caused the ATE to report errors. It has been reduced back to under 650ms.
- The fan blip time was longer than had been configured by a random time up to 500ms
- Under some conditions a PanelDue or similar client might not fetch all the waiting responses, leading to responses being delayed or lost, or temporarily running out of output buffers
- Empty responses to commands were being sent to PanelDue instead of being suppressed
- Duet 3 + attached SBC: when file-related commands from PanelDue (e.g. M20, M36) were sent to DCS for processing, the JSON response sent to PanelDue was corrupted after the first 256 bytes. This meant that PanelDue was only able to retrieve file and macro lists when there were no more than about 9 files, and that file information displayed by PanelDue was often incomplete

RepRapFirmware 3.01-RC7
=======================

Recommended compatible firmware:
- DuetWebControl 2.1.2
- DuetWiFiServer 1.23
- Duet Software Framework version 1.3.2 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.01-RC7

Upgrade notes:
- The tool number (P parameter) in a M563 command must now be in the range 0-49
- Duet 3 users with expansion or tool boards must use firmware version 3.01-RC7 on those boards too, otherwise tool/expansion board warnings may be reported as errors, and vice versa. It doesn't matter whether you upgrade the main board firmware before or after the expansion or tool board firmware.
- See also the notes for 3.01-RC6 if upgrading from an earlier version

Known issues and limitations:
- The response to the M122 P1 command is sent as multiple fragments, so the Duet3D ATE doesn't find the board ID in it. This applies to previous 3.01-RC versions too.
- Z probe types 1, 2 and 5 are only supported for Z probe 0, and if using Duet 3 only for a probe connected to the main board. All other Z probes must be of type 8 or 9.
- Duet 3: an endstop switch on the main board will not stop movement of a motor on an expansion board unless a motor on the main board is also moving
- Duet 3: when updating the firmware on one or more tool boards or expansion boards, after the updates have completed you must reset the main board or at least run config.g in order to reconfigure the expansion or tool boards
- Additional limitations apply to Duet 3 systems with expansion and/or tool boards. See https://duet3d.dozuki.com/Wiki/Duet_3_firmware_configuration_limitations.

New features/changed behaviour:
- Experimental support for NeoPixel LED strips has been added for Duet 3. See the M150 X parameter.
- When running a simulation, object model property job.duration is now the simulated time not the actual time
- Added object model property job.lastDuration
- Times in the object model related to the current job are now reported as integers instead of floating point values
- The heater fault messages have been improved (thanks gtj0)
- Recent versions of S3D changed the print time comment when the print time is at least 1 hour but less than 2 hours. RRF now recognises the new format.
- Added restore points to the object model
- Added tools and trackedObjects to the Limits section of the object model
- Increased the maximum number of tracked object to 30 on Duet 3

Bug fixes:
- The object model sequence numbers were not updated when several object model variables were changed, so DSF and DWC did not know they had changed and did not update their copies of them
- M950 J# command with no other parameters reported the incorrect state of the input
- If nested config.g files were used (e.g. because M505 was used to change the system folder) then the effect of M83, G1 Fxxx etc. commands in the nested file were lost
- G30 and G29 commands did not set variable 'result' when some types of error occurred, e.g. when the Z probe was already triggered at the start of the probing move.
- In the object model the speed factor and extrusion factors were only reported to one decimal place
- Other fixes related to communication with DSF

RepRapFirmware 3.01-RC6
=======================

Recommended compatible firmware:
- DuetWebControl 2.1.0
- DuetWiFiServer 1.23
- Duet Software Framework version 1.3.0 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.01-RC5

Upgrade notes:
- Users of Duet 3 with attached SBC should upgrade to DSF 1.3.0 at the same time as upgrading to this release
- If you were using M308 H or L parameters for thermistors attached to a Duet 3 main board, you will need to adjust those values
- Reminders for those upgrading from version 2.x firmware: (1) you cannot upgrade to this release directly from 2.x, you must upgrade to the 3.0 release first; and (2) you will need to make substantial changes to your config.g file.

New features/changed behaviour:
- When pausing a job in CNC mode, the spindle speed is now saved in restore point 1
- The M308 thermistor H and L parameters on Duet 3 main boards have been re-scaled to match the scaling used on Duet 3 expansion and tool boards.
- M308 thermistor H and L parameters are now constrained to the range -128 to +127.

Known issues
- On Duet 3 the values of vin, v12 and mcuTemp in the object model always read as zero for expansion and tool boards. You can see the actual values using M122.

Known limitations:
- Z probe types 1, 2 and 5 are only supported for Z probe 0, and if using Duet 3 only for a probe connected to the main board. All other Z probes must be of type 8 or 9.
- Duet 3: an endstop switch on the main board will not stop movement of a motor on an expansion board unless a motor on the main board is also moving
- Duet 3: when updating the firmware on one or more tool boards or expansion boards, after the updates have completed you must reset the main board or at least run config.g in order to reconfigure the expansion or tool boards
- Additional limitations apply to Duet 3 systems with expansion and/or tool boards. See https://duet3d.dozuki.com/Wiki/Duet_3_firmware_configuration_limitations.

Bug fixes:
- On Duet Maestro, M291 S3 commands initiated from the 12864 display did not work
- When printing a file, the first layer time was reported incorrectly, and when the print completed the total print time reported was incorrect

RepRapFirmware 3.01-RC5
=======================

Recommended compatible firmware:
- DuetWebControl 2.0.7 (or 2.1.0 when available)
- DuetWiFiServer 1.23
- Duet Software Framework version 1.2.5 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.01-RC4

Upgrade notes:
- Reminders for those upgrading from version 2.x firmware: (1) you cannot upgrade to this release directly from 2.x, you must upgrade to the 3.0 release first; and (2) you will need to make substantial changes to your config.g file.

New features/changed behaviour:
- G29 and G30 now accept an optional K parameter to specify which Z probe to use
- On Duet WiFi/Ethernet the stepper driver microstep counters are checked and cleared whenever VIN power is applied or reapplied. This is to combat the phantom stepping that sometimes occurs.
- M486 object cancellation is now implemented. Object labels are read from comments in the file if provided. When printing from SD card the object model has an extra subtree "job.build" to describe the known objects, so that a future version of Duet Web Control will be able to offer an object cancellation interface. Caution: although the code has been designed to handle multi-tool machines, it has not yet been tested with prints that use more than one tool.

Known issues
- On Duet Maestro, M291 S3 commands initiated from the 12864 display do not work
- When a print completes, the total print time reported in the completion message is incorrect
- On Duet 3 the values of vin, v12 and mcuTemp in the object model always read as zero for expansion and tool boards. You can see the actual values using M122.

Known limitations:
- Z probe types 1, 2 and 5 are only supported for Z probe 0, and if using Duet 3 only for a probe connected to the main board. All other Z probes may only be of type 8 or 9.
- Duet 3: an endstop switch on the main board will not stop movement of a motor on an expansion board unless a motor on the main board is also moving
- Duet 3: when updating the firmware on one or more tool boards or expansion boards, after the updates have completed you must reset the main board or at least run config.g in order to reconfigure the expansion or tool boards
- Additional limitations apply to Duet 3 systems with expansion and/or tool boards. See https://duet3d.dozuki.com/Wiki/Duet_3_firmware_configuration_limitations.

Bug fixes:
- On the Duet Maestro 12864 display the speed factor and extrusion factor items were not converted to and from percentages
- Inverting the sense of input pins in analog mode (e.g. for a type 1 Z probe) didn't work

RepRapFirmware 3.01-RC4
=======================

Recommended compatible firmware:
- DuetWebControl 2.1.0 when available, 2.0.7 until then
- DuetWiFiServer 1.23
- Duet Software Framework version 1.2.5 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.01-RC4

Upgrade notes:
- M207 (set firmware retraction parameters) without the new P parameter is applied to all existing tools but not to any tools created after the M207 command. Therefore, make sure that your M207 command is later in config.g than all your M563 tool creation commands.
- Previously, a M563 command with no P parameter could be used to add a constant (specified in the S parameter) to all tool numbers in GCode commands. This is no longer supported. It was only needed when using very old versions of RRF with old versions of slic3r.
- If you have a Z probe that needs to be deployed/retracted, please test that deployment and retraction are working before relying on the Z probe. This is especially important if you are using Duet 3 with attached SBC.

Known issues
- On the Duet Maestro 12864 display, speed factor and extrusion factor display and control doesn't work correctly
- You can only have one Z probe of type 1, 2 or 5 and if using Duet 3 it must be attached to the main board. You can have multiple Z probes of types 8 and 9 including probes attached to Duet 3 expansion and tool boards.
- The G29 and G30 commands only allow the use of Z probe 0
- Duet 3: an endstop switch on the main board will not stop movement of a motor on an expansion board unless a motor on the main board is also moving
- Duet 3: when updating the firmware on one or more tool boards or expansion boards, after the updates have completed you must reset the main board or at least run config.g in order to reconfigure the expansion or tool boards
- Duet 3: the values of vin, v12 and mcuTemp in the object model always read as zero for expansion and tool boards. You can see the actual values using M122.
- Additional limitations apply to Duet 3 systems with expansion and/or tool boards. See https://duet3d.dozuki.com/Wiki/Duet_3_firmware_configuration_limitations.

New features/changed behaviour:
- Parameters in commands received from the SBC attached to a Duet 3 may now be expressions, except in parameters that take a colon-separated list of values. Meta commands ('if', 'while' etc.) are still not supported when using an attached SBC.
- Round brackets in GCode lines are no longer treated as enclosing comments if the machine is not in CNC mode
- Added functions radians(arg) and degrees(arg) which convert the argument from degrees to radians, and from radians to degrees
- M915 now reports the axis or extruder speed that corresponds to the fullsteps/second value of the H parameter
- Added many more object model properties, including inputs[] describing the state of each GPin, volumes[] describing the attached SD cards, and seqs{} to help DSF and UIs know which non-live object model properties have changed
- M207 retraction parameters are now settable on a per-tool basis. The P parameter selects which tool to set. M207 with no P parameter applies the parameters provided to all existing tools. Retraction settings in the object model are moved from extruders[].retraction to tools[].retraction.
- Each Z probe can now have its own deploy and retract files. Z probe number # (where # counts up from zero) looks first for deployprobe#.g and if that is not found it falls back to deployprobe.g. Similarly it uses retractprobe#.g in preference to retractprobe.g.
- The M401 (deploy probe) and M402 (retract probe) commands now accept an optional P parameter which is the Z probe number to deploy mor retract, default 0.
- The daemon GCode task is now enabled even on Duet 3 with attached SBC
- The status no longer shows as Busy when the daemon macro file is running, if no other macro is running or movement in progress
- Increased maximum number of axes on Duet WiFi/Ethernet from 9 to 10

Bug fixes:
- If an array of items in the object model (e.g. heaters, sensors) included null entries because of gaps in the item numbers created, and an object model expression referred to a prioerty of such as null element, the firmware crashed
- If a while-loop was not followed by at least one GCode command or meta command outside the loop before the end of the file, the loop was never executed more than once
- If an extruder-only move specified a feed rate, and the following printing move didn't specify a feed rate because it happened to be the same as the feed rate of the extruder-only move, then the speed factor wouldn't get applied to that move. Likewise if a GCode file used a G1 Fxxx line with no movement in order to dset the feed rate of the following moves, the speed factor would not be applied to those moves.
- M409 incorrectly allowed the '.' to be omitted between the closing square bracket of an index and the following field name
- On Duet 3 in standalone mode, on the Ethernet interface the limit on the number of MDNS services was set too low, so only the 'echo' service was created
- On Duet WiFi, the HTTP rr_model call sometimes caused a reset due to stack overflow
- When running true bed levelling or auto calibration, if all probe points had zero height error then the initial deviation could be reported as 'nan'
- After triggering for the first time, the status of a Z probe on a tool or expansion board that was not a BLTouch was not subsequently updated, so the state remained as triggered, which prevented further probing

RepRapFirmware 3.01-RC3
=======================

Recommended compatible firmware:
- DuetWebControl 2.0.7
- DuetWiFiServer 1.23
- Duet Software Framework 1.2.4.0 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.01-RC3

Upgrade notes:
- If you use a Duet 3 with expansion boards and/or tool boards, you must use firmware 3.01-RC3 on those boards. Otherwise remote endstops and GP input pins will no longer work.
- See the upgrade notes for 3.01-RC2 if you are upgrading from a version earlier than that

New features/changed behaviour
- There is now a daemon GCode channel, which looks for and executes file sys/daemon.g. This can be used to execute regular tasks. If the end of the file is reached, or the file is not found, it delays for 1 second and starts again. This feature is not yet available on Duet 3 with attached SBC.
- Increased maximum stack/macro file depth from 5 to 7
- If the macro stack depth is exceeded, the current macros in the stack are abandoned; and if the macro was called from a GCode print file, that file is abandoned too
- A G4 command will no longer wait for all movement to complete if the input channel executing the G4 has not commanded any motion since it last waited for motion to stop. This is to allow G4 to be used to introduced delays in trigger and deamon GCode files, without causing motion to stop. M400 can still be used to wait for motion to stop.
- Filament monitor types types 4 (rotating magnet + filament presence switch) and 6 (laser + filament presence switch) now provoide object model property 'filamentPresent'. Types 1 and 2 already did.
- Added object model properties 'extruders[].filament' and 'tools[].filament'
- Added support for file runonce.g. If this file is present at startup, it is run after running config.g and activating the network, and then deleted.
- On Duet 3, an emergency stop now tries to stop all CAN-connected expansion boards and tool boards
- On Duet 3, Z probes of types 8 (unfiltered digital) and 9 (BLTouch) connected to expansion boards and tool boards are supported. Other types of Z probe are supported only when they are connected to the main board.
- When a GCode channel locks movement and waits for movement to stop, if there is no movement but moves have been queued, those moves are now executed immediately. Previously there could be a short delay before they were executed.

Bug fixes:
- When homing switches were already triggered at the start of a homing move, sometimes the move queue got stuck, requiring a reboot
- If G10 was used to set the standby temperature of a heater for some tool, and the same heater was an active heater for the current tool, the target temperature would incorrectly be set to the standby value (this was a new bug in 3.01-RC2)
- External SD cards didn't work on Duet Maestro
- The seconds in the last-modified times of files were reported incorrectly (this was a long-standing bug)
- In five-bar SCARA kinematics, the X and Y motors were not treated as continuous rotation axes (thanks bondus)
- When M32 was run a second time, the line numbering was not reset
- Object model variable fans[].actualValue was incorrectly reported to 1 decimal place instead of 2

Other:
- In the Duet 3 build, upgraded Lwip (the TCP/IP network stack) from version 2.0.3 to 2.1.2
- Integrated PRs from ajquick (for building nonstandard Duet 2 configuration without smart driver support) and wilriker (simplification of fan configuration code)

RepRapFirmware 3.01-RC2
=======================

Recommended compatible firmware:
- DuetWebControl 2.0.7
- DuetWiFiServer 1.23
- Duet Software Framework 1.2.4.0 (for Duet 3/Raspberry Pi users)
- Duet 3 expansion board and tool board firmware 3.01-RC2

Upgrade notes:
- If you use M577 or M581 commands, you will need to change them.
- If you use M143 commands with P or X parameters, you will need to change them
- If you use M558 commands with the I parameter, you will need to change them

Changed behaviour:
- The I (invert) parameter of M558 has been removed. If you were using I1 then you will need to invert the pin name instead.
- The parameters to M577 have changed. See  https://duet3d.dozuki.com/Wiki/Gcode?revisionid=HEAD#Section_M577_RepRapFirmware_3_01RC2_and_later.
- The parameters to M581 have changed. See https://duet3d.dozuki.com/Wiki/Gcode?revisionid=HEAD#Section_M581_RepRapFirmware_3_01RC2_and_later.
- The P parameter to M143 now has a different meaning. Also the X (sensor) parameter has been replaced by T. See https://duet3d.dozuki.com/Wiki/Gcode#Section_M143_Maximum_heater_temperature.
- On Duet 3, M143 now works for heaters on expansion boards and tool boards provided that they are running version 3.01-RC2 of their own firmware
- When tuning a heater using M303 H# the S parameter is now mandatory
- The speed factor (M220) is no longer applied to extruder-only moves or to movement commands in system or user macro files

New features:
- General purpose inputs may now be configured using M950 J# C"pin-name". These are used by M577 and M581. Their states may be queried in the object model. On Duet 3 they may refer to pins on expansion boards and tool boards as well as pins on the main board.
- Object model key **heat.sensors** has been moved to **sensors.analog**
- Object model key **sensors.inputs** has been added. This lists the input states of the configured general purpose inputs.
- Object model key **state.previousTool** is added. It is the tool number that was active at the start of the last T command that caused a tool change (or implied T command caused by executing M109), or -1 if no tool was active at that time.
- Object model key **limits** has been added. This gives the maximum number of heaters, fans etc. that the firmware supports. It has the verbose flag set, so it is normally hidden. Send **M409 k"limits" f"v"** to see all the limits.
- Object model key **network.name** has been added. It returns the machine name set by M550.
- In Laser mode, GCode lines with coordinates etc. but no G or M command are now accepted if the most recent command was G0, G1, G2, or G3. This was already the case in CNC mode.

Bug fixes:
- Round-robin scheduling of GCode input sources has been restored so that no channel can monpolise the motion system
- On some Duet 3 boards, axes were not flagged as homed when VIN power was lost but 5V power remained
- When using the M109 command, the firmware did not prevent you from setting temperatures that exceeded the limit set by M143
- The RPM of non-thermostatic fans with tachos wasn't reported continuously

RepRapFirmware 3.01-RC1
=======================

Recommended compatible firmware:
- DuetWebControl 2.07
- DuetWiFiServer 1.23
- Duet Software Framework 1.2.4.0 (for Duet 3/Raspberry Pi users)

Upgrade notes:
- Object model variables move.initialDeviation and move.calibrationDeviation are renamed to move.calibration.initial and move.calibration.final. If you use these variables in bed.g or in other macro files then you will need to update those files.

New features:
- Implemented M952, which is used to set the CAN addresses of tool boards, and exceptionally to alter the CAN bus timing
- Added expansion boards and filament monitors to the object model
- Added move.calibration to the object model and added numFactors as a property of it
- Added move.axes[].babystepping to object model
- On Duet 3, increased maximum heaters per tool from 4 to 8, maximum extruders per tool from 6 to 8, maxumum bed heaters from 9 to 12, maximum total heaters to 32 and maximum extra heater protection instances to 32
- In conditional GCode, literal 'null' is now provided. Object-valued OM elements can be compared for equality/inequality with null.

Bug fixes:
- The M587 command didn't set up the access point password correctly, resulting in "Wrong password" reports when trying to connect to the access point
- if..elif GCode meta commands with multiple elif parts sometimes gave rise to error messages "'elif' did not follow 'if'"
- When G32 true bed levelling failed (for example, because the correction required exceeded the limit), the initial and final deviation were left unchanged. Now thay are both set to the initial deviation measured by probing.
- If the M581 command was used with a T parameter but no P parameter, it reported "missing T parameter".
- The machine coordinates returned to DWC and in the object model were not updated
- The 'abort' command stopped printing but didn't reset the state to Idle
- The 'continue' command was not recognised

RepRapFirmware 3.01-beta3
=========================

Recommended compatible firmware:
- DuetWebControl 2.07
- DuetWiFiServer 1.23
- Duet Software Framework 1.2.4.0 (for Duet 3/Raspberry Pi users)

Upgrade notes:
- Object model property move.meshDeviation is renamed moved.calibration.meshDeviation
- In the M409 command and the rr_model HTTP command, the maximum depth is onw specified by letter d followed by a digit string, instead of just a digit string
- See also upgrade notes for 3.01beta1

Known issues and limitations:
- The new conditional GCode commands and expressions and parameters in GCode commands will not work on Duet 3 with a Raspberry Pi or other SBC attached, until this support has been added to Duet Software Framework
- If you try to report the entire object model using M409, the response may be too long to send and you may get a null response instead. For this reason, M409 without parameters now reports only the top-level property names as if parameter F"1" was used. Use M409 with a key string to drill down into them.
- The M587 command doesn't set up the access point password correctly, resulting in "Wrong password" reports when trying to connect to the access point
- if..elif GCode meta commands with multiple elif parts sometimes give rise to error messages "'elif' did not follow 'if'"
- When G32 true bed levelling fails (for example, because the correction required exceeds the limit), the initial and final deviation are left unchanged

New features and changed behaviour:
- More object model fields have been added

Bug fixes since 3.01-beta2:
- If you tried to access a string-valued field of the object model from a GCode command or meta command, the # operator was always applied to it automatically
- If you used an object model variable beginning with g, m or t in an expression within a regular GCode command, and there was a space or tab character immediately before that letter, then after executing the command the parser would try to interpret that variable name as a new command, resulting in an error message
- On Duet 3 if you executed 2 consecutive G1 H2 moves that involved only motors on expansion boards, the firmware crashed with a memory protection fault
- A fan that hadn't explicitly been configured using M106 with some parameter other than P or S was reported to DWC as being non-controllable

RepRapFirmware 3.01beta2
========================

Recommended compatible firmware:
- DuetWebControl 2.06
- DuetWiFiServer 1.23
- Duet Software Framework 1.2.3.0 (for Duet 3/Raspberry Pi users)

Upgrade notes:
- See upgrade notes for 3.01beta1

Known issues and limitations:
- If you try to access a string-values field of the object model from a GCode command or meta command, the # operator is always applied to it automatically 
- The new conditional GCode commands and expressions and parameters in GCode commands will not work on Duet 3 with a Raspberry Pi or other SBC attached, until this support has been added to Duet Software Framework
- If you try to report the entire object model using M409, the response may be too long to send and you may get a null response instead. For this reason, M409 without parameters now reports only the top-level property names. Use M409 with a key string to drill down into them.

New features and changed behaviour:
- Many new object model fields have been added
- M409 now allows a number to be included in the flags field. This is the maximum depth to which the object model tree will be reported. It defaults to 1 if the key string is empty, otherwise to a large number.

Bug fixes:
- Object model properties move.initialDeviation, move.calibrationDeviation.mean and move.meshDeviation.mean were inaccessible
- Equality between floating point numbers gave the wrong result
- Function calls in GCode meta commands didn't work unless extra brackets were used
- If a GCode line was too long after stripping line numbers, leading white space and comments, the firmware restarted instead of reporting an error
- When an under-voltage event occurs, all axes are now flagged as not homed
- The maximum step rate possible was reduced in earlier RRF3 releases. Some of that loss has been restored.

RepRapFirmware 3.01beta1
========================

Recommended compatible firmware:
- DuetWebControl 2.06
- DuetWiFiServer 1.23
- Duet Software Framework 1.2.2 (for Duet 3/Raspberry Pi users)

Upgrade notes:
- You cannot upgrade a Duet WiFi, Ethernet or Maestro direct to this release from RRF 1.x or 2.x because the firmware binary is too large for the old IAP. You must upgrade to version 3.0 first, then from 3.0 you can upgrade to this release.
- If upgrading a Duet WiFi/Ethernet/Maestro from the 3.0 release, note that default fans are no longer created. Unless your config.g file already used M950 to create the fans explicitly, add commands M950 F0 C"fan0", M950 F1 C"fan1" and M950 F2 C"fan2" to config.g before your M106 commands. Likewise, default endstop switches are not set up, so you will need to set up X and Y endstops (and Z if needed) explicitly, using one M574 line for each, and specifying the port name. Example: M574 X1 S1 P"xstop".

Limitations:
- The new conditional GCode commands and expressions and parameters in GCode commands will not work on Duet 3 with a Raspberry Pi or other SBC attached, until this support has been added to Duet Software Framework

New features and changed behaviour:
- The **if**, **elif**, **else**, **while**, **break**, **continue**, **echo** and **abort** GCode meta commands are implemented, along with expression evaluation. See https://duet3d.dozuki.com/Wiki/GCode_Meta_Commands.
- M409 and a corresponding HTTP call rr_model have been added, to allow parts of the object model to be queried
- Parts of the RepRapFirmware Object Model have been implemented. Values can be retrieved from the OM within GCode command parameters, by M409, and by the new rr_model HTTP command. See https://duet3d.dozuki.com/Wiki/Object_Model_of_RepRapFirmware.
- The MakeDirectory and RenameFile local SD card functions now create the full path recursively if necessary
- The rr_connect message returns additional field "apiLevel":1 if it succeeds. This can be used as an indication that the rr_model command is supported.

Bug fixes:
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
