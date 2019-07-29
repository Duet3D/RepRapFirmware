Summary of important changes in recent versions
===============================================

Version 2.04RC1
============
Compatible files:
- DuetWiFiServer 1.23
- DuetWebControl 1.22.6 or 2.0.0-RC6 or 2.0.0-RC7

Upgrade notes:
- The P parameter of the G29 S0 (or plain G29) command has been withdrawn. See below under "changed behaviour".

Feature improvements/changed behaviour:
- The P parameter of the G29 S0 (or plain G29) command has been withdrawn, because it didn't work when deployprobe.g and retractprobe.g files were used and wasn't easy to fix without wasting memory. A new subfunction G29 S3 P"name.csv" has been added to facilitate saving the height map file under a different name. It behaves the same as M374 P"name.csv".
- M118 now appends '\n' to the message text except when the destination is http
- G31 with no parameters now reports the G31 parameters of the current Z probe as well as the current reading
- Support for pulse-generating filament sensors has been improved for the case that the sensor produces a high number of pulses per mm of filament movement

Bug fixes:
- When auto delta calibration adjusted the delta radius and/or the diagonal rod length, it made an incorrect adjustment to the homed height
- On a delta printer, if multiple rod lengths are specified in the M665 command and the first 3 rod lengths were not equal to each other, this resulted in incorrect motion
- M557 with a P parameter but no XY or R parameters now reports an error
- Attempts to jog axes 0.05mm beyond the limits set by M208 alternately succeeded/returned the axis to the limit

Internal changes:
- Changes for compatibility with latest versions of CoreNG and RRFLibraries projects

Version 2.03
============
Compatible files:
- DuetWiFiServer 1.23
- DuetWebControl 1.22.6 or 2.0.0-RC6

Upgrade notes:
- Restore points (created by G60 and created automatically at the start of a pause or a tool change) now have their coordinates stored independently of any workplace offsets. So if you create a restore point and then change the workplace offsets, when you go back to the restore point it will go back to the same machine position regardless of the change in workplace offsets.
- Tool changers, IDEX printers and similar using tfree#.g and tpost#.g files: tool offsets are now applied within the tfree#.g and tpost#.g macros (but not in the tpre#.g file because no tool is selected at that point).
- DueX2 and DueX5 users: if you have been experiencing high I2C error counts, then in the past this usually led to the machine printing very slowly when the errors started occurring. Changes to the I2C drivers should allow the machine to recover from the error in most cases. However, if it does not recover then the machine will most likely continue to run as normal, except that the states of endstops on the DueX will not be read correctly and commands to change settings of fans on the DueX won't work. So watch out for these different symptoms.
- Duet Maestro users with a 12864 display may need to make minor changes to their menu files to correct for changes in spacing and automatic insertion of % characters after certain values e.g. fan speed
- Laser mode: for safety, the G1 S parameter is no longer sticky by default. You can make it sticky by adding parameter S1 to the M452 command.
- If you have a CoreXY or other Core architecture printer, and you were using any axis factor parameters in your M667 command in config.g, those parameters are no longer supported. You will need to use M669 matrix parameters instead.
- nanoDLP users: an empty macro file M650.g must be created in /sys, and file peel-move.g must be renamed to M651.g
- The M135 command is no longer supported, but AFAIK nobody used it
- The Duet 06/085 build no longer supports ".local" network addressing. It has been disabled because of quality issues with the MDNS library.

Known issues:
- When auto delta calibration adjusts the delta radius and/or the diagonal rod length, it makes an incorrect adjustment to the homed height. This will be fixed in a forthcoming update. Meanwhile, run a second auto calibration cycle to correct the homed height.
- On a delta printer, if multiple rod lengths are specified in the M665 command and the first 3 rod lengths are not equal to each other, this results in incorrect motion
- M557 with a P parameter but no XY or R parameters should report an error, but doesn't
- The P parameter of the G29 S0 command is ignored if there is a deployprobe.g file and/or a retractprobe.g file

Feature improvements/changed behaviour:
- Added M566 P parameter to control the jerk policy
- The M114 response now includes the virtual extruder position (for Octoprint) and the bed compensation amount at the current position
- In SCARA printers/laser cutters (not CNC machines), if we can't do a coordinated travel move due to unreachable intermediate positions, try an uncoordinated one
- Duet Maestro build now supports laser cutters
- The Move section of the diagnostics output now includes the Z shift
- The Move section of the diagnostics output only includes bed probe heights if 3/4/5-point bed compensation is in use
- M1 no longer disables drives (you can do that in sleep.g)
- M0/M1 with a stop.g/sleep.g file no longer deselects the tool, it just turns off tool heaters unless the H1 parameter is present
- After grid probing, the min/max errors are printed as well as mean and standard deviation, and recorded in the height map file
- Implemented M675 (thanks wilriker)
- If you run mesh bed probing and there is a substantial height offset, a warning is generated
- If you load a mesh and the Z datum has not been set by probing and there is a Z probe, a warning is generated
- Delta printer limit checking has been rewritten to be more accurate
- In CoreNG, increment I2C reset count when resetting the I2C system
- Recognise filament usage comment in Prusa slicer
- G53 is now available even when workplace coordinates not supported in build (cancels tool offsets)
- When using a mixing extruder, the feed rate is now scaled in proportion to total mix, for serial extruder drives etc.
- Allow M203 max speeds lower than 1mm/sec
- Enable laser in Duet085 build
- M584 formatting improvement when a has no extruders
- M563 P# formatting improvements when the tool has no heaters or no drives
- Added extra diagnostics for when a filename is too long
- When a DueX is attached, a separate task is now used to read the states of DueX endstop inputs when they change. This should give much lower latency.
- Added 12864 menu items 534-539
- The 12864 display no longer automatically adds a space column after each item, except for left-justified text items without an explicit width
- The 12864 display system automatically appends a % character to value/alter items that are normally expressed in percent, e.g. fan speeds, print speed, extrusion factors
- M302 now waits for movement to stop
- M291 now unlocks movement if it is locked, so that PanelDue or DWC can be used to jog axes if M291 was invoked from another input stream
- The status response for DWC and returned by M408 S2 now includes the workplace coordinate system number with variable name "wpl"
- On the Duet Maestro, stepper driver open load detection is now disabled when the driver is operating in stealthChop mode
- Prints can now be baby stepped, paused and cancelled while they are waiting for temperatures to be reached
- Increased maximum number of triggers from 10 to 16
- Increased number of output buffers on Duet WiFi/Ethernet/Maestro from 20 to 24
- M505 (set SD card system folder) is now implemented
- M470 and M471 commands are now implemented (thanks wilriker)
- On deltas with more than 3 towers, M666 can be used to set endstop corrections for all towers. The additional towers must be named UVW.
- The M291 command no longer locks movement if no axis jog buttons in that command are enabled (https://forum.duet3d.com/topic/8988/wishlist-interrupt-tool-change-from-a-trigger/16)
- Tool offsets are applied within the tpre#.g and tpost#.g macros
- One the Duet 06/085 it is now possible to define 2 additional fans. The M106 A parameter can be used to map them to unused heater channels.
- When changing pressure advance, movement is stopped momentarily to avoid sudden under- or over-extrusion (https://forum.duet3d.com/topic/9140/extruder-behaves-strangle-when-disabling-pressure-advance/10)
- Baby stepping axes other than Z is now supported, however the current implementation doesn't apply it until all moves already in the pipeline have been completed (https://forum.duet3d.com/topic/8015/change-z-babystepping-to-y-babystepping-for-belt-printers/14)
- G20 and G21 inch/mm unit setting commands are now applied per input channel and the settings are pushed/popped
- If you try to use a G- or M-code that the firmware doesn't recognise, it will try to run a macro file of that name. For example, command M650 tries to run /sys/M650.g.
- M650 and M651 are no longer recognised, unless you create macros M650.g and M651.g
- The heater tuning timeouts have been increased to 30 minutes for bed and chamber heaters (was 20 minutes) and 7 minutes for extruder heaters (was 5 minutes)
- M203 Inn sets minimum movement speed
- M203 now reports speeds in mm/sec
- M208 reports error if min >= max
- Allow hex inputs anywhere in G/M code parameters where an unsigned value is required
- Requested fan speed is now scaled by the M106 X parameter
- M111 Sn for n != 0 now requires a P parameter, to reduce issues caused by Repitier Host using the M111 command for other purposes
- Increased temperature sampling rate from 2 to 4 samples/sec
- Increased the allowed number of consecutive temperature reading errors from 5 to 8
- Support for generalised Cartesian/Core kinematics inc. MarkForged and variants thereof
- Support for different rod lengths on each tower of a delta printer
- Support for additional towers on delta printers (up to 6 in total)
- When the first move on a machine with CoreXY or similar kinematics is a diagonal move, all relevant motors are now enabled, in order to lock them even if only one of them needs to move
- G2/G3 commands using the R (radius) parameter instead of IJ arc centre parameters are now supported
- When dynamic acceleration adjustment (DAA) is activated, if the maximum acceleration was configured lower than the required value, DAA was not applied. Now it checks to see whether the acceleration can be halved to still get some benefit from DAA.
- M572 and M221 with no extruder drive number now sets all extruders used by the current tool, https://forum.duet3d.com/topic/8444/setting-pressure-advance-in-filament-file
- On the 12864 display, the default column for an item is now 1 extra pixel past the end of the previous item, so as to leave a thin space between them
- On the 12864 display, when displaying print files, filenames beginning with '.' are no longer displayed
- The SPI clock frequency of the 12864 display can now be configured using the M918 F parameter
- Slicer PE changed the comment string it uses to provide estimated print times. RRF now recognises the new format as well as the old one, https://forum.duet3d.com/topic/8440/rrf-2-02-slic3r-pe-1-41-2-filament-used-and-print-times-wrong
- Reworked some of the filament monitor code to try to reduce the tolerance needed when using 'good' filaments, also added support for  experimental v2 laser filament sensor firmware
- Disabled mdns in legacy Duets because of code quality issues causing reboots, https://forum.duet3d.com/topic/8352/duet-0-6-randomly-reboots/5
- rr_fileinfo and M36 with no filename now include estimated print time and simulation time in the response for DWC2
- In CNC and laser mode the user Z coordinate is updated after a tool change, https://forum.duet3d.com/topic/8181/tool-offset-honored-but-not-displayed-correctly
- On SCARA and delta printers, geometric limits are now applied even when not applying M208 limits due to use of M564 S0
- New S-3 function for G30 command. G30 S-3 probes the bed and sets the Z probe trigger height to the stopped height.
- M92 command now includes an optional S parameter to specify the microstepping that the steps/mm is quotes at. If the actual microstepping in use is different, the specified steps/mm will be adjusted accordingly (thanks wikriker).
- M585 command L parameter for inverting probe logic level is supported (thanks chrishamm)
- The M408 S2 and http status responses now include the bed standby temperature (thanks gtjoseph)
- The M669 parameters to define SCARA kinematics now include an R (minimum radius) parameter, to handle machines for which the minimum available radius is sometimes higher than the radius when the distal axis is homed
- G17 is implemented (it does nothing), and G18/G19 report an error
- The segment length used by G2/G3 now depends on both the radius and the speed of the move, not just the radius as in RRF 2.02. So small segment lengths are used when doing CNC milling at low speeds.
- If a print is paused and then cancelled while the printer is still heating up, heating is cancelled
- Added M122 P105 subfunction to display the sizes of various objects allocated by RRF
- Removed M135 command

Bug fixes:
- When disabling HTTP protocol or disabling the network, release any output buffers and set the number of sessions to zero. Similarly for Telnet.
- After running stop.g ensure all moves are finished before setting motors to idle current
- When downloading a .zip file don't set content-encoding to gzip
- FTP responder: initialise haveCompleteLine (thanks gtjoseph)
- On SCARA printers, when attempting to move beyond arm limits, the XY coordinates computed from the adjusted motor endpoints were incorrect 
- If Z probe averaging mode was being used when mesh probing, an incorrect Z shift could be applied
- M0 H1 turned heaters off after running stop.g instead of leaving them on
- Independent leadscrew moves for true bed levelling didn't work when driver numbers >= MaxAxes were used to drive Z motors
- Added missing newline at end of some error messages
- M585 L parameter was not working
- In resume.g a G1 R command went to the wrong coordinates if workplace coordinate offsets were being used
- Homing files when workplace coordinate offsets were active cause other axes to move
- Resurrect.g now works when workplace offsets are used, and it restores the current workplace # and all workplace offsets
- In resurrect.g, the current tool is now selected after running resurrect-prologue, and its tpre and tpost files are run (for tool changer)
- In resurrect.g, fan speeds are now set up after the tool is selected so that the mapped fan speed is correct
- In resurrect.g, when setting the last known position with G92 prior to calling resurrect-prologue.g, allow for tool offsets
- Babystepping could cause an incorrect Z coordinate to be set under some conditions
- When I2C timeouts occurred, no attempt was made to reset the I2C controller before the next I2C transaction attempt. The I2C driver now resets the controller on the Duet MCU after an I2C error, also it retries the failed transaction twice.
- Some PWM channels didn't work correctly in the Due06/085 build (this was also fixed in temporary release 2.03RC1+1).
- In resurrect.g file the M290 command now commands absolute babystepping, the filename in M23 command is enclosed in double quote marks, and the inches/mm setting is restored
- The W5500 chip could not be reset on Duet Maestro
- M109 did not run the tool change files if no tool was active initially
- If a print finishes or is cancelled when Z hop is active because of a G10 command without a subsequent G11, the Z hop is cancelled (but not the associated retraction)
- Blank lines in 12864 display menu files are now ignored
- Visibility attributes were not correctly applied to 12864 display value, alter and image menu items
- Fixed a couple of issues with the 12864 display "files" menu item when the file path refers to SD card 1. The SD card is mounted automatically if it isn't already mounted.
- On the 12864 display on the Duet Maestro, if the gcodes folder contained any files with names starting '.' or '..' (not counting the current and parent directories), then when selecting a file to print the wrong file might be selected
- HttpResponder fix to better handle low buffer space situations (thanks sdavi)
- If axis scaling and baby stepping were both used, then tool offsets and workplace coordinate offsets were not correctly accounted for when converting machine position to user position
- In laser and CNC mode, limit checking was being applied to move types other than zero (https://forum.duet3d.com/topic/9203/laser-mode-endstop-limit-and-homing/4)
- In laser and CNC modes, under certain conditions (in particular, when using non-rational steps/mm values) out-of-limits messages might be reported when attempting movement after homing (https://forum.duet3d.com/topic/9363/m453-issue-after-homing)
- Running G30 P0 X0 Y0 Z-9999 S-1 sometimes reported the deviation as 'nan' instead of 0.0
- In earlier 2.03beta versions, if you auto-calibrated a delta printer then the motor positions were slightly wrong afterwards until you homed the printer
- On delta printers with more than 3 towers, the homed height calculation for additional towers used the Y coordinate incorrectly.
- If you used M141 to define a chamber heater using a heater number other than 0 or 1, the firmware crashed
- Fixed 12864 display scrolling when the folder included filenames starting with '.'
- Fixed remaining M105 status response received when M997 S1 sent from USB
- Fixed firmware update messages getting erased on PanelDue during M997 S1
- M408 Sn was hanging for N>5
- Fixed leadscrew adjustment bug introduced in 2.03beta1
- Commands such as G1 X1E1 no longer get treated as if they also have an E parameter
- Setting M558 A parameter to anything >31 set it to 0 instead of to 31
- G92 should not constrain the passed coordinates to the M208 limits if M564 S0 has been used to disable limits
- On Polar printers, moves that crossed the boundary between -180deg and +180deg turntable positions were executed very slowly
- On the 12864 display, fields that became visible and then became hidden again were not erased from the screen
- If an assertion failure occurred in the FreeRTOS kernel when no task was active, or a stack overflow was detected when no task was active, the crash handler itself crashed while trying to retrieve the task name, so the stored software reset data was incorrect
- On a few 12864 displays, Kanji characters were displayed on tpo of the graphics display
- On delta printers, the code to limit the height at the end of a move to the reachable height didn't always ensure that the heights of the intermediate positions were reachable
- When height map and filament files were written, the month number in the date written in the header was too low by 1
- If a M106 Pnn Snn command is received, and fan #nn is being used as a print cooling fan, the reported print cooling fan speed is now updated
- When the WiFi firmware was being updated, general status responses are no longer sent to USB or PanelDue, so that the firmware update progress messages are clearer
- M453 switches to CNC mode even if the P parameter was not valid
- MBytes/sec -> Mbytes/sec in M122 P104 report

Version 2.02 (Duet 2 series) and 1.23 (Duet 06/085)
===================================================

Upgrade notes:
- **Very important!** If you use M452 to put your machine into Laser mode, you must replace all S parameters in G1 commands in homing files etc. by H parameters. This is because S is now used to control laser power.
- Previously, you could omit the P0 parameter when configuring fan 0. The P0 parameter is now compulsory when configuring fan 0. The only parameters now recognised in M106 commands with no P parameter are S and R.
- If you use the G53 command, note that this now causes tool offsets not to be applied
- If you use a 12864 display, you may wish to add a 'value N501' item to your menus to allow M117 messages to be displayed.
- Please note, the Duet 06/085 release has undergone mimimal testing.

Changes to how G, M and T codes are executed:
- G0 and G1: The primary parameter letter used to control the move type has been changed from S to H. This is because S is the standard parameter to control laser power on a laser cutter or engraver. However, unless you use M452 to put the firmware into Laser mode, you can continue to use S to set the move type.
- When switched into Laser mode (M452), the S parameter on G1 commands sets the laser power. A G1 command with no S parameter keeps the previous laser power. A G0 move always turns the laser off.
- G2/G3 commands now only require that a nonzero I or J parameter is provided; and if the final XY coordinates are not specified, or are equal to the initial XY coordinates, then the firmware generates a complete circle
- In CNC mode, a GCode line that either starts with an axis letter and follows a G0/1/2/3 command or starts with I or J and follows a G2/3 command is assumed to repeat the previous command using the new parameters
- G2 and G3 moves now use a variable segment length that depends on the arc radius
- G10 L2 and G10 L20 can now be used with no P parameter, meaning use the current coordinate system
- If G30 S-2 is commanded when no tool is selected, the command is not executed and an error message is generated
- When G53 is in effect, tool offsets and axis mapping are no longer applied
- G60 now saves the current tool as well as the current user coordinates
- M106 supports a new X parameter to set the maximum allowed fan PWM (thanks @wikriker)
- M106 supports a new A parameter to map the fan to a different fan output or a heater output, or to re-enable a disabled fan
- M116 now accepts an optional S parameter to specify the acceptable temperature difference
- M122 has an additional "power good" report to indicate whether the internal stepper drivers can be used
- M122 now has P103 subfunction to measure the sin + cosine calculation time
- M122 now has P104 subfunction to perform SD card write timing
- M205 is supported to set the jerk limits (in mm/sec) as an alternative to M566
- M208 now accepts Xaa:bb Ycc:dd etc. as an alternative to separate M208 S0 and M208 s1 commands
- M206 now sets the offsets for the current workplace instead of workplace 0
- M226 is now supported even when no file is being printed
- M260 can now receive I2C bytes as well as send them, use the new R parameter to specify how many
- In M260 and M261, I2C addresses can be specified in hex format in quotes, e.g. "0x71" or "x71"
- M291 no longer allows S0 and T0 together because that would create a message that can never time out of be dismissed
- M302 now allows the minimum extrusion and retraction temperatures to be configured (thanks @wikriker)
- M305 temperature sensor type 300-307 now supports a C parameter to select the input channel and a D parameter to select differential mode
- M408 now accepts a P parameter. P0 (default) gives the previous behaviour. P1 S"filter" now returns those parts of the object model that match "filter".
- The mapped fan speed is now sent at the start of the fan speed list in the M408 response so that PanelDue can display it
- If G31 parameters were read from config-override.g (M501) then a subsequent M500 command saves them too
- M500 information saved to config-override.g now includes the workplace coordinate offsets
- When M500 is used the current date/time if known is included in the header comment in config-override.g
- M555 P6 selects nanoDLP compatibility mode
- M557 now supports a P parameter to set the number of X and Y points, as an alternative to using the S parameter to set the spacing
- M558: zero or negative Z probe tolerance (S parameter) with A parameter > 1 now means always average all readings
- M558 now accepts a C parameter to select the endstop number when the mode is 4. M558 P6 is translated to M558 P4 C4, and M558 P7 is translated to M558 P4 C2.
- Added M569 parameters B, H and V to configure blanking time value, hysteresis values (2 or 3 of them) and stealthchop/spreadCycle switchover register value. Also added these values to the M569 report when only a P parameter is given.
- M569 accepts a new F parameter for the off-time. Valid values are 1 to 15. 
- M569 now rejects disallowed combinations of TOFF and TBL in the chopper control register of TMC2660 or TMC22xx drivers are rejected
- M584 can now use dummy (high) driver numbers to assign an axis or extruder to no driver
- M591 responses have been shortened by removing unnecessary text
- The M591 report for a simple (switch) filament monitor now includes the filament present/not present status
- M600 is supported
- M650 and M651 are supported (for nanDLP)
- M703 is supported (thanks chrishamm)
- M851 is supported for Marlin compatibility. M851 Znn is equivalent to G31 Z-nn except that is also causes a subsequent M500 command to include the G31 parameters in config-override.g, as if M500 P31 had been used instead.
- In M906 if you set the motor idle current percentage to 0 then all drives will be disabled when all motors have been idle for the idle timeout and all axes will be flagged as not homed
- M918 now resets and initialises the 12864 display
- M918 with no parameters now reports current display and encoder settings
- T R# (where # is a restore point number) is now supported

Motion control changes:
- After attempting to apply babystepping to existing queued moves, any residual babystepping is now actioned immediately instead of waiting for another move
- When TMC22xx drivers on the Duet Maestro are configured in stealthChop mode, the driver is programmed to switch over to spreadCycle automatically at high speeds
- Added special support for coast-to-end in RecalculateMove (but pressure advance should work better than coast-to-end)
- A message is displayed when the stepper drivers report open-circuit motors
- Dynamic acceleration control/dynamic ringing cancellation is supported. Use M593 to configure it.
- If you try to move motors connected to internal drivers when VIN is too low or too high, a warning message is generated

Web server changes:
- The HTTP reject message now includes a CORS header, for future versions of DWC
- A content-length header is included whenever a file is returned, for future versions of DWC
- HTTP requests to fetch a directory are redirected to index.html in /www, for future versions of DWC
- The HTTP index page is now index.html, but it falls back to reprap.htm if index.html is not found
- When uploading a file via HTTP the firmware now pre-allocates SD card storage for it
- If the HTTP server runs out of buffers while trying to service a request it now returns a 503 http error
- Improved the HTTP 'page not found' message
- Http responses now use \r\n as the line ending, not \n

Duet Maestro 12864 display changes:
- Support for 12864 displays on Duet Maestro greatly improved (thanks to M3D for much of this)
- 12864 display items 510-518 now display user position, not machine position
- 12864 display can now show 1-line M117 messages
- 12864 display can now show M291 messages boxes and the dialog for and manual Z-probing
- 12864 now supports adjusting baby stepping and XYZ position directly using the encoder
- 12864 display now allows you to left-, centre- or right-align 'text', 'value' and 'alter' items, and the text within buttons is centre-aligned
- 12864 menu files are now allowed to start with letters G, M and T as well as all other letters
- On the 12864 display, error messages can be acknowledged by a button press
- The maximum length of a menu file line for the Duet Maestro 12864 display has been increased from 80 to 100 characters

Other new features and changed behaviour:
- When the WiFi module is in client mode it tries to auto-reconnect continuously if the connection is lost
- If a file delete request fails because the file or the path is not found, the firmware no longer generates a global error message (but it still returns an error to the caller)
- Changed "Resume-after-power-fail state saved" message to "Resume state saved"
- When delta auto calibration produces NaNs, calibration is abandoned and an error message is produced
- Emergency stop commands sent by PanelDue running latest firmware are acted on immediately
- Added 'deprecated' message when legacy 3-, 4- or 5-point bed compensation is used
- The Idle task is now included in task list
- When storage module debug is enabled, failing to open a file is now a warning not an error because it is a normal occurrence when optional files are not present (e.g. tool change files, start.g, stop.g)
- Increased number of restore points from 3 to 6
- Implemented the object model framework and a few variables
- Z leadscrew or manual be levelling adjustment results are now logged even if the process failed, if logging is enabled
- Mesh probing results are now logged, if logging is enabled
- Error and warning messages generated by incorrect GCode commands are now logged, if logging is enabled
- Added paused coordinates to 'printing paused' message
- If a bad curve fit occurs during tuning, the values found are displayed as A, C and D instead of G, tc and td to better relate to M307
- Hardware configurations with more or fewer endstop inputs than motor drivers are supported
- When a print is paused, the coordinates of the pause point are included in the "Printing paused" message
- If heater tuning fails due to a bad curve fit, the parameters measured are reported as A, C and D (to match the M307 parameter letters) instead of G, tc and td
- On Duet Ethernet and Duet Maestro the physical link speed and half/full duplex status is included in the Network section of the M122 report
- The total number of axes is now passed to DWC and PanelDue as well as the number of visible axes

Internal changes:
- Memory for a 12864 display and the associated menu system is not allocated until M918 is processed
- Upgraded to latest version of FatFS

Bug fixes:
- During printing, the count of layers printed was sometimes incorrect.
- On the Duet Maestro, the step timer clock sometimes returned the incorrect value. This could cause occasional jerky movement and possibly lost steps.
- Homing a CoreXY machine with both DDA and Move debug enabled caused a crash (thanks @sdavi)
- Fixed behaviour when moves call for extrusion amounts smaller than one microstep
- On the Duet Maestro the 12864 display CS pin is now set low at startup, to avoid spurious characters being displayed if other SPI activity occurs before the display is initialised
- On the 12864 display, multi-line images did not display correctly
- On the 12864 display, the last byte of images didn't display correctly
- On the 12864 display, when entering a subfolder from a file list, the screen did not update until you rotated the encoder
- On the 12864 display, buttons sometimes disappeared when moving between them
- On the 12864 display, item numbers 79, 179 and 279 were not implemented
- On the 12864 display, after displaying an error message the 20-second inactivity timeout was used before reverting to the main menu instead of the 6-second error message timeout
- On the Duet 2 Maestro, if the SD card menu on the 12864 display was used then the network kept disconnecting
- 12864 display of speed factor was wrong
- Fixed potential buffer overflow issues in 12864 menu code
- The scheduled move count was too high by 1 after an emergency pause
- The print progress calculated from filament used was incorrect when using a mixing tool if the sum of mix values was not 1 (e.g. when ditto printing)
- On SCARA printers, a G30 command immediately after homing the proximal and distal arms could fail due to rounding errors
- Heaters were turning on momentarily when the Duet was reset
- M106 Svv and M106 Pn Svv were always setting max PWM if vv > 1 and there were additional parameters in the command e.g. F
- G30 H parameters didn't work if deployprobe.g or retractprobe.g files were present
- A processor timing issue could cause the watchdog to trigger and reset the Duet as soon as it was enabled. This caused some Duet+DueX configurations to take several attempts to start up.
- After using G30 S-2 the tool offset was set in the wrong direction
- After using G30 S-2 the user coordinates were not updated to account for new tool offset
- Under some conditions the M400 command could greatly slow down movements, making it look as if the print had stalled
- Lookahead errors were occasionally reported because of small rounding errors
- Further limited the amount of CPU time used to refresh the 12864 display
- Fixed incorrect check for G2/G3 missing parameter
- Fix CoreXYUV stall detection
- Absolute babystepping was restricted to 1mm change
- After using G10 L2 or G10 L20 to change workplace coordinate offsets, the user positions of axes other than X and Y were not updated
- M915 now recognises the E parameter
- M915 output was truncated if no drives were specified
- On the Duet 2 Maestro, if a BLTouch Z probe was used then the pin didn't always stay retracted after the probe was triggered
- If G30 S-1 was sent with the Z probe type set to zero then reported trigger height was an undefined value
- Additional axes on delta printers were not coordinated with XYZ movement
- DWC disconnected when a message box and a beep were pending at the same time
- The code to report overheating TMC22xx or TMC2660 drivers to Duet Web Control, Panel Due console and USB hosts wasn't working
- On a delta printer, if you created additional axes then the movement of these axes was not coordinated with the effector movement
- If your GCode generated a beep (M300) and a dialog box (M291) in quick succession, a bad JSON response was constructed, causing Duet Web Control to disconnect
- The list of GCode files and folders displayed by DWC was incomplete if there were too many files. The list is now complete if DWC 1.22.1 or later is used.
- I2C errors sometimes occurred if there was a task switch in the middle of an I2C transaction
- The E parameter was ignored in G0 commands
- Fixed bad JSON response when the 'first' parameter of a rr_filelist HTML command was non-zero
- M106 Snn commands with no P parameter failed if fan 0 had been disabled but the print cooling fan was mapped to another fan in the current tool definition
- If config.g invoked a macro then final values were copied to GCode sources too early and a subsequent M501 command wasn't acted on (thanks chrishamm)
- If an emergency stop occurred during execution of a macro, an internal seek error message was sometimes generated. Emergency stop now closes any active print files and macro files.

Version 2.01 (Duet 2 series) and 1.22 (Duet 06/085)
===================================================

Upgrade notes:
- Compatible files are DuetWiFiserver 1.21 and DuetWebControl 1.21.2-dc42. Use of DWC 1.21 or earlier may result in "Not authorized" disconnections if you have a password set.
- On the Duet WiFi and Duet Ethernet, in previous versions drivers 3 to 11 inclusive defaulted to being extruder drives. In this version, only drivers 2 to 9 inclusive default to extruder drives. This is to avoid issues when users try to change microstepping on all extruder drives. If you want to use drivers 10 and 11 as extruder drive,s you will have to assign them explicitly using M584. Also if have not used M584 to assign drives and you are listing all the extruder drives in a command (e.g. M906 E1000:1000:1000:1000:1000:1000:1000:1000:1000) then you will need to reduce the number of E values listed from 9 to 7.

Bug fixes:
- The assumed Z position at power up had an undefined value
- Non-movement commands were not correctly synchronised with movement commands when there were more than 8 non-movement commands interspersed with more than 2 seconds of movement
- If M28/M29 was used in a macro file to then the commands between M28 and M29 were executed as well as being written to the target file. Also the M29 command to close the file was not recognised if it occurred right and the end of the macro file with no following newline character.
- Fan RPM readings were incorrect
- If additional axes were created on a delta printer, after they were homed the axis coordinate was incorrect
- If retractprobe.g contained any movement commands, G32 worked but produced no output at the end of bed probing
- If you used the jog buttons in DWC before homing the axes, and you didn't use M564 H0 in config.g, then as well as the "insufficient axes homed" error message, a stack underflow error message was produced
- Neither M561 nor G29 S2 adjusted the user Z coordinate if bed compensation was previously active (G29 S2 did if you ran it twice)
- Duet 06/085 only: fixed a buffer overflow in the Netbios responder code
- Duet 2 series: when an under-voltage event occurred, spurious driver status warnings/errors were sometimes reported
- Duet 2 series: when an under- or over-voltage event occurred, the VIN voltage reported was the current voltage, not the voltage when the event was recorded
- When an axis was made visible and later hidden, subsequent move commands sometimes sent step commands incorrectly to the driver(s) associated with that axis. This could cause unwanted movement if the axis was still mapped to a real driver. If it was mapped to a dummy driver, it could still cause step errors to be recorded and/or some movements to be slowed down.
- The longest loop time reported by M122 was distorted by the fact that M122 itself takes a long time to execute, due to the volume of output it produces and the need to synchronise with the Network task
- When using a mixing extruder, the feed rate for extruder-only moves was incorrect
- If additional axes were not created in the order UVWABC then incorrect homing files might be run (thanks chrishamm)
- On the Duet Maestro, the 7th stepper step/dir pin numbers were swapped
- If you paused a print during a G2/G3 arc move, on resuming it the arc centre was at the wrong place. This release defers the pause until the arc move is completed.
- If a command that interrogated the network (e.g. M122 on the Duet WiFi) was sent from USB, PanelDue or another non-network channel, the network subsystem could be accessed by multiple tasks concurrently, causing network disconnections or other errors
- When using a bltouch, between probe points the pin retracted, deployed and retracted again
- M206 with no parameters didn't report the current axis offsets
- During heating, the firmware returned M408 S0 responses to the PanelDue port even if the last request was M408 S2
- Fixed VBUS detection (thanks chrishamm)
- If the resume threshold in the M911 command was set higher than the supply voltage then the save-on-power-off mechanism never got primed. It will now prime at the auto-save threshold plus 0.5V or the resume threshold, whichever is lower.
- Fixed "2dtstc2diva=u" in debug printout
- Where a G- or M-code command parameter was supposed to accept unsigned values only, if a negative value was supplied then it was accepted and converted to a large unsigned value

New features/changed behaviour:
- Fans can now be named (thanks chrishamm)
- The Z probe MOD pin can now be accessed as a GPIO pin
- The maximum number of drivers per axis on the Duet WiFi/Ethernet has been increased form 4 to 5
- Dumb drivers no longer default to being extruders by default on the Duet WiFi/Ethernet/Maestro
- On the Duet 2 Maestro, the 2 optional add-on drivers are now assumed to be TMC2224 with UART interface
- When the Z probe type is set to 9 for BLTouch, the probe output is no longer filtered, for faster response
- If an error occurs while reading or writing the SD card, the operation is retried. The M122 report includes the maximum number of retries that were done before a successful outcome.
- On the Duet WiFi/Ethernet, at startup the firmware does additional retries when checking for the presence of a DueX2 or DueX5 and/or additional I/O expansion board
- When a resurrect.g file is generated, it now includes G92 commands just before it invokes resurrect-prologue.g, to set the assumed head position to the point at which power was lost or the print was paused. This is to better handle printers for which homing Z is not possible when a print is already on the bed. Caution: this doesn't allow for any Z lift or other movement in the power fail script.
- RTOS builds only: added a separate software watchdog to monitor the Heat task
- RTOS builds only: in the M122 report, the software reset data now includes which task was active, and only owned mutexes are listed
- Upgraded compiler to 2018-q2-update
- If the firmware gets stuck in a spin loop, the RTOS builds now saves data from the process stack instead oif the system stack, to provide more useful information
- Increased M999 delay to 1 second
- The report generated by M122 now includes a list of mutexes and their owners
- Added SW_ENC pin on CONN_SD to available GPIO ports (thanks chrishamm)

Version 2.0 (Duet 2 series) and 1.21.1 (other hardware)
=============================================================
Upgrade notes:
- Compatible files are DuetWiFiserver 1.21 and DuetWebControl 1.21.1. Use of older versions of DWC may result in "Not authorized" disconnections.
- When the machine mode is set to CNC, G0 movement behaviour is changed to align more with the NIST standard (see 2.0RC1 release notes).
- If you have a DHT temperature/humidity sensor connected to the CS6 pin on a Duet 2, the channel numbers (X parameter in the M305 commands) are changed to 405 (was 400) for the temperature sensor and 455 (was 401) for the humidity sensor.
- If you have a simple switch-type filament monitor configured using M591, you need to add the S1 parameter to enable it
- The number of GCode files in a single folder that can be displayed by DWC is lower than before. This was required to fix other issues. If you can't see all of your GCode files, or files that you have just uploaded, the workaround is to move some files into subfolders. If you can't see any subfolders in DWC, you may have to move the SD card to a PC to do this. Alternatively, use the backup facility in DWC to backup the files that you can see, then delete them; then you should be able to see the other files.

New features and changed behaviour:
- The M569 response when only the P parameter is given now includes chopper configuration register if it is a smart driver
- Multiple DHT sensors are supported, connected to any of the 8 SPI daughter board chip select pins
- The Duet 2 Maestro build now supports DHT sensors
- Simple switch-type filament sensors can now be enabled/disabled using S1/S0 in the M591 command
- If the HSMCI idle function times out, an error code bit is now set
- If config.g is not found then config.g.bak is run instead of default.g
- If an error occurs when accessing the SD card, the error message now includes an error code
- Added S3 option to M20 to get file list including size, date/time etc.
- rr_files, rr_filelist, M20 S2 and M20 S3 now provide for retrieving the list if files in chunks, using a "first" parameter in rr_files/rr_filelist or "R" parameter in M20 S2/S2, and a "next" field in the response
- M502 now resets all firmware parameters back to the values in config.g except network parameters
- RRF attempts to pass the estimated print time and simulated print time from GCode files to DWC and PanelDue
- When M37 is used to simulate a file, at the end of a successful simulation the simulated print time is appended to the file unless parameter F0 is included in the M37 command
- The default folder for the M36 command is now 0:/gcodes instead of 0:/
- M144 S1 now sets the bed to Active mode. M144 with any other S parameter or no S parameter sets the bed to standby as before.
- Added more functionality to 12864 displays on Duet 2 Maestro
- Default stepper driver mode for TMC2224 drivers is now stealthchop2
- Stepper driver mode for TMC2660 and TMC2224 drivers can now be set via the D parameter in M569
- Stepper driver chopper control register can now be set via the C parameter in M569 - USE THIS	ONLY IF YOU KNOW WHAT YOU ARE DOING!
- When Z probe type 0 is selected and DWC/PanelDue have prompted the user to jog Z, axis movement before homing is allowed
- When the machine mode is set to CNC, G0 moves are now done using the maximum travel speed of the machine in accordance with the NIST standard, and E and F parameters are no longer recognised.
- M114 reports the user coordinates first and the machine coordinates at the end
- Minimum allowed jerk setting (M556) is reduced from 1mm/sec to 0.1mm/sec
- When M500 is used a warning is given if M501 was not run in config.g
- G2 and G3 no longer require all of X, Y, I, J to be specified. X or Y and I or J is sufficient.
- The user coordinates are updated if G10 is used to change the offsets of the current tool
- Added Z probe type 10 (Z motor stall)
- Simulations can now be run when the printer is not homed
- Multiple splindles are supported in CNC mode (thanks chrishamm)

Bug fixes:
- If getting file info for DWC or PanelDue timed out, it didn't close the file. This could lead to running out of open file entries.
- The DHT sensor task ran out of stack space under some conditions
- Corrected DHT start bit timing to avoid a bus conflict
- Fixed unreliable DHT sensor reading in RTOS build, caused by call to micros()
- Pausing between the segments of a segmented move didn't happen even if the jerk settings were high enough
- Possible fix for incorrect extrusion in the first move after resuming from a pause
- If filament monitors were deleted or the type changed, this could result in an exception
- When step rate limiting occurred due to the speed and microstepping combination needing an excessive pulse rate, movement could become irregular
- When the SD card was removed during a print it said 1 file was invalidated even if there were more
- When the SD card was removed during a print, several "internal error" messages wree generated, but no "print abandoned" or similar message, and the heaters remain on
- Emergency stop now turns off all spindles if the machine type is CNC, and the laser if the machine type is Laser
- HTTP request parsing error recovery didn't work on the Duet 2 boards. One consequence was that connecting from Internet Explorer crashed the Duet.
- Spurious stall warnings were sometimes generated when simulating a print
- The print monitor didn't think the print had started until a nozzle had reached target temperature. This meant that layer counting didn't work on machines with no tool heaters, or when the tool temperatures were fluctuating.
- The print monitor didn't count layers when simulating a print
- Axes beyond Z were ignored in G2/G3 moves
- DWC and the Duet could deadlock if the Duet ran out of output buffers
- If the system ran out of output buffers when multiple tasks were generating output (e.g. DWC or Telnet combined with PanelDue or USB) then in rare cases the firmware would reboot
- When high microstepping was used so that MaxReps got very high, certain sequences of movement commands could lock up the movement system. MaxReps has been replaced by a hiccup count.
- M122 reported some parts of network status twice on Duet 2 Ethernet and Duet 2 Maestro
- If a PT1000 sensor was configured using M305 but a thermistor was plugged in instead, the firmware reported semi-random high temperatures instead of an error
- If a PT1000 sensor was configured using M305 and then M305 was used to change it back to a thermistor, it remained configured as a PT1000
- If a delta printer failed to home then DWC might disconnect due to NaN values for the machine coordinates in the rr_status response
- The M105 response on a multi-tool system was not in the exact format that Octoprint required
- Excessive decimal places in some values in M408 responses and rr_status requests have been removed
- G1 E moves with the S1 parameter (i.e. filament loading with extruder stall detection) on a delta reported "Error: G0/G1: attempt to move delta motors to absolute positions"
- Duet Web Control clients that go to sleep without disconnecting first are timed out after 8 seconds (this was already happening on the Duet 06/085 but not on Duet 2 series)
- VSSA fault detection was not working on the Duet Ethernet in firmware 1.21
- If G30 was used to set an accurate Z height after mesh bed probing or loading a height map, if bed compensation was then cancelled then any Z offset from the height map remained. One consequence of this was that if bed probing was run again, the original height map Z offset was carried through to the new one, but the sign of the offset was reversed.

Other changes:
- RepRapFirmware 2.0 uses a real time operating system kernel (FreeRTOS). Currently there are just four tasks: Main, Heat, Network and DHT (if DHT sensors are configured). The tasks and their free stack space are listed in the M122 diagnostics report.
- Custom SafeStrtod, SafeVsnprintf and related functions are used instead of C library strtod, vsnprintf etc. The replacements are thread safe and use less stack than the originals.
- nano-newlib is used instead of newlib
- Incorporated chrishamm's changes to scanner support

Version 1.21
============
Upgrade notes:
- The compatible DuetWiFiServer version is 1.21. If you are already running firmware 1.19 or later, you can install this before or after upgrading the main firmware, it doesn't matter.
- The compatible Duet Web Control is 1.21. Install it after upgrading the main firmware.
- On Cartesian and CoreXY printers, normal G0 and G1 moves are no longer allowed before the corresponding axes have been homed. In particular, if your homex.g, homey.g and homeall.g files raise Z a little at the start and lower it at the end, you will need to add the S2 parameter to those G1 Z moves. Otherwise the G1 Z move will be refused unless Z has already been homed and the homing macro will be terminated. If you want to allow axis movement prior to homing, put M564 H0 in config.g.
- The binary filename for the Duet WiFi and Duet Ethernet is now called Duet2CombinedFirmware.bin. However, your existing firmware version may expect it to be called DuetWiFiFirmware.bin or DuetEthernetFirmware.bin. For your convenience, this release includes copies of Duet2CombinedFirmware.bin with those names.
- If you have a start.g macro file in the /sys folder of your SD card, remove or rename it, unless you want it to be run every time you start a print from SD card
- See also the upgrade notes for version 1.20 if you are upgrading from a version earlier than that

New features and changed behaviour:
- The rr_status and M408 reports now include the spindle speed
- M106 now also reports the current fan PWM for thermostatically-controlled fans
- Machine coordinates are always used when running system macros automatically.
- New M569 T parameter options to specify step pulse width, step pulse interval, direction setup and direction hold times
- M665 now sets the M208 limits (except Z min) to match the machine limits, so that Duet Web Control reports the correct values
- On CoreXZ machines we no longer require Z to be homed before bed probing with G30
- M589 now checks that the password is either empty or 8 characters minimum
- G10 L2 is supported as an alternative way to set tool offsets
- G10 L20 is supported as an alternative way to set workspace coordinates
- G30 with no parameters now supports multi-tap, and the head is raised or the bed lowered to the dive height after probing
- Heater fault detection is suppressed when heaters are suspended during bed probing
- DuetWiFiServer.bin uses a new SDK version, which seems to resolve some issues
- On boards with a W5500 Ethernet interface, the Ethernet PHY is now programmed to auto negotiate
- Added M564 H0 command to allow axis movement before homing on Cartesian/CoreXY printers
- The filament length comment proposed to be generated by the next version of Cura when using more than one filament is supported
- On Cartesian and CoreXY printers, normal movement commands are no longer permitted until the corresponding axes have been homed
- Illegal movement commands in a print file or macro file cause the file to be terminated and heaters/spindle motors/lasers to be turned off; except that when the printer is in FDM mode, G0 and G1 moves outside the movement limits are just truncated as before
- The M39 command reports the SD card cluster size
- If GCode attempts to set the temperature of a non-existent bed or chamber heater to zero (to turn it off), the error message that would normally be generated is suppressed
- G60 command to set a restore point is implemented
- M671 command now supports the F (fudge factor) parameter
- The standstill current fraction can now be set on the Duet Maestro build
- M118 support added (thanks chrishamm)
- When using external stepper drivers the DIR signal is no longer changed before the step pulse has ended
- The M452, M453 and M573 commands now support the I1 parameter to invert the laser, spindle or extrusion signal polarity
- A common firmware image is used for both the Duet WiFi and the Duet Ethernet. For now the name of the binary file is DuetWiFiFirmware.bin.
- The filament usage comments generated by Ideamaker are now recognised
- A separate Z probe type (P9 in M558) is now used for BLTouch and compatible Z probes. When this is selected, the deployprobe.g macro is run just before each probing move, and retractprobe.g is run every time the probe triggers, or at the end of the probing move if it doesn't trigger
- All heaters are turned off during probing moves if parameter B1 is specified in the M558 command
- The HTTP and FTP connection password is now compared using a constant-time algorithm to avoid leaking information
- The M671 command can be used with just one pair of X and Y coordinates, which disables the manual bed calibration assistant
- Continuous rotation proximal and distal joints are now supported on SCARA printers, and continuous rotation turntables are assumed on Polar printers
- When beginning a print from SD card, macro /sys/start.g is run if it is present
- Sending M0 or M1 from a source other than the SD card no longer stops an SD card print that is not paused
- When a serial input channel detects a framing or overrun error other than in a GCode end-of-line comment, the line of GCode in which it occurs is discarded
- When multi-touch Z probing is enabled, if during bed.g bed probing the maximum probe count is exceeded, the average of all the height readings is used instead of refusing to do calibration or levelling at the end
- Any I2C errors that occur when accessing a DueX5 or SX1509B expander are recorded, and the error count is displayed in the M122 report
- Added support for nonlinear extruder drives (M592)
- Added support for Duet3D laser filament monitor
- Added support for M260 and M261 (send/receive I2C)
- Added support for workspace coordinates (G10 L2, G53 to G59.3)
- Added absolute babystepping mode in M290 command
- Added multi-touch Z probing (M558 A and S parameters)
- Added M39 command to return SD card free space and other SD card information
- M591 D# response now includes the measured steps/mm and tolerance
- Support endstops 5-9 when no DueX board is present
- Show Duet board revision as 1.02+ if we detect it
- Recognise Ideamaker generated-by string
- Cache is now disabled on the ATSAM4E
- Ported DHCP changes from LWIP 2 to Duet 06/085 build

Other changes:
- Duet Web Control and PanelDue now report the coordinates relative to the origin of the current workplace
- Rewrote dhcp_rec function on Duet06/085 to avoid goto statements (possible fix for startup problem when using DHCP)

Changes to DuetWiFServer:
- Errors reported by LWIP for listening PCBs are handled, and reported if WiFi debugging is enabled

Changes to iap and iap4e:
- The binary file is no longer deleted after successful installation
- On the Duet WiFi/Ethernet the DIAG LED blinks while the firmware is being installed

Bug fixes:
- Pulse-type filament monitors are now working
- WiFi sockets whose connections abort are now terminated to make them available for re-use
- DWC Machine Properties page shows the correct state of active low endstops
- M116 commands were sometimes executed out-of-order relative to previous G10 commands if movement commands were in progress
- A short delay is inserted when M558 is used to change the Z probe type, to allow the averaging filters to accumulate the new data
- M291 messages which are non-blocking (i.e. mode < 2) are now synchronised to queued moves, like M117 messages
- Corrected Z probe input pin in RADDS build
- Possible fix to Duet 06/085 failure to start when using DHCP
- Duet Ethernet only: fixed bugs in the DHCP client code that could cause the printer to become very slow
- G2 and G3 arc moves are terminated if the attempt to exceed the axis limits
- During simulation the status is set to "Simulating" instead of "Printing"
- M556 with a zero S parameter no longer messes up the coordinate calculations
- When large files were uploaded or copied to the SD card and the cluster size was small, HTTP requests could time out while DWC attempted to get the information for those files
- Endstops 5 thru 9 on a DueX2/DueX5 board can now be used for simple filament sensors
- When resuming a paused print the first move executed was sometimes incorrect
- When the Duet WiFi was configured to run in access point mode, sometimes it wouldn't start in config.g or started with the wrong SSID and had to be started manually. This is believed fixed, but you may need to start it manually the first time.
- When the M207 Z hop setting was changed during a print while a travel move is in progress, at the end of the travel move the head was lowered by the new value for Z hop instead of the original value
- Endstop pins 5-9 (E2-E6) on the expansion connector are now working
- If neither of the /www/reprap.htm and /www/html404.htm files was present when the web server as asked to fetch a file, it returned a 404 error, however there was no associated plain text response so most browsers displayed a blank screen (fixed in Duet WiFi/Ethernet build only)
- If a G0 or G1 command resulted in no movement (e.g. because it was a duplicate of a previous line) then correct synchronisation between subsequent movement and non-movement commands was lost
- When a simulated print ends or is cancelled, stop.g, sleep.g and cancel.g are no longer run
- Fixed a 1-step error in the commanded extrusion amount that the filament sensor compares with the measured extrusion
- Filament monitors are now disabled when simulating a print
- Fixed step number calculation bug that caused benign step error reports with some values of pressure advance
- Fixed a lookahead bug that caused occasional step errors. Print quality and speed may have been affected.
- If no temperature sensor is configured for a heater. M305 Pn with no other parameters no longer allocates a thermistor
- Fixed "listen failed" error after repeated use of FTP
- Fixed M304
- Fixed crash when an attempt was made to configure a filament monitor on a DueX endstop input
- Fixed jerky curves when pressure advance is used and the slicer doesn't command a uniform extrusion rate
- Fixed missing newline at end of "Done printing file" message (for Pronterface)
- M350 Enn with only 1 extruder value specified is now applied to all extruder drives

Version 1.20
===============
Upgrade notes:
- Recommended DuetWiFiServer.bin version is 1.20
- Recommended DuetWebControl version is 1.20RC3. until version 1.20 is available
- Windows device driver versoin 1.19 remains compatible with this release
- If you have a SCARA printer with nonzero crosstalk parameters (C parameters in the M669 command), you may need to adjust the crosstalk values
- If you are using a Duet to control a RepRapPro Ormerod, Huxley Duo or Mendel 3 printer or any other printer that uses the Z probe to do X homing, you need to remove the X parameter from the existing M574 command in config,g and add line M574 X1 S2.
- If you are using a Duet 06 or 085 and you don't already set the P parameter in your G31 command, add P400 to that command to get the same behaviour as before, because the default is now 500.
- If you are using PT100 sensors, make sure you don't have any additional parameters in your M305 commands for those heaters left over from when you were using thermistors. In particular, the R parameter now configures the reference resistor value on the PT100 interface board, and must be omitted or set to 400 when using the Duet3D PT100 daughter board.
- The parameters to the M911 command (which configures power fail handling) have changed since version 1.19.2. If you use this command in config.g you will have to change it accordingly.
- On a Duet WiFi, if your M552 command in config.g includes a P parameter with an IP address (which was previously ignored), you will need to remove them
- If you currently have G31 parameters for your active Z probe in config-override.g that are different from the ones in config.g, you should copy them to config.g, otherwise they will be lost next time you run M500.
- The 'set output on extrude' function (M571) no longer defaults to FAN0 output. If you use this feature, you must define the output pin explicitly using the P parameter at least once.
- If you are using external drivers and you are not already using the M569 T parameter to extend the minimum step pulse width and interval for them, then you may need to add the T parameter, because efficiency improvements have reduced the minimum width and interval.
- If you are upgrading from a firmware version prior to 1.19, see also the upgrade notes for firmware 1.19.

Known issues:
- If you have a Duet3D beta filament monitor configured and you run a print in simulation mode, the filament monitor keeps reporting insufficient extrusion and pausing the print. This issue is also present in earlier releases.

New features - kinematics and motion:
- Added CoreXYUV kinematics, see M669 command
- Added Hangprinter kinematics (not tested yet), see M669 command
- Added Polar kinematics (not tested yet), see M669 command
- SCARA kinematics crosstalk parameters now relate the movement units, not the number of motor steps
- On SCARA printers, arm position limits are applied as well as XY size limits
- SCARA arm mode changes are now only permitted in uncoordinated (G0) moves
- SCARA printers can now use the manual bed levelling assistant
- SCARA prints are simulated without segmentation so that the simulation runs much faster. In tests, the difference in the simulation time with/without segmentation was negligible.
- When using a segmented kinematics such as SCARA, or when long moves are segmented due to mesh bed compensation, segmented moves can be paused between segments
- M584 can now be used to create additional axes using any of the letters UVWABC in any order
- The start and end speeds of short segmented moves in accelerating or decelerating sequences are more closely matched where possible, which should give smoother motion
- XY speed limiting is now done separately for each kinematics, in particular for CoreXY printers it is more accurate

New features - heating and temperature measurement:
- PT1000 sensors connected to thermistor inputs are now supported
- Heater fault timeout to cancelling print is now configurable (M570 S parameter, in minutes)
- Multiple bed and chamber heaters are supported on the Duet WiFi/Ethernet
- M307 now accepts an F parameter to allow the PWM frequency to be set
- M562 with no parameters now clears all heater faults
- M109 and M104 commands now set both the active and the standby temperatures of the tool.
- M109 now only selects a tool if no tool was selected
- Duet WiFi and Duet Ethernet: Extruder heater PWM values are now compensated for supply voltage
- When a heater fault occurs, the print is now paused and all heaters are turned off except bed and chamber heaters. After a timeout period, the print is cancelled, all remaining heaters are turned off, and the firmware attempts to turn the power off as if M81 had been received.
- Heater 0 values are sent to to PanelDue even if there is no heated bed
- The value of the reference resistor on MAX31865 PT100 interface boards can now be configured (thanks bpaczkowski)
- 3-wire PT100 sensors are now supported (thanks zlowred)
- The calculation of PID parameters from the heater model has been changed to provide faster heating and less risk of undershoot
- Added support for DHT11, DHT21 and DHT22 temperature/humidity sensors (thanks chrishamm)
- Increased ADC oversample bits to 2
- When tuning a heater, any previous maximum PWM value that was set is now ignored
- The thermocouple type letter in the M305 command to configure a MAX31856-based thermocouple adapter may now be in lower case
- M408 S1/2/3 responses now include dummy values for the bed heater if there is no bed heater
- The change to fast PID parameters is now made when the temperature is within 3C of the target instead of when within 1C

New features - GCode interpreter:
- The maximum length of GCode commands has been increased, in particular to allow long passwords in M587 commands
- T R1 activates the tool that was active at the last pause
- A line of GCode may now contain multiple G- and M-commands. The commands are executed sequentially. A command that takes an un-quoted string parameter must be the last command on the line. A T command must be on a line by itself.
- String parameters (e.g. filenames) can optionally be enclosed in double quote characters, in all GCode commands for which double quote characters are not compulsory
- When a T command is sent and the current tool does not change, the firmware now makes sure that the tool heaters are turned on, in case they had been turned off explicitly e.g. due to upgrading WiFi firmware
- When executing macros, non-movement commands are now synchronised to movement commands even if they are not normally synchronised for that GCode command source
- M574 command has new options S2 to select the Z probe and S3 to select motor load detection, in place of using an endstop switch
- The M81 command now accepts an optional S1 parameter, which defers the power down until al thermostatic fans have stopped
- Added M122 P1 command for use in the Duet test procedure
- M122 now includes the minimum and maximum StallGuard registers for each driver seen since the last M122 command
- M558 now supports new Z probe types 7 (switch connected to Z endstop) and 8 (as type 5 without any filtering, for faster response)
- Added M915 to configure motor load monitoring, see https://duet3d.com/wiki/Stall_detection_and_sensorless_homing.
- All error messages relating to incorrect use of a G- or M-code now include the G- or M-number of the command that caused them
- Added support for M3, M4, M5 and M450-M453
- Added protection against a dud command line containing letter M being interpreted as a M0 command
- M572 command now allows multiple colon-separate D values

New features - networking:
- Added deleteexisting=yes option in http move command
- The FTP responder now supports the "CWD ." command
- Tool offsets and fan mapping are now passed to DWC
- M587 now checks that the password is either empty or is at least 8 characters long
- Duet WiFi only: M587 and M589 without parameters now report the IP addresses etc. as well as the SSID (needs DuetWiFiServer 1.20)
- The parameters in rr_ http commands are now all order-independent
- Duet WiFi: M122 diagnostics now include the wifi module sleep mode and additional network diagnostics
- On the Duet WiFi the M552 command takes an optional SSID parameter, allowing you to connect to a specified SSID even if it is hidden

New features - other:
- Loss of power is now handled much faster. The print is paused in the middle of a move if necessary. The M911 parameters are changed to facilitate this.
- Following a power failure, M916 can now be used to resume the print instead of using M98 Presurrect.g
- A resurrect.g file is now created any time the print is paused. This allows for planned power down and resume.
- Added event logging, controlled by the M929 command
- More free memory is available, especially in the Duet 0.6/0.8.5 build
- The software reset data now records the date/time of the reset if known and a longer stack trace
- SD card detection is now working properly (Duet WiFi/Ethernet only)
- A file that is open on the SD card can no longer be deleted
- DuetWiFi/Duet Ethernet: the secondary hardware watchdog is now enabled and generates an interrupt, so that if interrupts are enabled at the time then a crash dump can be saved. If this fails then the primary watchdog will reset the processor as before.
- The filament monitor always updates calibration accumulators during a print even if not in calibration mode
- New debugging module 14 added to report debugging message from the WiFi module to USB. Use M111 S1 P14 to activate it. Needs DuetWiFiServer 1.20.
- You can now disable monitoring of TMC2660 drivers that are not in use by using parameter R-1 in the corresponding M569 command
- The M585 (probe tool) command is now implemented (thanks chrishamm)
- If axis lengths are adjusted by probing, a subsequent M500 command saves them in config-override.g
- If tool offsets are adjusted by probing, a subsequent M500 command saves them in config-override.g 
- The layer counting mechanism has been modified to better handle GCode files that use a different layer height when printing support
- Debug messages sent to the USB port are truncated or thrown away if a software watchdog reset is imminent
- The TMC2660 drivers are configured to detect short-to-ground conditions faster
- The grid defined by M557 is now stored separately from the grid loaded by G29 S1 so that they don't overwrite each other
- The commands to resume printing that are written to resurrect.g now move the head to 2mm above the printing height, then sideways, then down
- When M591 and G32 are used to produce manual bed levelling adjustments, the first screw defined in the M671 command is left alone. Previously the screw needing the smallest correction was left alone.
- When doing a firmware upgrade, the message sent to the USB port now warns that the USB will be disconnected

Bug fixes:
- M107 now turns off the mapped fan
- Fixed move timing when a long slow printing move follows a faster printing move
- If a file being printed executed a macro right at the start, DWC could assume that the print had already finished (thanks chrishamm)
- M109 commands could cause unwanted head movement when no tool change was required
- If a filament error occurred more than once, the print would be paused each time but a message box would only be displayed the first time 
- If a new request arrived but no responder was available, there was no timeout waiting for a responder (thanks chrishamm)
- Z probe types 5 and 8 didn't work on Duet 085
- Fixed issues caused by insufficient RAM (stack running into heap) by freeing up additional RAM in the movement structures
- M201/203/208/566 commands now allow parameters for invisible axes to be set
- If a M584 command does not include the P parameter, the number of visible axes is only set to the number of total axes if new axes are created in the command
- Semicolons inside quoted strings were still stripped out from some gcode sources
- Giving silly numbers of microsteps in M350 messed up the steps/mm
- High microstepping and high acceleration on deltas led to poor timing in deceleration phase and high MaxReps
- The commands in tool change macro files were sometimes not fully executed before the next action
- Speed limits in CoreXYU kinematics were not correct for the U axis
- Workaround for SX1509B chip problem: if an analog write was performed to a pin on an SX1509B device, subsequent digital writes and pinMode calls didn't work
- When a print was resumed after power failure, the amount of extrusion during the initial partial move was incorrect
- The determination of whether a print is in the process of pausing did not take account of all possible gcode sources
- M0 and M1 commands no longer turn off heaters and drives when in simulation mode
- SCARA printer homing didn't take account of the crosstalk parameters
- M589 with an S parameter now flags an error if there is no I (IP address) parameter
- When resuming a print, the initial feed rate wasn't being passed to the SD card GCode source
- Configuring MAX31856 thermocouple boards sometimes resulted in strange behaviour because of a buffer overflow
- The code to detect M122 early also recognised commands of the form Mxxx122 where xxx were non-numeric
- An error in computing the time taken to execute moves that were not yet frozen caused the first movement on a SCARA printer following homing to be jerky
- An extra space in the output from the M114 command confused Pronterface, causing it to print exception messages
- An error in the motion planning system that could lead to a null pointer dereference has been fixed. It is not known what symptoms (if any) were associated with this bug.
- If a Duet3D filament sensor was connected and configured but flashing an error code instead of sending filament data, the error recovery code running on the Duet caused short pauses in the print
- On a delta printer if you created additional axes, when you tried to home them it ran homedelta.g instead of e.g. homeu.g
- On a delta printer with additional axes, you can now do XYZ moves as soon as the towers have been homed
- Fixed a possible race condition if the time and date were set at midnight
- The 4-leadscrew auto/4-screw manual bed levelling code didn't work properly
- The P parameter was missing from G10 commands written to resurrect.g
- Arm angle limits are now applied when converting Cartesian to SCARA coordinates
- The correction limit is no longer applied when computing manual bed levelling screw corrections
- When G29 was run on a SCARA printer, probe points could be incorrectly skipped and spurious "not reachable" messages generated
- Pullup resistors are now enabled on endstop inputs 10 and 11 on the CONN_SD connector
- Fixed duplicate error message when a gcode file is not found
- Fixed reference to homing the towers of a SCARA printer in an error message

Other changed behaviour:
- When uploading firmware to the WiFi module, the highest and lowest baud rates have been reduced to 230400 and 9600 respectively
- When uploading firmware to the WiFi module, the system parameter is cleared
- M500 no longer saves G31 parameters to config-override.g by default, but you can use M500 P31 to include them
- The default value for the P parameter in the G31 command has been changed from 400 to 500
- XYZ options have been removed from the M558 command. Use the new M574 S2 option instead.
- G31 P parameter now defaults to 500 instead of 400 on the Duet 06/085 build just as it has done in the other builds
- X homing no longer defaults to using the Z probe in the Duet 06/085 build
- M906 and M913 commands no longer wait for all movement to stop, so that M913 can be used in the power fail script

Internal changes:
- Upgraded compiler version
- Changed to use the hardware floating point ABI on the SAM4
- Simplified conditional directives that depend on the target hardware, by adding new #defines for supported features and a DUET_06_085 #define.
- Changed a path to a file in the linker command line to work around an intermittent Eclipse bug

Version 1.19.2
==============

Upgrade notes from version 1.19:
- If your printer is a SCARA then you now need to set appropriate M208 lower and upper limits for X and Y
- If you are upgrading a Duet WiFi from firmware 1.19, install DuetWiFiFirmware 1.19.2 first, and confirm that the new version is running before installing DuetWiFiServer 1.19.2
- **Important!** See also the upgrade notes for version 1.19 if you are upgrading from 1.18.2 or earlier
- Recommended DuetWebControl version is 1.19
- Recommended DuetWiFiServer version is 1.19.2

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
- In conjunction with DuetWiFiServer 1.19.2, if connection to the access point is lost and the automatic reconnection attempt fails, the WiFi module will attempt a manual reconnect. Reconnection attempts are reported to PanelDue and to USB, and the reconnect count is included in the M122 report.
- Jerk is no longer applied to the boundaries between travel moves and printing moves

Bug fixes:
- Incorrect, very large height errors were sometimes shown in the G29 height map when the number of probe points used approached the maximum supported
- M669 was supposed to report the current kinematics, but didn't except for SCARA printers
- SCARA kinematics didn't apply limits to the Z axis or to any additional axes. Delta printers didn't apply limits to any additional axes.
- SCARA forward kinematics were wrong
- SCARA minimum radius limit was wrong
- If the number of commands in the deferred command queue exceeded 8, commands could be lost
- If a G1 command was used with a coordinate for an axis that X or Y was mapped, that coordinate was ignored unless the S1 or S2 command modifier was used. This affected tool change files on IDEX machines.
- If a reset was forced by the software watchdog because it got stuck in a spin loop, the reset reason in M122 was displayed as 'User'
- The progress bar on PanelDue and the estimated end time based on filament usage were slightly ahead of the true values, especially for small gcode files
- If a move had a tiny amount of XY movement and a much larger amount of extrusion, an extrusion speed in excess of the extruder speed limit set in M203 might be used. This could also be triggered if a move that was too short to need any steps was followed by a pure extrusion move.

Version 1.19
============

Upgrade notes from version 1.18.2:
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
