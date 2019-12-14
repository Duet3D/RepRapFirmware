# List of RepRapFirmware JSON Responses

This document describes the four big JSON responses which are used to report status and configuration details.
For a list of HTTP-related JSON responses, see [HTTP requests](HTTP%20requests.md).

In these responses the following conditions apply:

- Booleans represented as 0 (false) or 1 (true)
- Indices pointing to another resource *might* not be valid. This is due to possible limitations of the CAN expansion board management
- ADC readings are reported on a scale of 0 to 1000
- Lengths are reported in mm
- Speeds are reported in mm/s
- Times are reported in seconds
- Temperatures are reported in Celsius

## Standard Status Response (Type 1)

This status response contains various fields that are expected to change frequently. These values are also included in the type 2 and 3 status responses.

```
{
	"status": "O",
	"coords": {
		"axesHomed": [0, 0, 0],
		"wpl": 1,
		"xyz": [0.000, 0.000, 0.000],
		"machine": [0.000, 0.000, 0.000],
		"extr": []
	},
	"speeds": {
		"requested": 0.0,
		"top": 0.0
	},
	"currentTool": -1,
	"output": {
		"beepDuration": 1234,
		"beepFrequency": 4567,
		"message": "Test message",
		"msgBox": {
			"msg": "my message",
			"title": "optional title",
			"mode": 0,
			"seq": 5,
			"timeout": 10.0,
			"controls": 0
		}
	},
	"params": {
		"atxPower": -1,
		"fanPercent": [-100],
		"speedFactor": 100.0,
		"extrFactors": [],
		"babystep": 0.000,
		"seq": 1
	},
	"sensors": {
		"probeValue": 1000,
		"probeSecondary": 1000,
		"fanRPM": [-1]
	},
	"temps": {
		"bed": {
			"current": -273.1,
			"active": -273.1,
			"standby": -273.1,
			"state": 0,
			"heater": 0
		},
		"current": [-273.1],
		"state": [0],
		"tools": {
			"active": [],
			"standby": []
		},
		"extra": []
	},
	"time": 596.0,
	"scanner": {
		"status": "D",
		"progress": 0.0
	},
	"spindles": [],
	"laser": 0.0
}
```

### Machine Status

The `status` field provides a single character for the machine status. It may have one of the following values:

| Status character | Status |
| ---------------- | ------ |
| C | Reading configuration file |
| F | Flashing new firmware |
| H | Halted (after emergency stop) |
| O | Off (powered down, low input voltage) |
| D | Pausing print (decelerating) |
| R | Resuming print (after pause) |
| S | Print paused (stopped) |
| M | Simulating a (print) file |
| P | Printing |
| T | Changing tool |
| B | Busy (executing macro, moving) |

Values higher in the table also have a higher priority. So, for example, tool changes are not reported while a print is in progress.

### Coordinates

- `coords/wpl` (optional): Selected workplace (see G10 and G54 to G59.3). May not be present if the board does not support workplaces
- `cords/xyz`: User coordinates of the axes (i.e. with tool offsets applied)
- `coords/machine`: Mchine coordinate of the axes (i.e. without tool offsets applied)
- `coords/extr` Total amount of filament extruded per extruder drive (with extrusion factors applied). See also `extrRaw` in response type 3

### Output

RepRapFirmware may request output via the client application. Only if this is the case, the `output` field is present.

#### Beeps

If no PanelDue is attached, beeps (see M300) are reported via the `beepDuration` and `beepFrequency` fields.
These fields are not present if M300 was not used.

#### Output message

A user may want to output a generic message via M117. If this is the case, the message string is reported via the `message` field.

#### Message boxes

RepRapFirmware allows a user to use message boxes if required. These message boxes may be configured via M291/M292.
The corresponding values can be found in the optional `msgBox` field. See the documentation of M291 for further details.

- `msgBox/controls` is a bitmap of configured axis controls (X = 1, Y = 2, Z = 4 etc.)
- `msgBox/seq` is incremented whenever M291 is used
- `msgBox/timeout` is the time left for displaying this message box or 0.0 if it does not time out

### Temperatures

Slow heaters like `temps/bed`, `temps/chamber`, and `temps/cabinet` (second chamber) have the following properties:

- `current`: Current heater temperature
- `active`: Target temperature when turned on (in active state)
- `standby`: Target temperature when in standby mode
- `state`: State of the heater. See below
- `heater`: Index of the assigned heater

The fields `temps/bed`, `temps/chamber`, and `temps/cabinet` may not be present if no bed and/or chamber is configured.

- `temps/current`: Array of heater temperatures
- `temps/state`: Array of heater states
- `temps/tools/active`: Array holding arrays of configured active heater temperatures. Tools may have multiple heaters assigned
- `temps/tools/standby`: Array holding arrays of configured standby heater temperatures

#### Heater states

| Heater state | Meaning | Description |
| ------------ | ------- | ----------- |
| 0 | off | Heater is turned off |
| 1 | standby | Heater is in standby mode |
| 2 | active | Heater is active |
| 3 | fault | Heater fault occurred |
| 4 | tuning | Heater is being tuned |
| 5 | offline | Remote heater from an expansion board is not available |

#### Extra heaters

Extra heaters represent custom sensor and can be found in `temps/tools/extra`. Every extra heater is reported in the following format:

```
{
	"name": "MCU",
	"temp": 34.7
}
```

### Scanner

RepRapFirmware may support an external interface for 3D scanners. This field is not be present if scanner support is not enabled in the firmware,
hence this field may not be present either. There are two fields for this interface in the status response:

- `status`: Status of the external 3D scanner. See below
- `progress`: Progress of the current operation (0.0 to 100.0)

Possible scanner states:

| Status character | State |
| ---------------- | ----- |
| D | Disconnected |
| I | Idle |
| S | Scanning |
| P | Post processing |
| C | Calibrating |
| U | Uploading |

### Spindles

The `spindles` field may be only present if the machine is in `CNC` mode or if the advanced status respone (type 2) is queried.

It provides a list of configured spindles in the format

```
{
	"current": 0.0,
	"active": 0.0,
	"tool": 0
}
```

where `current` is the current RPM, `active` the target RPM, and `tool` the number of the assigned tool.

### Other fields

- `speeds/requested`: Requested feedrate of the current move
- `speeds/top`: Top feedrate of the current move
- `currentTool`: Selected tool number (not index) or -1 if none is selected
- `params/atxPower`: ATX power state. This may be -1 if ATX power is not configured
- `params/fanPercent`: Fan percent (0.0 to 100.0). If the fan is disabled, this may be negative (-1)
- `params/speedFactor`: Speed factor override. 100% equals 100.0 and this is always greater than 0.0
- `params/extrFactors`: Extrusion factor override values. Values of 100% equals 100.0 and they are always greater than 0.0
- `params/babystep`: Z babystepping amount
- `sensors/probeValue`: ADC reading of the Z probe
- `sensors/probeSecondary` (optional): Seconday probe ADC reading if the probe is modulated. This field is not present if the probe is unmodulated
- `sensors/fanRPM`: Array of fan RPMs, values may be negative if not configured
- `time`: Time in seconds since the machine started. May be used to detect firmware resets
- `laser` (optional): PWM value of the attached laser. This field is only present if the machine is in `Laser` mode

## Advanced Status Response (Type 2)

This status response type provides fields that may change but not as frequently as those in the standard status response.

```
{
	"params": {
		"fanNames": [""],
	},
	"temps": {
		"names": [""}
	},
	...
	"coldExtrudeTemp": 160.0,
	"coldRetractTemp": 90.0,
	"compensation": "None",
	"controllableFans": 0,
	"tempLimit": 290.0,
	"endstops": 0,
	"firmwareName": "RepRapFirmware for Duet 3 v0.6",
	"firmwareVersion": "3.0beta12+1",
	"geometry": "cartesian",
	"axes": 3,
	"totalAxes": 3,
	"axisNames": "XYZ",
	"volumes": 1,
	"mountedVolumes": 0,
	"mode": "FFF",
	"name": "duet3",
	"probe": {
		"threshold": 500,
		"height": 0.70,
		"type": 0
	},
	"tools": [],
	"mcutemp": {
		"min": 24.0,
		"cur": 37.4,
		"max": 37.6
	},
	"vin": {
		"min": 0.2,
		"cur": 0.3,
		"max": 0.3
	},
	"v12": {
		"min": 0.2,
		"cur": 0.2,
		"max": 0.2
	}
}
```

### Geometry

The current geometry is reported as part of the `geometry` field. The following geometries are supported:

| Value | Geometry |
| ----- | -------- |
| cartesian | Cartesian |
| coreXY | CoreXY |
| coreXYU | CoreXYU |
| coreXYUV | CoreXYUV |
| coreXZ | CoreXZ |
| markForged | markForged |
| Hangprinter | Hangprinter |
| delta | Linear delta |
| Polar | Polar |
| Rotary delta | Rotary delta |
| Scara | Scara |

If the geometry is unknown, `unknown` is reported.

### Machine mode

It is possible to choose between `FFF`, `CNC`, and `Laser` mode. As a consequence, the following values are reported as part of the `mode` field:

| Value | Mode |
| ----- | ---- |
| FFF | 3D printing mode |
| CNC | CNC mode |
| Laser | Laser cutting/engraving mode |


### Probe

The `probe` field may not be present if no Z probe is configured. If it is, it provides the following fields:

- `probe/threshold`: Threshold value for the Z probe to be triggered
- `probe/height`: Z height of the Z probe when triggered
- `probe/type`: Probe type. See below

Possible probe types are:

| Probe type | Name |
| ---------- | ---- |
| 0 | No probe |
| 1 | Simple analog probe |
| 2 | Dumb modulated probe |
| 3 | Alternate analog probe |
| 4 | E0 switch **deprecated** |
| 5 | Digital probe |
| 6 | E1 switch **deprecated** |
| 7 | Z switch **deprecated** |
| 8 | Unfiltered digital probe |
| 9 | BLTouch |
| 10 | Z motor stall detection |

### Tool mapping

In order to figure out the mapping between heaters and tools, the tool mapping is reported as part of the advanced status response.
The tool mapping is reported as an array of items like

```
{
	"number": 0,
	"name": "",
	"heaters": [],
	"drives": [],
	"axisMap": [[0],[1]],
	"fans": 1,
	"filament": "PLA",
	"offsets": [0.00, 0.00, 0.00]
}
```

where

- `number`: Tool number (T*n*)
- `name`: Optional tool name or `""` if none is configured
- `heaters`: List of assigned heater indices
- `drives`: List of assigned drives
- `axisMap`: XY axis mapping. Mapped X axes are reported in the first item and mapped Y axes in the second one
- `fans`: Bitmap of the mapped tool fans (controllable by `M106` without `P` parameter)
- `filament` (optional): Name of the loaded filament
- `offsets`: Tool offsets. The size of this array equals the number of visible axes

### Other fields

- `params/fanNames`: Array of custom fan names. Every fan name defaults to `""` if no custom name has been configured
- `temps/names`: Custom names of the heaters. This is `""` for every heater that does not have a custom name
- `coldExtrudeTemp`: Minimum temperature for extrusions
- `coldRetractTemp`: Minimum temperature for retractions
- `compensation`: Current type of bed compensation. May be either `Mesh`, `n Point` (where n is the number of probe points), or `None`
- `controllableFans`: Bitmap of fans that are user-controllable. This excludes thermostatic fans
- `tempLimit`: Maximum allowed heater temperature of the heater protection items. This is used for scaling the temperature chart on the web interface
- `endstops`: Bitmap of currently triggered axis endstops
- `firmwareName`: Name of the firmware **deprecated - see config response**
- `firmwareVersion`: Version of the firmware **deprecated - see config response**
- `axes`: Number of visible axes
- `totalAxes`: Number of total axes
- `axisNames`: Letters of the visible axes
- `volumes`: Total number of supported volumes
- `mountedVolumes`: Bitmap of mounted volumes (1 = drive 0, 2 = drive 2, ...)
- `name`: Hostname of the machine
- `mcutemp` (optional): Minimum, current, and maximum MCU temperatures
- `vin` (optional): Minimum, current, and maximum input voltage
- `v12` (optional): Minimum, current, and maximum voltage on the 12V rail

## Print Status Response (Type 3)

This status response type provides extra details about the file being processed. The format is

```
{
	...
	"currentLayer": 0,
	"currentLayerTime": 0.0,
	"extrRaw": [],
	"fractionPrinted": 0.0,
	"filePosition": 0,
	"firstLayerDuration": 0.0,
	"firstLayerHeight": 0.00,
	"printDuration": 0.0,
	"warmUpDuration": 0.0,
	"timesLeft": {
		"file": 0.0,
		"filament": 0.0,
		"layer": 0.0
	}
}
```

where the fields are:

- `currentLayer`: Number of the current layer being printed
- `currentLayerTime`: Time of the current layer
- `extrRaw`: Total amount of extruded filament without extrusion multipliers applied. This is useful to estimate the print progress
- `fractionPrinted`: Fraction of the file printed on a scale of 0.0 to 100.0. This equals `filePosition / fileSize`
- `filePosition`: Byte position of the file being printed
- `firstLayerDuration`: Duration of the first layer or 0.0 if unknown
- `firstLayerHeight`: Height of the first layer
- `printDuration`: Total print duration (excluding pause times) 
- `warmUpDuration`: Time needed to warm up the heaters
- `timesLeft/file`: Estimation for the time left based on the file consumption
- `timesLeft/filament`: Estimation for the time left based on the filament consumption
- `timesLeft/layer`: Estimation for the time left based on the layer times

## Configuration response

In addition to the responses above, the config response provides values that are not expected to change unless the machine is reconfigured.
Note that a user may choose to reconfigure the machine at any time. The config response comes in the following format:

```
{
	"axisMins": [0.0, 0.0, 0.0],
	"axisMaxes": [220.0, 200.0, 180.0],
	"accelerations": [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0],
	"currents": [800.0, 1000.0, 800.0, 1000.0, 800.0, 800.0, 8000, 800.0],
	"firmwareElectronics": "Duet 3 MB6HC",
	"firmwareName": "RepRapFirmware",
	"boardName": "MB6HC",
	"firmwareVersion": "3.0beta12+1",
	"dwsVersion": "1.24",
	"firmwareDate": "2019-11-05b1",
	"idleCurrentFactor": 35.0,
	"idleTimeout": 30.0,
	"minFeedrates": [10.0, 10.0, 0.5, 0.33, 0.33, 0.33, 0.33, 0.33],
	"maxFeedrates": [250.0, 250.0, 3.0, 60.0, 60.0, 60.0, 60.0, 60.0]
}
```

The following fields are reported:

- `axisMins`: Minima of the visible axes
- `axisMaxes`: Maxima of the visible axes
- `accelerations`: Accelerations of the drives
- `currents`: Configured motor currents
- `firmwareElectronics`: Name of the firmware electronics
- `firmwareName`: Name of the firmware (usually RepRapFirmware)
- `boardName` (optional): Short name of the board, only applicable for Duet 3. May be `MB6HC` for Duet v0.6 or `MBP05` for the Duet v0.5 prototype
- `firmwareVersion`: Version string of the firmware
- `dwsVersion` (optional): Version of the DuetWiFiSocketServer (only Duet WiFi)
- `firmwareDate`: Indicates when the firmware was built
- `idleCurrentFactor`: Motor currents are reduced by this factor when the motors are idle
- `idleTimeout`: Motor current reduction is performed after this time in seconds
- `minFeedrates`: Minimum feedrates of the drives
- `maxFeedrates`: Maximum feedrates of the drives
