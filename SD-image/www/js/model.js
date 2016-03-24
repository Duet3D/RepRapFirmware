/* Interface logic for the Duet Web Control v1.11
 * 
 * written by Christian Hammacher
 * 
 * licensed under the terms of the GPL v2
 * see http://www.gnu.org/licenses/gpl-2.0.html
 */

var jsVersion = "1.11";
var sessionPassword = "reprap";
var translationWarning = false;		// Set this to "true" if you want to look for missing translation entries

/* Settings */

var settings = {
	autoConnect: true,				// automatically connect once the page has loaded

	updateInterval: 250,			// in ms
	haltedReconnectDelay: 5000,		// in ms
	updateReconnectDelay: 20000,	// in ms
	extendedStatusInterval: 10,		// nth status request will include extended values
	maxRequestTime: 8000,			// maximum time to wait for a status response in ms
	notificationTimeout: 5000,		// in ms

	halfZMovements: false,			// use half Z movements
	logSuccess: false,				// log all sucessful G-Codes in the console
	uppercaseGCode: true,			// convert G-Codes to upper-case before sending them
	useKiB: true,					// display file sizes in KiB instead of KB
	showFanControl: false,			// show fan controls
	showFanRPM: false,				// show fan RPM in sensors
	showATXControl: false,			// show ATX control
	confirmStop: false,				// ask for confirmation when pressing Emergency STOP
	useDarkTheme: false,			// load dark theme by Fotomas
	moveFeedrate: 6000,				// in mm/min

	defaultActiveTemps: [0, 170, 195, 205, 210, 235, 245, 250],
	defaultStandbyTemps: [0, 95, 120, 155, 170],
	defaultBedTemps: [0, 55, 65, 90, 110, 120],
	defaultGCodes: [
		["M0", "Stop"],
		["M1", "Sleep"],
		["M84", "Motors Off"],
		["M561", "Disable bed compensation"]
	],

	language: "en",

	webcamURL: "",
	webcamInterval: 5000			// in ms
};
var defaultSettings = jQuery.extend(true, {}, settings);		// need to do this to get a valid copy

/* Variables */

var isConnected = false, justConnected, isUploading, updateTaskLive, stopUpdating;
var ajaxRequests = [], extendedStatusCounter, lastStatusResponse, configResponse, configFile;
var lastSentGCode;

var fileInfo, currentLayerTime, maxLayerTime, lastLayerPrintDuration;

var geometry, probeSlowDownValue, probeTriggerValue;
var heatedBed, chamber, numHeads, numExtruderDrives, toolMapping;
var coldExtrudeTemp, coldRetractTemp;

var recordedBedTemperatures, recordedChamberTemperatures, recordedHeadTemperatures, layerData;
var isPrinting, isPaused, printHasFinished;

var currentPage = "control", refreshTempChart = false, refreshPrintChart = false, waitingForPrintStart;
var translationData, darkThemeInclude;

var currentGCodeDirectory, knownGCodeFiles, gcodeUpdateIndex, gcodeLastDirectory;
var currentMacroDirectory, nownMacroFiles, macroUpdateIndex, macroLastDirectory;

var fanSliderActive, speedSliderActive, extrSliderActive;

function resetGuiData() {
	setPauseStatus(false);
	setPrintStatus(false);
	setGeometry("cartesian");

	justConnected = isUploading = updateTaskLive = false;
	stopUpdating = true;
	fileInfo = lastStatusResponse = configResponse = configFile = undefined;

	lastSentGCode = "";

	probeSlowDownValue = probeTriggerValue = undefined;
	heatedBed = 1;
	chamber = 0;
	numHeads = 2;			// 6 Heaters max, only 2 are visible on load
	numExtruderDrives = 2;	// 6 Extruder Drives max, only 2 are visible on load

	coldExtrudeTemp = 160;
	coldRetractTemp = 90;

	recordedBedTemperatures = [];
	recordedChamberTemperatures = [];
	layerData = [];
	recordedHeadTemperatures = [[], [], [], [], [], []];

	maxLayerTime = lastLayerPrintDuration = 0;

	setToolMapping(undefined);

	knownGCodeFiles = knownMacroFiles = [];
	gcodeUpdateIndex = macroUpdateIndex = -1;
	currentGCodeDirectory = "/gcodes";
	currentMacroDirectory = "/macros";

	waitingForPrintStart = fanSliderActive = speedSliderActive = extrSliderActive = false;
}

/* Connect / Disconnect */

function connect(password, regularConnect) {
	// Close all notifications before we connect...
	if (regularConnect) {
		$.notifyClose();
	}

	$(".btn-connect").removeClass("btn-info").addClass("btn-warning disabled").find("span:not(.glyphicon)").text(T("Connecting..."));
	$(".btn-connect span.glyphicon").removeClass("glyphicon-log-in").addClass("glyphicon-transfer");
	$.ajax("rr_connect?password=" + password, {
		dataType: "json",
		error: function() {
			showMessage("danger", T("Error"), T("Could not establish a connection to the Duet firmware! Please check your settings and try again."), 0);
			$(".btn-connect").removeClass("btn-warning disabled").addClass("btn-info").find("span:not(.glyphicon)").text(T("Connect"));
			$(".btn-connect span.glyphicon").removeClass("glyphicon-transfer").addClass("glyphicon-log-in");
		},
		success: function(data) {
			if (data.err == 2) {		// Looks like the firmware ran out of HTTP sessions
				showMessage("danger", T("Error"), T("Could not connect to Duet, because there are no more HTTP sessions available."), 0);
				$(".btn-connect").removeClass("btn-warning disabled").addClass("btn-info").find("span:not(.glyphicon)").text(T("Connect"));
				$(".btn-connect span.glyphicon").removeClass("glyphicon-transfer").addClass("glyphicon-log-in");
			}
			else if (regularConnect)
			{
				if (data.err == 0) {	// No password authentication required
					sessionPassword = password;
					postConnect();
				}
				else {					// We can connect, but we need a password first
					showPasswordPrompt();
				}
			}
			else {
				if (data.err == 0) {	// Connect successful
					sessionPassword = password;
					postConnect();
				} else {
					showMessage("danger", T("Error"), T("Invalid password!"), 0);
					$(".btn-connect").removeClass("btn-warning disabled").addClass("btn-info").find("span:not(.glyphicon)").text(T("Connect"));
					$(".btn-connect span.glyphicon").removeClass("glyphicon-transfer").addClass("glyphicon-log-in");
				}
			}
		}
	});
}

function postConnect() {
	log("success", "<strong>" + T("Connection established!") + "</strong>");

	isConnected = justConnected = true;
	extendedStatusCounter = settings.extendedStatusInterval; // ask for extended status response on first poll

	startUpdates();
	if (currentPage == "files") {
		updateGCodeFiles();
	} else if (currentPage == "control" || currentPage == "macros") {
		updateMacroFiles();
	} else if (currentPage == "settings") {
		getConfigResponse();
		if ($("#page_config").is(".active")) {
			getConfigFile();
		}
	}

	$(".btn-connect").removeClass("btn-warning disabled").addClass("btn-success").find("span:not(.glyphicon)").text(T("Disconnect"));
	$(".btn-connect span.glyphicon").removeClass("glyphicon-transfer").addClass("glyphicon-log-out");

	enableControls();
	validateAddTool();
}

function disconnect() {
	if (isConnected) {
		log("danger", "<strong>" + T("Disconnected.") + "</strong>");
	}
	isConnected = false;

	$(".btn-connect").removeClass("btn-success").addClass("btn-info").find("span:not(.glyphicon)").text(T("Connect"));
	$(".btn-connect span.glyphicon").removeClass("glyphicon-log-out").addClass("glyphicon-log-in");

	$.ajax("rr_disconnect", { dataType: "json", global: false });
	ajaxRequests.forEach(function(request) {
		request.abort();
	});
	ajaxRequests = [];

	resetGuiData();
	resetGui();
	updateGui();

	disableControls();
	validateAddTool();
	setToolMapping(undefined);
}

/* AJAX */

function startUpdates() {
	stopUpdating = false;
	if (!updateTaskLive) {
		updateStatus();
	}
}

function stopUpdates() {
	stopUpdating = updateTaskLive;
}

function updateStatus() {
	if (stopUpdating) {
		updateTaskLive = false;
		return;
	}
	updateTaskLive = true;

	var ajaxRequest = "rr_status";
	if (extendedStatusCounter >= settings.extendedStatusInterval || (currentPage == "settings" && (!isPrinting || extendedStatusCounter != 0))) {
		extendedStatusCounter = 0;
		ajaxRequest += "?type=2";
	} else if (isPrinting) {
		ajaxRequest += "?type=3";
	} else {
		ajaxRequest += "?type=1";
	}
	extendedStatusCounter++;

	$.ajax(ajaxRequest, {
		dataType: "json",
		success: function(status) {
			// Don't process this one if we're no longer connected
			if (!isConnected) {
				return;
			}
			var needGuiUpdate = false;

			/*** Extended status response ***/

			// Cold Extrusion + Retraction Temperatures
			if (status.hasOwnProperty("coldExtrudeTemp")) {
				coldExtrudeTemp = status.coldExtrudeTemp;
			}
			if (status.hasOwnProperty("coldRetractTemp")) {
				coldRetractTemp = status.coldRetractTemp;
			}

			// Endstops
			if (status.hasOwnProperty("endstops")) {
				var endstops = status.endstops;
				for(i=0; i<9; i++) {
					var displayText;
					if (endstops & (1 << i)) {
						displayText = T("Yes");
					} else {
						displayText = T("No");
					}

					$("#tr_drive_" + i + " > td:nth-child(2)").text(displayText);
				}
			}

			// Printer Geometry
			if (status.hasOwnProperty("geometry")) {
				setGeometry(status.geometry);
			}

			// Machine Name
			if (status.hasOwnProperty("name")) {
				setTitle(status.name);
			}

			// Probe Parameters (maybe hide probe info for type 0 someday?)
			if (status.hasOwnProperty("probe")) {
				probeTriggerValue = status.probe.threshold;
				probeSlowDownValue = probeTriggerValue * 0.9;	// see Platform::Stopped in dc42/ch firmware forks

				var probeType;
				switch (status.probe.type) {
					case 0:
						probeType = T("Switch (0)");
						break;
					case 1:
						probeType = T("Unmodulated (1)");
						break;
					case 2:
						probeType = T("Modulated (2)");
						break;
					case 3:
						probeType = T("Alternative (3)");
						break;
					case 4:
						probeType = T("Two Switches (4)");
						break;
					default:
						probeType = T("Unknown ({0})", status.probe.type);
						break;
				}
				$("#dd_probe_type").text(probeType);
				$("#dd_probe_height").text(status.probe.height + " mm");
				$("#dd_probe_value").text(probeTriggerValue);
			}

			// Tool Mapping
			if (status.hasOwnProperty("tools")) {
				setToolMapping(status.tools);
			}

			// CPU temperature
			if (status.hasOwnProperty("cputemp")) {
				$(".cpu-temp").removeClass("hidden");
				$("#td_cputemp").html(status.cputemp.toFixed(1) + " °C");
			}

			/*** Default status response ***/

			// Status
			var printing = false, paused = false;
			switch (status.status) {
				case 'F':	// Flashing new firmware
					setStatusLabel("Updating", "success");
					break;

				case 'H':	// Halted
					setStatusLabel("Halted", "danger");
					break;

				case 'D':	// Pausing / Decelerating
					setStatusLabel("Pausing", "warning");
					printing = true;
					paused = true;
					break;

				case 'S':	// Paused / Stopped
					setStatusLabel("Paused", "info");
					printing = true;
					paused = true;
					break;

				case 'R':	// Resuming
					setStatusLabel("Resuming", "warning");
					printing = true;
					paused = true;
					break;

				case 'P':	// Printing
					setStatusLabel("Printing", "success");
					printing = true;
					break;

				case 'B':	// Busy
					setStatusLabel("Busy", "warning");
					break;

				case 'I':	// Idle
					setStatusLabel("Idle", "default");
					break;
			}
			setPrintStatus(printing);
			setPauseStatus(paused);
			justConnected = false;

			// Set homed axes
			setAxesHomed(status.coords.axesHomed);

			// Update extruder drives
			if (status.coords.extr.length != numExtruderDrives) {
				numExtruderDrives = status.coords.extr.length;
				needGuiUpdate = true;
			}
			for(var i=1; i<=numExtruderDrives; i++) {
				$("#td_extr_" + i).html(status.coords.extr[i - 1].toFixed(1));
			}
			if (numExtruderDrives > 0) {
				$("#td_extr_total").html(status.coords.extr.reduce(function(a, b) { return a + b; }));
			} else {
				$("#td_extr_total").html("n/a");
			}

			// XYZ coordinates
			if (geometry == "delta" && !status.coords.axesHomed[0]) {
				$("#td_x, #td_y, #td_z").html("n/a");
			} else {
				$("#td_x").html(status.coords.xyz[0].toFixed(2));
				$("#td_y").html(status.coords.xyz[1].toFixed(2));
				$("#td_z").html(status.coords.xyz[2].toFixed(2));
			}

			// Current Tool
			if (lastStatusResponse != undefined && lastStatusResponse.currentTool != status.currentTool) {
				var btn = $("div.panel-body[data-tool='" + lastStatusResponse.currentTool + "'] > button.btn-select-tool");
				btn.attr("title", T("Select this tool"));
				btn.find("span:last-child").text(T("Select"));
				btn.find("span.glyphicon").removeClass("glyphicon-remove").addClass("glyphicon-pencil");

				btn = $("div.panel-body[data-tool='" + status.currentTool + "'] > button.btn-select-tool");
				btn.attr("title", T("Select this tool"));
				btn.find("span.glyphicon").removeClass("glyphicon-remove").addClass("glyphicon-pencil");
				btn.find("span:last-child").text(T("Deselect"));
			}

			// Output
			if (status.hasOwnProperty("output")) {
				if (status.output.hasOwnProperty("beepDuration") && status.output.hasOwnProperty("beepFrequency")) {
					beep(status.output.beepFrequency, status.output.beepDuration);
				}
				if (status.output.hasOwnProperty("message")) {
					showMessage("info", T("Message from Duet firmware"), status.output.message, 0);
				}
			}

			// ATX Power
			setATXPower(status.params.atxPower);

			// Fan Control
			var newFanValue = (status.params.fanPercent.constructor === Array) ? status.params.fanPercent[0] : status.params.fanPercent;
			if (!fanSliderActive && (lastStatusResponse == undefined || $("#slider_fan_print").slider("getValue") != newFanValue)) {
				if ($("#override_fan").is(":checked") && settings.showFanControl) {
					sendGCode("M106 S" + ($("#slider_fan_print").slider("getValue") / 100.0));
				} else {
					$("#slider_fan_control").slider("setValue", newFanValue);
					$("#slider_fan_print").slider("setValue", newFanValue);
				}
			}

			// Speed Factor
			if (!speedSliderActive && (lastStatusResponse == undefined || $("#slider_speed").slider("getValue") != status.params.speedFactor)) {
				$("#slider_speed").slider("setValue", status.params.speedFactor);
			}
			if (!extrSliderActive) {
				for(var i=0; i<status.params.extrFactors.length; i++) {
					var extrSlider = $("#slider_extr_" + (i + 1));
					if (lastStatusResponse == undefined || extrSlider.slider("getValue") != status.params.extrFactors) {
						extrSlider.slider("setValue", status.params.extrFactors[i]);
					}
				}
			}

			// Fetch the latest G-Code response from the server
			if (lastStatusResponse == undefined || lastStatusResponse.seq != status.seq) {
				$.ajax("rr_reply", {
					dataType: "html",
					success: function(response) {
						response = response.trim();
						if ((response != "") || (lastSentGCode != "" && settings.logSuccess)) {
							// What kind of reply are we dealing with?
							var style = (response == "") ? "success" : "info", isError = false;
							if (response.match("^Error: ") != null) {
								style = "warning";
								isError = true;
							}

							// Prepare line breaks for HTML
							lastSentGCode = lastSentGCode.trim().replace(/\n/g, "<br/>");
							response = response.replace(/\n/g, "<br/>");

							// Log this message in the G-Code console
							var prefix = (lastSentGCode != "") ? "<strong>" + lastSentGCode + "</strong><br/>" : "";
							log(style, prefix + response.replace(/Error:/g, "<strong>Error:</strong>"));

							// If the console isn't visible, show a notification too
							if (currentPage != "console") {
								if (lastSentGCode != "") {
									if (isError) {
										showMessage(style, T("{0} has returned an error:", lastSentGCode), response.substring(6).trim());
									} else if (response != "") {
										showMessage(style, lastSentGCode, response);
									} else {
										showMessage(style, "", "<strong>" + lastSentGCode + "</strong>");
									}
								} else {
									showMessage(style, "", response.replace(/Error:/g, "<strong>Error:</strong>"));
								}
							}
						}

						// Reset info about last sent G-Code again
						lastSentGCode = "";
					}
				});
			}

			// Sensors
			setProbeValue(status.sensors.probeValue, status.sensors.probeSecondary);
			$("#td_fanrpm").html(status.sensors.fanRPM);

			// Heated bed
			var bedTemp = undefined;
			if (status.temps.hasOwnProperty("bed")) {
				if (!heatedBed) {
					heatedBed = 1;
					needGuiUpdate = true;
				}

				bedTemp = status.temps.bed.current;
				setCurrentTemperature("bed", status.temps.bed.current);
				setTemperatureInput("bed", status.temps.bed.active, 1);
				setHeaterState("bed", status.temps.bed.state, status.currentTool);
			} else if (heatedBed) {
				heatedBed = 0;
				needGuiUpdate = true;
			}

			// Chamber
			var chamberTemp = undefined;
			if (status.temps.hasOwnProperty("chamber")) {
				if (!chamber)
				{
					chamber = 1;
					needGuiUpdate = true;
				}

				chamberTemp = status.temps.chamber.current;
				setCurrentTemperature("chamber", chamberTemp);
				setTemperatureInput("chamber", status.temps.chamber.active, 1);
				setHeaterState("chamber", status.temps.chamber.state, status.currentTool);
			} else if (chamber) {
				chamber = 0;
				needGuiUpdate = true;
			}

			// Heads
			if (status.temps.heads.current.length != numHeads) {
				numHeads = status.temps.heads.current.length;
				needGuiUpdate = true;
			}
			for(var i=0; i<status.temps.heads.current.length; i++) {
				setCurrentTemperature(i + 1, status.temps.heads.current[i]);
				setTemperatureInput(i + 1, status.temps.heads.active[i], 1);
				setTemperatureInput(i + 1, status.temps.heads.standby[i], 0);
				setHeaterState(i + 1, status.temps.heads.state[i], status.currentTool);
			}
			recordHeaterTemperatures(bedTemp, chamberTemp, status.temps.heads.current);

			/*** Print status response ***/

			if (status.hasOwnProperty("fractionPrinted") && fileInfo != undefined) {
				var printJustFinished = false;
				if (!printHasFinished) {
					var progress = 100, progressText = [];

					// Get the current layer progress text
					if (fileInfo.height > 0 && fileInfo.layerHeight > 0) {
						var numLayers;
						if (status.firstLayerHeight > 0) {
							numLayers = ((fileInfo.height - status.firstLayerHeight) / fileInfo.layerHeight) + 1;
						} else {
							numLayers = (fileInfo.height / fileInfo.layerHeight);
						}
						numLayers = numLayers.toFixed();
						progressText.push(T("Layer: {0} of {1}", status.currentLayer, numLayers));
					}

					// Try to calculate the progress by checking the filament usage
					if (fileInfo.filament.length > 0) {
						var totalFileFilament = (fileInfo.filament.reduce(function(a, b) { return a + b; })).toFixed(1);
						var totalRawFilament = (status.extrRaw.reduce(function(a, b) { return a + b; })).toFixed(1);
						progress = ((totalRawFilament / totalFileFilament) * 100.0).toFixed(1);

						var remainingFilament = (totalFileFilament - totalRawFilament);
						if (progress < 0) {
							progress = 0;
						} else if (progress > 100) {
							progress = 100;
							totalRawFilament = totalFileFilament;
							remainingFilament = 0;
							printJustFinished = printHasFinished = true;
						}
						progressText.push(T("Filament Usage: {0}mm of {1}mm", totalRawFilament, totalFileFilament));

						// TODO: Make this optional
						progressText[progressText.length - 1] += " " + T("({0}mm remaining)", remainingFilament.toFixed(1));

					}
					// Otherwise by comparing the current Z position to the total height
					else if (fileInfo.height > 0) {
						progress = ((status.coords.xyz[2] / fileInfo.height) * 100.0).toFixed(1);
						if (progress < 0) {
							progress = 0;
						} else if (progress > 100) {
							progress = 100;
							printJustFinished = printHasFinished = true;
						}
					}
					// Use the file-based progress as a fallback option
					else {
						progress = status.fractionPrinted;
						if (progress < 0) {
							progress = 100;
							printJustFinished = printHasFinished = true;
						}
					}
					setProgress(progress, T("Printing {0}, {1}% Complete", fileInfo.fileName, progress), 
							(progressText.length > 0) ? progressText.reduce(function(a, b) { return a + ", " + b; }) : "");
				}

				// Print Chart
				if (status.currentLayer > 1) {
					var realPrintTime = (status.printDuration - status.warmUpDuration - status.firstLayerDuration);
					if (layerData.length == 0) {
						if (status.currentLayer > 2) {						// add avg values on reconnect
							addLayerData(status.firstLayerDuration, false);
							realPrintTime -= status.currentLayerTime;
							for(var layer=2; layer<status.currentLayer; layer++) {
								addLayerData(realPrintTime / (status.currentLayer - 1), false);
							}
							drawPrintChart();
						} else {											// else only the first layer is complete
							addLayerData(status.firstLayerDuration, true);
						}

						if (status.currentLayer == 2) {
							lastLayerPrintDuration = 0;
						} else {
							lastLayerPrintDuration = realPrintTime;
						}
					} else if (printJustFinished || status.currentLayer - 1 > layerData.length) {
						addLayerData(realPrintTime - lastLayerPrintDuration, true);
						lastLayerPrintDuration = realPrintTime;
					}
				}

				// Warm-Up Time
				if (status.warmUpDuration > 0) {
					if (status.warmUpDuration < 0.5) {
						$("#td_warmup_time").html(T("none"));
					} else {
						$("#td_warmup_time").html(convertSeconds(status.warmUpDuration));
					}
				} else if (!printHasFinished) {
					$("#td_warmup_time").html("n/a");
				}

				// Current Layer Time
				if (!printHasFinished) {
					if (status.currentLayerTime > 0 || status.currentLayer > 1) {
						currentLayerTime = status.currentLayerTime;
						$("#td_layertime").html(convertSeconds(status.currentLayerTime));
					} else if (status.firstLayerDuration > 0) {
						currentLayerTime = status.firstLayerDuration;
						$("#td_layertime").html(convertSeconds(status.firstLayerDuration));
					} else {
						$("#td_layertime").html("n/a");
					}
				} else {
					$("#td_layertime").html("n/a");
				}

				// Print Duration
				if (status.printDuration > 0) {
					$("#td_print_duration").html(convertSeconds(status.printDuration));
				} else if (!printHasFinished) {
					$("#td_print_duration").html("n/a");
				}

				// First Layer Height (maybe we need to update the layer height info)
				if (status.firstLayerHeight > 0 && $("#dd_layer_height").html().indexOf("/") == -1)
				{
					$("#dd_layer_height").html(status.firstLayerHeight + " mm / " + $("#dd_layer_height").html());
				}

				// Print Estimations
				if (printHasFinished) {
					["filament", "layer", "file"].forEach(function(id) {
						if ($("#tl_" + id).html() != "n/a") {
							$("#tl_" + id).html("00s");
							$("#et_" + id).html((new Date()).toLocaleTimeString());
						}
					});
				} else {
					if (fileInfo.filament.length > 0) {
						setTimeLeft("filament", status.timesLeft.filament);
					} else {
						setTimeLeft("filament", undefined);
					}
					setTimeLeft("file", status.timesLeft.file);
					if (fileInfo.height > 0) {
						setTimeLeft("layer", status.timesLeft.layer);
					} else {
						setTimeLeft("layer", undefined);
					}
				}
			}

			// Update the GUI when we have processed the whole status response
			if (needGuiUpdate) {
				updateGui();
			}
			drawTemperatureChart();

			// Set timer for next status update
			if (status.status == 'F') {
				isConnected = updateTaskLive = false;
				log("info", "<strong>" + T("Updating Firmware...") + "</strong>");
				showUpdateMessage();
				setTimeout(function() {
					connect(sessionPassword, false);
				}, settings.updateReconnectDelay);
			} else if (status.status == 'H') {
				isConnected = updateTaskLive = false;
				log("danger", "<strong>" + T("Emergency Stop!") + "</strong>");
				setTimeout(function() {
					connect(sessionPassword, false);
				}, settings.haltedReconnectDelay);
			} else {
				setTimeout(updateStatus, settings.updateInterval);
			}

			// Save the last status response
			lastStatusResponse = status;
		}
	});
}

function getConfigFile() {
	// Lock the config file as long as the request is being processed
	$("#text_config").prop("readonly", true);

	// Request config file from the Duet
	$.ajax("rr_configfile", {
		dataType: "html",
		global: false,
		success: function(response) {
			if (response != "") {
				configFile = response;
				$("#div_config > h1").addClass("hidden");
				$("#text_config").val(response).prop("readonly", false).removeClass("hidden").trigger("input");
				$("#row_save_settings").removeClass("hidden");
			}
		}
	});
}

function getConfigResponse() {
	$.ajax("rr_config", {
		dataType: "json",
		success: function(response) {
			configResponse = response;
			$("#firmware_name").text(response.firmwareName);
			$("#firmware_version").text(response.firmwareVersion + " (" + response.firmwareDate + ")");

			if (response.hasOwnProperty("configFile")) {
				$("#div_config > h1").addClass("hidden");
				$("#text_config").removeClass("hidden").prop("readonly", true).val(response.configFile).trigger("input");
			} else {
				$("#div_config > h1").removeClass("hidden").text(T("loading"));
				$("#text_config").addClass("hidden");
			}

			for(var drive = 0; drive < response.accelerations.length; drive++) {
				if (drive < response.axisMins.length) {
					$("#tr_drive_" + drive + " > td:nth-child(3)").text(response.axisMins[drive] + " mm");
				}
				if (drive < response.axisMaxes.length) {
					$("#tr_drive_" + drive + " > td:nth-child(4)").text(response.axisMaxes[drive] + " mm");
				}
				$("#tr_drive_" + drive + " > td:nth-child(5)").text(response.minFeedrates[drive] + " mm/s");
				$("#tr_drive_" + drive + " > td:nth-child(6)").text(response.maxFeedrates[drive] + " mm/s");
				$("#tr_drive_" + drive + " > td:nth-child(7)").text(response.accelerations[drive] + " mm/s²");
				if (response.hasOwnProperty("currents")) {
					$("#tr_drive_" + drive + " > td:nth-child(8)").text(response.currents[drive] + " mA");
				}
			}
			if (response.hasOwnProperty("idleCurrentFactor")) {
				$("#dd_idle_current").text(response.idleCurrentFactor.toFixed(0) + "%");
			}
			if (response.hasOwnProperty("idleTimeout")) {
				var idleTimeoutText = (response.idleTimeout == 0) ? T("never") : convertSeconds(response.idleTimeout);
				$("#dd_idle_timeout").text(idleTimeoutText);
			}
		}
	});
}

function updateGCodeFiles() {
	if (!isConnected) {
		gcodeUpdateIndex = -1;
		$(".span-refresh-files").addClass("hidden");
		$("#table_gcode_files").css("cursor", "");
		clearGCodeDirectory();
		clearGCodeFiles();
		return;
	}

	if (gcodeUpdateIndex == -1) {
		stopUpdates();
		gcodeUpdateIndex = 0;
		gcodeLastDirectory = undefined;
		clearGCodeFiles();

		$.ajax("rr_files?dir=" + encodeURIComponent(currentGCodeDirectory) + "&flagDirs=1", {
			dataType: "json",
			success: function(response) {
				if (isConnected) {
					knownGCodeFiles = response.files.sort(function (a, b) {
						return a.toLowerCase().localeCompare(b.toLowerCase());
					});

					var i = 0;
					while (i < knownGCodeFiles.length) {
						if (knownGCodeFiles[i][0] == '*')
						{
							var dirName = knownGCodeFiles[i].substring(1);
							setGCodeDirectoryItem(addGCodeFile(dirName));
							knownGCodeFiles.splice(i, 1);
						}
						else
						{
							addGCodeFile(knownGCodeFiles[i]);
							i++;
						}
					}

					if (knownGCodeFiles.length == 0) {
						if (currentPage == "files") {
							$(".span-refresh-files").removeClass("hidden");
						}
						startUpdates();
					} else {
						$("#table_gcode_files").css("cursor", "wait");
						updateGCodeFiles();
					}
				}
			}
		});
	} else if (gcodeUpdateIndex < knownGCodeFiles.length) {
		var row = $("#table_gcode_files tr[data-item='" + knownGCodeFiles[gcodeUpdateIndex] + "']");
		$.ajax("rr_fileinfo?name=" + encodeURIComponent(currentGCodeDirectory + "/" + knownGCodeFiles[gcodeUpdateIndex]), {
			dataType: "json",
			row: row,
			dir: currentGCodeDirectory,
			success: function(response) {
				if (!isConnected || this.dir != currentGCodeDirectory) {
					return;
				}
				gcodeUpdateIndex++;

				if (response.err == 0) {	// File
					setGCodeFileItem(this.row, response.size, response.height, response.firstLayerHeight, response.layerHeight, response.filament, response.generatedBy);
				} else {					// Directory
					setGCodeDirectoryItem(this.row);
				}

				if (currentPage == "files") {
					if (gcodeUpdateIndex >= knownGCodeFiles.length) {
						$(".span-refresh-files").removeClass("hidden");
						$("#table_gcode_files").css("cursor", "");
						startUpdates();
					} else {
						updateGCodeFiles();
					}
				} else {
					startUpdates();
				}
			}
		});
	}
}
function updateMacroFiles() {
	if (!isConnected) {
		macroUpdateIndex = -1;
		$(".span-refresh-macros").addClass("hidden");
		$("#table_macro_files").css("cursor", "");
		clearMacroFiles();
		return;
	}

	if (macroUpdateIndex == -1) {
		stopUpdates();
		macroUpdateIndex = 0;
		macroLastDirectory = undefined;
		clearMacroFiles();

		$.ajax("rr_files?dir=" + encodeURIComponent(currentMacroDirectory) + "&flagDirs=1", {
			dataType: "json",
			success: function(response) {
				if (isConnected) {
					knownMacroFiles = response.files.sort(function (a, b) {
						return a.toLowerCase().localeCompare(b.toLowerCase());
					});

					var i = 0;
					while (i < knownMacroFiles.length) {
						if (knownMacroFiles[i][0] == '*')
						{
							var dirName = knownMacroFiles[i].substring(1);
							setMacroDirectoryItem(addMacroFile(dirName));
							knownMacroFiles.splice(i, 1);
						}
						else
						{
							addMacroFile(knownMacroFiles[i]);
							i++;
						}
					}

					if (knownMacroFiles.length == 0) {
						if (currentPage == "macros") {
							$(".span-refresh-macros").removeClass("hidden");
						}
						startUpdates();
					} else {
						$("#table_macro_files").css("cursor", "wait");
						updateMacroFiles();
					}
				}
			}
		});
	} else if (macroUpdateIndex < knownMacroFiles.length) {
		var row = $("#table_macro_files tr[data-item='" + knownMacroFiles[macroUpdateIndex] + "']");
		$.ajax("rr_fileinfo?name=" + encodeURIComponent(currentMacroDirectory + "/" + knownMacroFiles[macroUpdateIndex]), {
			dataType: "json",
			row: row,
			dir: currentMacroDirectory,
			success: function(response) {
				if (!isConnected || this.dir != currentMacroDirectory) {
					return;
				}
				macroUpdateIndex++;

				if (response.err == 0) {	// File
					setMacroFileItem(this.row, response.size);
				} else {					// Directory
					setMacroDirectoryItem(this.row);
				}

				if (macroUpdateIndex >= knownMacroFiles.length) {
					if (currentPage == "macros") {
						$(".span-refresh-macros").removeClass("hidden");
					}
					$("#table_macro_files").css("cursor", "");
					startUpdates();
				} else if (currentPage == "control" || currentPage == "macros") {
					updateMacroFiles();
				} else {
					startUpdates();
				}
			}
		});
	}
}

// Send G-Code directly to the firmware
function sendGCode(gcode, fromInput) {
	lastSentGCode = gcode;

	// Although rr_gcode gives us a JSON response, it doesn't provide any results.
	// We only need to worry about an AJAX error event.
	$.ajax("rr_gcode?gcode=" + encodeURIComponent(gcode), {
		dataType: "json"
	});
}

/* AJAX Events */

$(document).ajaxSend(function(event, jqxhr, settings) {
	ajaxRequests.push(jqxhr);
});

$(document).ajaxComplete(function(event, jqxhr, settings) {
	ajaxRequests = $.grep(ajaxRequests, function(item) { item != jqxhr; });
});

$(document).ajaxError(function(event, jqxhr, settings, thrownError) {
	if (thrownError == "abort") {
		// Ignore this error if this request was cancelled intentionally
		return;
	}

	if (isConnected) {
		var msg = T("An AJAX error has been reported, so the current session has been terminated.<br/><br/>Please check if your printer is still on and try to connect again.");
		if (thrownError != "") {
			msg += "<br/><br/>" + T("Error reason: {0}", thrownError);
		}
		showMessage("danger", T("Communication Error"), msg, 0);

		disconnect();

		// Try to log the faulty response to console
		if (jqxhr.responseText != undefined) {
			console.log("Error! The following JSON response could not be parsed:");
			console.log(jqxhr.responseText);
		}
	}
});

/* Settings */

function checkBoundaries(value, defaultValue, minValue, maxValue) {
	if (isNaN(value)) {
		return defaultValue;
	}
	if (value < minValue) {
		return minValue;
	}
	if (value > maxValue) {
		return maxValue;
	}

	return value;
}

function loadSettings(usingCookie) {
	var loadedSettings;

	// Older versions of DWC used cookies to store the settings. This is disadvantageous for multiple reasons.
	// That's why we try to migrate these settings to localStorage, which allows a smaller HTTP request footprint. 
	if (!usingCookie) {
		if (localStorage.getItem("settings") == null) {
			var cookieScript = document.createElement("script");
			cookieScript.type = "text/javascript";
			cookieScript.src = "js/jquery.cookie.min.js";
			cookieScript.onload = function() { loadSettings(true) };
			document.body.appendChild(cookieScript);
			return;
		} else {
			loadedSettings = localStorage.getItem("settings");
		}
	} else {
		loadedSettings = $.cookie("settings");
		$.removeCookie("settings");
	}

	// Try to parse the loaded settings (if any)
	if (loadedSettings != undefined && loadedSettings.length > 0) {
		loadedSettings = JSON.parse(loadedSettings);

		// Webcam URL
		if (loadedSettings.hasOwnProperty("webcamURL")) {
			settings.webcamURL = loadedSettings.webcamURL;
		}

		// UI Timing
		if (loadedSettings.hasOwnProperty("updateInterval")) {
			settings.updateInterval = loadedSettings.updateInterval;
		}
		if (loadedSettings.hasOwnProperty("haltedReconnectDelay")) {
			settings.haltedReconnectDelay = loadedSettings.haltedReconnectDelay;
		}
		if (loadedSettings.hasOwnProperty("updatedReconnectDelay")) {
			settings.updateReconnectDelay = loadedSettings.updateReconnectDelay;
		}
		if (loadedSettings.hasOwnProperty("extendedStatusInterval")) {
			settings.extendedStatusInterval = loadedSettings.extendedStatusInterval;
		}
		if (loadedSettings.hasOwnProperty("maxRequestTime")) {
			settings.maxRequestTime = loadedSettings.maxRequestTime;
		}
		if (loadedSettings.hasOwnProperty("webcamInterval")) {
			settings.webcamInterval = loadedSettings.webcamInterval;
		}
		if (loadedSettings.hasOwnProperty("notificationTimeout")) {
			settings.notificationTimeout = loadedSettings.notificationTimeout;
		}

		// Behavior
		if (loadedSettings.hasOwnProperty("autoConnect")) {
			settings.autoConnect = loadedSettings.autoConnect;
		}
		if (loadedSettings.hasOwnProperty("halfZMovements")) {
			settings.halfZMovements = loadedSettings.halfZMovements;
		}
		if (loadedSettings.hasOwnProperty("logSuccess")) {
			settings.logSuccess = loadedSettings.logSuccess;
		}
		if (loadedSettings.hasOwnProperty("uppercaseGCode")) {
			settings.uppercaseGCode = loadedSettings.uppercaseGCode;
		}
		if (loadedSettings.hasOwnProperty("useKiB")) {
			settings.useKiB = loadedSettings.useKiB;
		}
		if (loadedSettings.hasOwnProperty("showFanControl")) {
			settings.showFanControl = loadedSettings.showFanControl;
		}
		if (loadedSettings.hasOwnProperty("showFanRPM")) {
			settings.showFanRPM = loadedSettings.showFanRPM;
		}
		if (loadedSettings.hasOwnProperty("showATXControl")) {
			settings.showATXControl = loadedSettings.showATXControl;
		}
		if (loadedSettings.hasOwnProperty("confirmStop")) {
			settings.confirmStop = loadedSettings.confirmStop;
		}
		if (loadedSettings.hasOwnProperty("useDarkTheme")) {
			settings.useDarkTheme = loadedSettings.useDarkTheme;
		}
		if (loadedSettings.hasOwnProperty("moveFeedrate")) {
			settings.moveFeedrate = loadedSettings.moveFeedrate;
		}

		// Default list items
		if (loadedSettings.hasOwnProperty("defaultActiveTemps")) {
			settings.defaultActiveTemps = loadedSettings.defaultActiveTemps;
		}
		if (loadedSettings.hasOwnProperty("defaultStandbyTemps")) {
			settings.defaultStandbyTemps = loadedSettings.defaultStandbyTemps;
		}
		if (loadedSettings.hasOwnProperty("defaultBedTemps")) {
			settings.defaultBedTemps = loadedSettings.defaultBedTemps;
		}
		if (loadedSettings.hasOwnProperty("defaultGCodes")) {
			settings.defaultGCodes = loadedSettings.defaultGCodes;
		}

		// Other (fallback in case language.xml couldn't be loaded)
		if (loadedSettings.hasOwnProperty("language")) {
			settings.language = loadedSettings.language;
		}
	}

	// Final migration, so we don't use the cookie JS next time
	if (usingCookie) {
		localStorage.setItem("settings", JSON.stringify(settings));
	}
	settingsLoaded();
}

function saveSettings() {
	// Webcam URL
	settings.webcamURL = $("#webcam_url").val();

	// Appearance and Behavior
	settings.autoConnect = $("#auto_connect").is(":checked");
	settings.halfZMovements = $("#half_z").is(":checked");
	settings.logSuccess = $("#log_success").is(":checked");
	settings.uppercaseGCode = $("#uppercase_gcode").is(":checked");
	settings.useKiB = $("#use_kib").is(":checked");
	settings.showFanControl = $("#fan_sliders").is(":checked");
	settings.showFanRPM = $("#fan_rpm_display").is(":checked");
	settings.showATXControl = $("#show_atx").is(":checked");
	settings.confirmStop = $("#confirm_stop").is(":checked");
	settings.useDarkTheme = $("#dark_theme").is(":checked");
	settings.moveFeedrate = checkBoundaries($("#move_feedrate").val(), defaultSettings.moveFeedrate, 0);
	if (settings.language != $("#btn_language").data("language")) {
		showMessage("success", T("Language has changed"), T("You have changed the current language. Please reload the web interface to apply this change."), 0);
	}
	settings.language = $("#btn_language").data("language");

	// UI Timing
	settings.updateInterval = checkBoundaries($("#update_interval").val(), defaultSettings.updateInterval, 50);
	settings.extendedStatusInterval = checkBoundaries($("#extended_status_interval").val(), defaultSettings.extendedStatusInterval, 1, 99999);
	settings.haltedReconnectDelay = checkBoundaries($("#reconnect_halt_delay").val(), defaultSettings.haltedReconnectDelay, 1000);
	settings.updateReconnectDelay = checkBoundaries($("#reconnect_update_delay").val(), defaultSettings.updateReconnectDelay, 1000);
	settings.maxRequestTime = checkBoundaries($("#ajax_timeout").val(), defaultSettings.maxRequestTime, 100);
	settings.webcamInterval = checkBoundaries($("#webcam_interval").val(), defaultSettings.webcamInterval, 100);
	settings.notificationTimeout = checkBoundaries($("#notification_timeout").val(), defaultSettings.notificationTimeout, 0);

	// Default G-Codes
	settings.defaultGCodes = [];
	$("#table_gcodes > tbody > tr").each(function() {
		settings.defaultGCodes.push([$(this).find("label").text(), $(this).find("td:eq(1)").text()]);
	});
	settings.defaultGCodes = settings.defaultGCodes.sort(function(a, b) {
		if (a[0][0] != b[0][0]) {
			return a[0].charCodeAt(0) - b[0].charCodeAt(0);
		}
		var x = a[0].match(/(\d+)/g)[0];
		var y = b[0].match(/(\d+)/g)[0];
		if (x == undefined || y == undefined) {
			return parseInt(a[0]) - parseInt(b[0]);
		}
		return x - y;
	});

	// Default Heater Temperatures
	settings.defaultActiveTemps = [];
	$("#ul_active_temps > li").each(function() {
		settings.defaultActiveTemps.push($(this).data("temperature"));
	});
	settings.defaultActiveTemps = settings.defaultActiveTemps.sort(function(a, b) { return a - b; });
	settings.defaultStandbyTemps = [];
	$("#ul_standby_temps > li").each(function() {
		settings.defaultStandbyTemps.push($(this).data("temperature"));
	});
	settings.defaultStandbyTemps = settings.defaultStandbyTemps.sort(function(a, b) { return a - b; });

	// Default Bed Temperatures
	settings.defaultBedTemps = [];
	$("#ul_bed_temps > li").each(function() {
		settings.defaultBedTemps.push($(this).data("temperature"));
	});
	settings.defaultBedTemps = settings.defaultBedTemps.sort(function(a, b) { return a - b; });

	// Config file (on demand) - NB: We only update it while it's displayed, else users might mess up their configs...
	if (configFile != undefined && configFile != $("#text_config").val() && $('a[href="#page_config"]').parent().hasClass("active"))
	{
		configFile = $("#text_config").val();

		var uploadFile = new File([configFile], "config.g", { type: "application/octet-stream" });
		startUpload("generic", [uploadFile], false);
	}

	// Save Settings
	localStorage.setItem("settings", JSON.stringify(settings));
}

function applySettings() {
	/* Behavior */

	// Webcam
	if (settings.webcamURL != "") {
		$("#panel_webcam").removeClass("hidden");
		updateWebcam(true);
	} else {
		$("#panel_webcam").addClass("hidden");
	}

	// Half Z Movements
	var decreaseChildren = $("#td_decrease_z a");
	var decreaseVal = (settings.halfZMovements) ? 50 : 100;
	decreaseChildren.each(function(index) {
		decreaseChildren.eq(index).data("z", decreaseVal * (-1)).contents().last().replaceWith(" Z-" + decreaseVal);
		decreaseVal /= 10;
	});
	var increaseChildren = $("#td_increase_z a");
	var increaseVal = (settings.halfZMovements) ? 0.05 : 0.1;
	increaseChildren.each(function(index) {
		increaseChildren.eq(index).data("z", increaseVal).contents().first().replaceWith("Z+" + increaseVal + " ");
		increaseVal *= 10;
	});

	// Show/Hide Fan Control
	if (settings.showFanControl) {
		$(".fan-control").removeClass("hidden");
	} else {
		$(".fan-control").addClass("hidden");
	}

	// Show/Hide Fan RPM
	if (settings.showFanRPM) {
		$(".fan-rpm").removeClass("hidden");
	} else {
		$(".fan-rpm").addClass("hidden");
	}

	// Show/Hide ATX Power
	if (settings.showATXControl) {
		$(".atx-control").removeClass("hidden");
	} else {
		$(".atx-control").addClass("hidden");
	}

	// Possibly hide entire misc control panel
	if (!settings.showFanControl && !settings.showATXControl) {
		$("#panel_control_misc").addClass("hidden");
	} else {
		$("#panel_control_misc").removeClass("hidden");
	}

	// Apply or revoke theme
	if (settings.useDarkTheme) {
		if (darkThemeInclude == undefined) {
			darkThemeInclude = $('<link onload="applyThemeColors(true);" rel="stylesheet" href="css/slate.css" type="text/css"></link>');
			darkThemeInclude.appendTo('head');
			$("#theme_notice").removeClass("hidden");
		}
	} else {
		if (darkThemeInclude != undefined) {
			darkThemeInclude.remove();
			darkThemeInclude = undefined;
			applyThemeColors(false);
			$("#theme_notice").addClass("hidden");
		}
	}

	/* Set values on the Settings page */

	// Webcam link
	$("#webcam_url").val(settings.webcamURL);

	// Appearance and Behavior
	$("#auto_connect").prop("checked", settings.autoConnect);
	$("#half_z").prop("checked", settings.halfZMovements);
	$("#log_success").prop("checked", settings.logSuccess);
	$("#uppercase_gcode").prop("checked", settings.uppercaseGCode);
	$("#use_kib").prop("checked", settings.useKiB);
	$("#fan_sliders").prop("checked", settings.showFanControl);
	$("#fan_rpm_display").prop("checked", settings.showFanRPM);
	$("#show_atx").prop("checked", settings.showATXControl);
	$("#confirm_stop").prop("checked", settings.confirmStop);
	$("#dark_theme").prop("checked", settings.useDarkTheme);
	$("#move_feedrate").val(settings.moveFeedrate);

	// UI Timing
	$("#update_interval").val(settings.updateInterval);
	$("#extended_status_interval").val(settings.extendedStatusInterval);
	$("#reconnect_halt_delay").val(settings.haltedReconnectDelay);
	$("#reconnect_update_delay").val(settings.updateReconnectDelay);
	$("#ajax_timeout").val(settings.maxRequestTime);
	$.ajaxSetup({ timeout: settings.maxRequestTime });
	$("#webcam_interval").val(settings.webcamInterval);
	$("#notification_timeout").val(settings.notificationTimeout);

	/* Default list items */

	// Default head temperatures
	clearHeadTemperatures();
	settings.defaultActiveTemps.forEach(function(temp) {
		addHeadTemperature(temp, "active");
	});
	settings.defaultStandbyTemps.forEach(function(temp) {
		addHeadTemperature(temp, "standby");
	});

	// Default bed temperatures
	clearBedTemperatures();
	settings.defaultBedTemps.forEach(function(temp) {
		addBedTemperature(temp);
	});

	// Default G-Codes
	clearDefaultGCodes();
	settings.defaultGCodes.forEach(function(entry) {
		addDefaultGCode(entry[1], entry[0]);
	});
}

/* File Uploads */

var uploadType, uploadFiles, uploadRows, uploadedFileCount;
var uploadTotalBytes, uploadedTotalBytes;
var uploadStartTime, uploadRequest, uploadFileSize, uploadFileName, uploadPosition;
var uploadedDWC, uploadIncludedConfig, uploadFirmwareFile;

function startUpload(type, files, fromCallback) {
	// Initialize some values
	stopUpdates();
	isUploading = true;	
	uploadType = type;
	uploadTotalBytes = uploadedTotalBytes = uploadedFileCount = 0;
	uploadFiles = files;
	$.each(files, function() {
		uploadTotalBytes += this.size;
	});
	uploadRows = [];
	if (!fromCallback) {
		uploadedDWC = false;
	}
	uploadIncludedConfig = false;
	uploadFirmwareFile = undefined;

	// Safety check for Upload and Print
	if (type == "print" && files.length > 1) {
		showMessage("warning", T("Error"), T("You can only upload and print one file at once!"));
		return;
	}

	// Unzip files if necessary
	if (type == "macro" || type == "generic") {
		var containsZip = false;
		$.each(files, function() {
			if (this.name.toLowerCase().match("\\.zip$") != null) {
				uploadedDWC |= this.name.toLowerCase().match("^duetwebcontrol.*\\.zip") != null;

				var fileReader = new FileReader();
				fileReader.onload = (function(theFile) {
					return function(e) {
						try {
							var zip = new JSZip(e.target.result);

							var zipFiles = [];
							$.each(zip.files, function(index, zipEntry) {
								if (!zipEntry.dir && zipEntry.name.match("\/\\.git") == null && zipEntry.name.match("README") == null) {
									var zipName = zipEntry.name.split("/");
									zipName = zipName[zipName.length - 1];

									var unpackedFile = new File([zipEntry.asArrayBuffer()], zipName, { type: "application/octet-stream", lastModified: zipEntry.date });
									zipFiles.push(unpackedFile);
								}
							});

							if (zipFiles.length == 0) {
								showMessage("warning", T("Error"), T("The archive {0} does not contain any files!", theFile.name));
							} else {
								startUpload(type, zipFiles, true);
							}
						} catch(e) {
							showMessage("danger", T("Error"), T("Could not read contents of file {0}!", theFile.name));
						}
					}
				})(this);
				fileReader.readAsArrayBuffer(this);

				containsZip = true;
				return false;
			}
		});
		if (containsZip) {
			// We're relying on an async task which will trigger this method again when required
			return;
		}
	}

	// Reset modal dialog
	$("#modal_upload").data("backdrop", "static")
		$("#modal_upload .close, #modal_upload button[data-dismiss='modal']").addClass("hidden");
	$("#btn_cancel_upload, #modal_upload p").removeClass("hidden");
	$("#modal_upload h4").text(T("Uploading File(s), {0}% Complete", 0));

	// Add files to the table
	$("#table_upload_files > tbody").remove();
	$.each(files, function() {
		if (type == "generic") {
			uploadIncludedConfig |= (this.name == "config.g");
			if (this.name.toUpperCase().match("^REPRAPFIRMWARE.*\.BIN") != null) {
				uploadFirmwareFile = this.name;
			}
		}

		var row = 	'<tr><td><span class="glyphicon glyphicon-asterisk"></span> ' + this.name + '</td>';
		row += 		'<td>' + formatSize(this.size) + '</td>';
		row +=		'<td><div class="progress"><div class="progress-bar progress-bar-info progress-bar-striped" role="progressbar"><span></span></div></div></td></tr>';
		$("#table_upload_files").append(row);
		uploadRows.push($("#table_upload_files > tbody > tr:last-child"));
	});
	$("#modal_upload").modal("show");

	// Start file upload
	uploadNextFile();
}

function uploadNextFile() {
	// Prepare some upload values
	var file = uploadFiles[uploadedFileCount];
	uploadFileName = file.name;
	uploadFileSize = file.size;
	uploadStartTime = new Date();
	uploadPosition = 0;

	// Determine the right path
	var targetPath = "";
	switch (uploadType) {
		case "gcode":	// Upload G-Code
		case "print":	// Upload & Print G-Code
			targetPath = currentGCodeDirectory + "/" + uploadFileName;
			break;

		case "macro":	// Upload Macro
			targetPath = currentMacroDirectory + "/" + uploadFileName;
			break;

		default:		// Generic Upload (on the Settings page)
			var fileExt = uploadFileName.split('.').pop().toLowerCase();
			switch (fileExt) {
				case "ico":
				case "html":
				case "htm":
				case "xml":
					targetPath = "/www/" + uploadFileName;
					break;

				case "css":
				case "map":
					targetPath = "/www/css/" + uploadFileName;
					break;

				case "eot":
				case "svg":
				case "ttf":
				case "woff":
				case "woff2":
					targetPath = "/www/fonts/" + uploadFileName;
					break;

				case "jpeg":
				case "jpg":
				case "png":
					targetPath = "/www/img/" + uploadFileName;
					break;

				case "js":
					targetPath = "/www/js/" + uploadFileName;
					break;

				default:
					targetPath = "/sys/" + uploadFileName;
			}
	}

	// Update the GUI
	uploadRows[0].find(".progress-bar > span").text(T("Starting"));
	uploadRows[0].find(".glyphicon").removeClass("glyphicon-asterisk").addClass("glyphicon-cloud-upload");

	// Begin another POST file upload
	uploadRequest = $.ajax("rr_upload?name=" + encodeURIComponent(targetPath), {
		data: file,
		dataType: "json",
		processData: false,
		contentType: false,
		timeout: 0,
		type: "POST",
		success: function(data) {
			if (isUploading) {
				finishCurrentUpload(data.err == 0);
			}
		},
		xhr: function() {
			var xhr = new window.XMLHttpRequest();
			xhr.upload.addEventListener("progress", function(event) {
				if (isUploading && event.lengthComputable) {
					// Calculate current upload speed (Date is based on milliseconds)
					uploadSpeed = event.loaded / (((new Date()) - uploadStartTime) / 1000);

					// Update global progress
					uploadedTotalBytes += (event.loaded - uploadPosition);
					uploadPosition = event.loaded;

					var uploadTitle = T("Uploading File(s), {0}% Complete", ((uploadedTotalBytes / uploadTotalBytes) * 100).toFixed(0));
					if (uploadSpeed > 0) {
						uploadTitle += " (" + formatSize(uploadSpeed) + "/s)";
					}
					$("#modal_upload h4").text(uploadTitle);

					// Update progress bar
					var progress = ((event.loaded / event.total) * 100).toFixed(0);
					uploadRows[0].find(".progress-bar").css("width", progress + "%");
					uploadRows[0].find(".progress-bar > span").text(progress + " %");
				}
			}, false);
			return xhr;
		}
	});
}

function finishCurrentUpload(success) {
	// Keep the progress updated
	if (!success) {
		uploadedTotalBytes += (uploadFileSize - uploadPosition);
	}

	// Update glyphicon and progress bar
	uploadRows[0].find(".glyphicon").removeClass("glyphicon-cloud-upload").addClass(success ? "glyphicon-ok" : "glyphicon-alert");
	uploadRows[0].find(".progress-bar").removeClass("progress-bar-info").addClass(success ? "progress-bar-success" : "progress-bar-danger").css("width", "100%");
	uploadRows[0].find(".progress-bar > span").text(success ? "100 %" : T("ERROR"));

	// Go on with upload logic if we're still busy
	if (isUploading) {
		uploadedFileCount++;
		if (uploadFiles.length > uploadedFileCount) {
			// Purge last-uploaded file row
			uploadRows.shift();

			// Upload the next one
			uploadNextFile();
		} else {
			// We're done
			finishUpload(true);
		}
	}
}

function cancelUpload() {
	isUploading = false;
	finishCurrentUpload(false);
	finishUpload(false);
	$("#modal_upload h4").text(T("Upload Cancelled!"));
	uploadRequest.abort();
	startUpdates();
}

function finishUpload(success) {
	// Reset upload variables
	isUploading = false;
	uploadFiles = uploadRows = [];
	$("#input_file_upload").val("");

	// Set some values in the modal dialog
	$("#modal_upload h4").text(T("Upload Complete!"));
	$("#btn_cancel_upload, #modal_upload p").addClass("hidden");
	$("#modal_upload .close, #modal_upload button[data-dismiss='modal']").removeClass("hidden");

	if (success) {
		// If everything went well, update the GUI immediately
		uploadHasFinished(true);
	} else {
		// In case an upload has been aborted, give the firmware some time to recover
		setTimeout(function() { uploadHasFinished(false); }, 1000);
	}
}

function uploadHasFinished(success) {
	// Make sure the G-Codes and Macro pages are updated
	if (uploadType == "gcode" || uploadType == "print") {
		gcodeUpdateIndex = -1;
		if (currentPage == "files") {
			updateGCodeFiles();
		}
	} else if (uploadType == "macro") {
		macroUpdateIndex = -1;
		if (currentPage == "control" || currentPage == "macros") {
			updateMacroFiles();
		}
	}

	// Start polling again
	startUpdates();

	// Deal with different upload types
	if (success) {
		// Check if a print is supposed to be started
		if (uploadType == "print") {
			waitingForPrintStart = true;
			if (currentGCodeDirectory == "/gcodes") {
				sendGCode("M32 " + uploadFileName);
			} else {
				sendGCode("M32 " + currentGCodeDirectory.substring(8) + "/" + uploadFileName);
			}
		}

		// Ask for page reload if DWC has been updated
		if (uploadedDWC) {
			$("#modal_upload").modal("hide");
			showConfirmationDialog(T("Reload Page?"), T("You have just updated Duet Web Control. Would you like to reload the page now?"), function() {
				location.reload();
			});
		}

		// Ask for software reset if it's safe to do
		else if (lastStatusResponse != undefined && lastStatusResponse.status == 'I') {
			if (uploadIncludedConfig) {
				$("#modal_upload").modal("hide");
				showConfirmationDialog(T("Reboot Duet?"), T("You have just uploaded a config file. Would you like to perform a software reset now?"), function() {
					sendGCode("M999");
				});
			}

			if (uploadFirmwareFile != undefined)
			{
				$("#modal_upload").modal("hide");
				showConfirmationDialog(T("Perform Firmware Update?"), T("You have just uploaded a firmware file. Would you like to update your Duet now?"), startFirmwareUpdate);
			}
		}
	}
}

function startFirmwareUpdate() {
	if (uploadFirmwareFile.toUpperCase() != "REPRAPFIRMWARE.BIN")
	{
		// The firmware filename is hardcoded in the IAP binary, so try to rename the uploaded file first
		$.ajax("rr_move?old=" + encodeURIComponent("/sys/" + uploadFirmwareFile) + "&new=" + encodeURIComponent("/sys/RepRapFirmware.bin"), {
			dataType: "json",
			success: function(response) {
				if (response.err == 0) {
					// Rename succeeded and flashing can be performed now
					sendGCode("M997");
				} else {
					// Looks like /sys/RepRapFirmware.bin already exists, attempt to delete it and try again
					$.ajax("rr_delete?name=" + encodeURIComponent("/sys/RepRapFirmware.bin"), {
						dataType: "json",
						success: function(response) {
							if (response.err == 0) {
								// File delete succeeded, attempt to start the firmware update once again
								startFirmwareUpdate();
							} else {
								// Something went wrong
								showMessage("danger", T("Error"), T("Could not rename firmware file!"));
							}
						}
					});
				}
			}
		});
	}
	else
	{
		// Filename is okay, start flashing immediately
		sendGCode("M997");
	}

}

/* Data helpers */

function requestFileInfo() {
	$.ajax("rr_fileinfo", {
		dataType: "json",
		success: function(response) {
			if (isConnected && response.err == 2) {
				// The firmware is still busy parsing the file, so try again until it's ready
				setTimeout(function() {
					if (isConnected) {
						requestFileInfo();
					}
				}, 250);
			} else if (response.err == 0) {
				// File info is valid, use it
				fileInfo = response;

				$("#span_progress_left").html(T("Printing {0}", response.fileName));

				$("#dd_size").html(formatSize(response.size));
				$("#dd_height").html((response.height > 0) ? (response.height + " mm") : "n/a");
				var layerHeight = (response.layerHeight > 0) ? (response.layerHeight + " mm") : "n/a";
				if (response.firstLayerHeight > 0) {
					$("#dd_layer_height").html(response.firstLayerHeight + " mm / " + layerHeight);
				} else {
					$("#dd_layer_height").html(layerHeight);
				}

				if (response.filament.length == 0) {
					$("#dd_filament").html("n/a");
				} else {
					var filament = response.filament.reduce(function(a, b) { return a + " mm, " + b; }) + " mm";
					$("#dd_filament").html(filament);
				}

				$("#dd_generatedby").html((response.generatedBy == "") ? "n/a" : response.generatedBy);

				$("#td_print_duration").html(convertSeconds(response.printDuration));
			}
		}
	});
}

function setToolMapping(mapping) {
	if (toolMapping != mapping) {
		toolMapping = mapping;

		// Clean up current tools
		$("#page_tools").children(":not(:first-child)").remove();

		// Create new panels for each tool
		if (toolMapping != undefined) {
			for(var i=0; i<toolMapping.length; i++) {
				var number = toolMapping[i].hasOwnProperty("number") ? toolMapping[i].number : (i + 1);

				var heaters;
				if (toolMapping[i].heaters.length == 0) {
					heaters = T("none");
				} else {
					heaters = toolMapping[i].heaters.reduce(function(a, b) { return a + ", " + b; });
				}

				var drives;
				if (toolMapping[i].drives.length == 0) {
					drives = T("none");
				} else {
					drives = toolMapping[i].drives.reduce(function(a, b) { return a + ", " + b; });
				}

				var div =	'<div class="col-xs-6 col-sm-6 col-md-3 col-lg-3"><div class="panel panel-default">';
				div +=		'<div class="panel-heading"><span>' + T("Tool {0}", number) + '</span></div>';
				div +=		'<div data-tool="' + number + '" class="panel-body">';
				div +=		'<dl><dt>' + T("Heaters:") + '</dt><dd>' + heaters + '</dd>';
				div +=		'<dt>' + T("Drives:") + '</dt><dd>' + drives + '</dd>';
				div +=		'</dl><div class="row"><div class="col-md-12 text-center">';
				if (lastStatusResponse != undefined && lastStatusResponse.currentTool == number) {
					div +=		'<button class="btn btn-success btn-select-tool" title="' + T("Deselect this tool") + '">';
					div +=		'<span class="glyphicon glyphicon-remove"></span> <span>' + T("Deselect") + '</span></button>';
				} else {
					div +=		'<button class="btn btn-success btn-select-tool" title="' + T("Select this tool") + '">';
					div +=		'<span class="glyphicon glyphicon-pencil"></span> <span>' + T("Select") + '</span></button>';
				}
				div +=		' <button class="btn btn-danger btn-remove-tool" title="' + T("Remove this tool") + '">';
				div +=		'<span class="glyphicon glyphicon-trash"></span> ' + T("Remove") + '</button>';
				div +=		'</div></div></div></div></div>';
				$("#page_tools").append(div);
			}
		}

		// Keep the GUI updated
		validateAddTool();
	}
}

function getTool(number) {
	if (toolMapping == undefined) {
		return undefined;
	}

	for(var i=0; i<toolMapping.length; i++) {
		if (toolMapping[i].hasOwnProperty("number")) {
			if (toolMapping[i].number == number) {
				return toolMapping[i];
			}
		} else if (i + 1 == number) {
			return toolMapping[i];
		}
	}
	return undefined;
}

function getToolsByHeater(heater) {
	if (toolMapping == undefined) {
		return [];
	}

	var result = [];
	for(var i=0; i<toolMapping.length; i++) {
		for(var k=0; k<toolMapping[i].heaters.length; k++) {
			if (toolMapping[i].heaters[k] == heater) {
				if (toolMapping[i].hasOwnProperty("number")) {
					result.push(toolMapping[i].number);
				} else {
					result.push(i + 1);
				}
			}
		}
	}
	return result;
}

function T(text) {
	var entry = text;
	if (translationData != undefined) {
		// Generate a regex to check with
		text = text.replace(/{(\d+)}/g, "{\\d+}").replace("(", "\\(").replace(")", "\\)");
		text = text.replace("?", "[?]").replace(".", "[.]");
		var regex = new RegExp("^" + text + "$");

		// Get the translation node and see if we can find an entry
		var root = translationData.getElementsByTagName(settings.language).item(settings.language);
		if (root != null) {
			for(var i=0; i<root.children.length; i++) {
				if (regex.test(root.children[i].attributes["t"].value)) {
					entry = root.children[i].textContent;
					break;
				}
			}

			// Log translation text if we couldn't find a suitable text
			if (translationWarning && entry == text) {
				console.log("WARNING: Could not translate '" + entry + "'");
			}
		}
	}

	// Format it with the given arguments
	var args = arguments;
	return entry.replace(/{(\d+)}/g, function(match, number) {
		number = parseInt(number) + 1;
		return typeof args[number] != 'undefined' ? args[number] : match;
	});
}

// May be called only once on page load to translate the page
function translatePage() {
	if (translationData != undefined) {
		var root = translationData.getElementsByTagName(settings.language).item(settings.language);
		if (root != null) {
			translateEntries(root, $("span, th, td, strong, dt, button"), "textContent");
			translateEntries(root, $("h1, h4, label, a"), "textContent");
			translateEntries(root, $("input[type='text']"), "placeholder");
			translateEntries(root, $("a, abbr, button, label, #chart_temp, input, td"), "title");
			translateEntries(root, $("img"), "alt");

			$("#btn_language").data("language", settings.language).children("span:first-child").text(root.attributes["name"].value);
			$("html").attr("lang", settings.language);
		}
	}
}

function translateEntries(root, entries, key) {
	var doNodeCheck = (key == "textContent");
	$.each(entries, function() {
		// If this node has no children, we can safely use it
		if (!doNodeCheck || this.childNodes.length < 2) {
			translateEntry(root, this, key);
			// Otherwise we need to check for non-empty text nodes
		} else {
			for(var i=0; i<this.childNodes.length; i++) {
				var val = this.childNodes[i][key];
				if (this.childNodes[i].nodeType == 3 && val != undefined && this.childNodes[i].childNodes.length == 0 && val.trim().length > 0) {
					translateEntry(root, this.childNodes[i], key);
				}
			}
		}
	});
}

function translateEntry(root, item, key) {
	if (item != undefined) {
		var originalText = item[key];
		if (originalText != undefined && originalText.trim() != "") {
			var text = originalText.trim();
			for(var i=0; i<root.children.length; i++) {
				var entry = root.children[i].attributes["t"].value;
				if (entry.indexOf("{") == -1 && entry == text) {
					item[key] = item[key].replace(text, root.children[i].textContent);
					return;
				}
			}

			// Log translation text if we couldn't find a suitable text
			if (translationWarning) {
				console.log("WARNING: Could not translate static '" + text + "'");
			}
		}
	}
}

// vim: ts=4:sw=4
