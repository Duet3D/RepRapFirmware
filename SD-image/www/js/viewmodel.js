/* Interface logic for the Duet Web Control v1.11
 * 
 * written by Christian Hammacher
 * 
 * licensed under the terms of the GPL v2
 * see http://www.gnu.org/licenses/gpl-2.0.html
 */

var maxTemperatureSamples = 1000;
var probeSlowDownColor = "#FFFFE0", probeTriggerColor = "#FFF0F0";

var tempChart;
var tempChartOptions = 	{
	colors: ["#0000FF", "#FF0000", "#00DD00", "#FFA000", "#FF00FF", "#337AB7", "#00FFFF", "#000000"],
	grid: {
		borderWidth: 0
	},
	xaxis: {
		show: false
	},
	yaxis: {
		min: 0,
		max: 280
	}
};
var tempChartPadding = 15;

var printChart;
var printChartOptions =	{
	colors: ["#EDC240"],
	grid: {
		borderWidth: 0,
		hoverable: true,
		clickable: true
	},
	pan: {
		interactive: true
	},
	series: {
		lines: {
			show: true
		},
		points: {
			show: true
		}
	},
	xaxis: {
		min: 1,
		tickDecimals: 0,
	},
	yaxis: {
		min: 0,
		max: 30,
		ticks: 5,
		tickDecimals: 0,
		tickFormatter: function(val) { if (!val) { return ""; } else { return val + "s"; } }
	},
	zoom: {
		interactive: true
	}
};

var webcamUpdating = false;

var notificationOptions = {
	animate: {
		enter: 'animated fadeInDown',
		exit: 'animated fadeOutDown'
	},
	placement: {
		from: "bottom",
		align: "center"
	},
	template: '<div data-notify="container" class="col-xs-11 col-sm-9 col-md-8 col-lg-5 alert alert-{0}" role="alert">' +
		'<button type="button" aria-hidden="true" class="close" data-notify="dismiss">×</button>' +
		'<span data-notify="icon"></span> ' +
		'<span data-notify="title">{1}</span> ' +
		'<span data-notify="message">{2}</span>' +
		'<div class="progress" data-notify="progressbar">' +
		'<div class="progress-bar progress-bar-{0}" role="progressbar" aria-valuenow="0" aria-valuemin="0" aria-valuemax="100" style="width: 0%;"></div>' +
		'</div>' +
		'<a href="{3}" target="{4}" data-notify="url"></a>' +
		'</div>'
};


$(document).ready(function() {
	disableControls();

	resetGuiData();
	updateGui();

	$("#web_version").append(", JS: " + jsVersion);
	$("#text_config").textareaAutoSize();

	$.notifyDefaults(notificationOptions);

	loadSettings(false);
});

function settingsLoaded() {
	applySettings();

	$.ajax("language.xml", {
		type: "GET",
		dataType: "xml",
		global: false,
		error: function() {
			pageLoadComplete();
		},
		success: function(response) {
			translationData = response;

			if (translationData.children == undefined)
			{
				// Internet Explorer and Edge cannot deal with XML files in the way we want.
				// Disable translations for those browsers.
				translationData = undefined;
				$("#dropdown_language, #label_language").addClass("hidden");
			} else {
				$("#dropdown_language ul > li:not(:first-child)").remove();
				for(var i=0; i<translationData.children[0].children.length; i++) {
					var id = translationData.children[0].children[i].tagName;
					var name = translationData.children[0].children[i].attributes["name"].value;
					$("#dropdown_language ul").append(	'<li><a data-language="' + id + '" href="#">' + name + '</a></li>');
					if (settings.language == id) {
						$("#btn_language > span:first-child").text(name);
					}
				}

				translatePage();
			}

			pageLoadComplete();
		}
	});

}

function pageLoadComplete() {
	log("info", "<strong>" + T("Page Load complete!") + "</strong>");

	if (settings.autoConnect) {
		// Users might want to connect automatically once the page has loaded
		connect(sessionPassword, true);
	}
}

function enableControls() {
	$("nav input, #div_heaters input, #main_content input").prop("disabled", false);			// Generic inputs
	$("#page_tools label").removeClass("disabled");												// and on Settings page

	$(".btn-emergency-stop, .gcode-input button[type=submit], .gcode").removeClass("disabled");	// Navbar
	$(".bed-temp, .gcode, .heater-temp, .btn-upload").removeClass("disabled");					// List items and Upload buttons

	$("#mobile_home_buttons button, #btn_homeall, #table_move_head a").removeClass("disabled");	// Move buttons
	$("#panel_extrude label.btn, #panel_extrude button").removeClass("disabled");				// Extruder Control
	$("#panel_control_misc label.btn").removeClass("disabled");									// ATX Power
	$("#slider_fan_control").slider("enable");													// Fan Control

	$("#page_print .checkbox").removeClass("disabled");											// Print Control
	$("#slider_fan_print").slider("enable");													// Fan Control
	$("#slider_speed").slider("enable");														// Speed Factor
	for(var extr=1; extr<=6; extr++) {
		$("#slider_extr_" + extr).slider("enable");												// Extrusion Factors
	}

	$(".online-control").removeClass("hidden");													// G-Code/Macro Files
}

function disableControls() {
	$("nav input, #div_heaters input, #main_content input").prop("disabled", true);				// Generic inputs
	$("#page_general input, #page_listitems input").prop("disabled", false);					// ... except ...
	$("#page_tools label").addClass("disabled");												// ... for Settings

	$(".btn-emergency-stop, .gcode-input button[type=submit], .gcode").addClass("disabled");	// Navbar
	$("#extruder_drives").addClass("disabled");													// Info Panels
	$(".bed-temp, .gcode, .heater-temp, .btn-upload").addClass("disabled");						// List items and Upload buttons

	$("#mobile_home_buttons button, #btn_homeall, #table_move_head a").addClass("disabled");	// Move buttons
	$("#panel_extrude label.btn, #panel_extrude button").addClass("disabled");					// Extruder Control
	$("#panel_control_misc label.btn").addClass("disabled");									// ATX Power
	$("#slider_fan_control").slider("disable");													// Fan Control

	$("#btn_pause, #page_print .checkbox").addClass("disabled");								// Print Control
	$("#slider_fan_print").slider("disable");													// Fan Control
	$("#slider_speed").slider("disable");														// Speed Factor
	for(var extr=1; extr<=6; extr++) {
		$("#slider_extr_" + extr).slider("disable");											// Extrusion Factors
	}

	$(".online-control").addClass("hidden");														// G-Code/Macro Files
}

function updateGui() {
	// Visibility for Heater Temperatures

	if (heatedBed) {			// Heated Bed
		$("#tr_bed").removeClass("hidden");
	} else {
		$("#tr_bed").addClass("hidden");
	}
	if (chamber) {				// Chamber
		$("#tr_chamber").removeClass("hidden");
	} else {
		$("#tr_chamber").addClass("hidden");
	}
	for(var i=1; i<=6; i++) {	// Heads (Heaters)
		if (i <= numHeads) {
			$("#tr_head_" + i).removeClass("hidden");
		} else {
			$("#tr_head_" + i).addClass("hidden");
		}
	}

	// Visibility for Extruder Drive columns

	for(var i=1; i<=6; i++) {
		if (i <= numExtruderDrives) {
			$(".extr-" + i).removeClass("hidden").css("border-right", "");
			$("#slider_extr_" + i).slider("relayout");
		} else {
			$(".extr-" + i).addClass("hidden").css("border-right", "");
		}
	}
	if (numExtruderDrives > 0) {
		$(".extr-" + numExtruderDrives).css("border-right", "0px");
		$("#row_status_2").removeClass("hidden");
	} else {
		$("#row_status_2").addClass("hidden");
	}

	// Add Zero Extruder Drive buttons

	if (isConnected) {
		$("#extruder_drives").removeClass("disabled");
		$("#ul_extruder_dropdown").html('<li><a href="#" class="zero-extruder" data-target="all">' + T("Zero All Drives") + '</li><li class="divider"></li>');
		for(var i=1; i<= numExtruderDrives; i++) {
			$("#ul_extruder_dropdown").append('<li><a href="#" class="zero-extruder" data-target="' + i + '">' + T("Zero Extruder Drive {0}", i) + '</a></li>');
		}

		$(".zero-extruder").off("click").click(function(e) {
			if (lastStatusResponse != undefined) {
				var gcode = "G92 E", target = $(this).data("target");
				for(var i=1; i<=numExtruderDrives; i++) {
					if (target != "all" && target != i) {
						gcode += lastStatusResponse.coords.extr[i - 1];
					} else {
						gcode += "0";
					}

					if (i < numExtruderDrives) {
						gcode += ":";
					}
				}
				sendGCode(gcode);
			}
			e.preventDefault();
		});
	}

	// Do some rearrangement for the panels if we have less than or exactly three extruder drives

	if (numExtruderDrives <= 3) {
		$(".div-head-temp").removeClass("hidden-sm");
		$("#col_extr_totals, #td_extr_total").addClass("hidden");
		for(var i=1; i<=3; i++) {
			$("th.extr-" + i).html(T("Drive " + i));
			$("#row_status_2 .extr-" + i).removeClass("hidden-md");
		}
		$("#div_heaters").removeClass("col-sm-5").addClass("col-sm-6");
		$("#div_temp_chart").removeClass("col-lg-3").addClass("col-lg-5");
		$("#div_status").removeClass("col-sm-7 col-lg-5").addClass("col-sm-6 col-lg-3");
	} else {
		$(".div-head-temp").addClass("hidden-sm");
		$("#col_extr_totals, #td_extr_total").removeClass("hidden");
		for(var i=1; i<=3; i++) {
			$("th.extr-" + i).html(T("D" + i));
			$("#row_status_2 .extr-" + i).addClass("hidden-md");
		}
		$("#div_heaters").removeClass("col-sm-6").addClass("col-sm-5");
		$("#div_temp_chart").removeClass("col-lg-5").addClass("col-lg-3");
		$("#div_status").removeClass("col-sm-6 col-lg-3").addClass("col-sm-7 col-lg-5");
	}

	// Charts

	resizeCharts();
	drawTemperatureChart();
	drawPrintChart();
}

function resetGui() {
	// Charts
	drawTemperatureChart();
	drawPrintChart();

	// Navbar
	setTitle("Duet Web Control");
	setStatusLabel("Disconnected", "default");

	// Heater Temperatures
	$("#table_heaters tr > th:first-child > span").text("");
	setCurrentTemperature("bed",  undefined);
	setTemperatureInput("bed", 0, 1);
	setCurrentTemperature("chamber",  undefined);
	setTemperatureInput("chamber", 0, 1);
	for(var i=1; i<=6; i++) {
		setCurrentTemperature(i, undefined);
		setTemperatureInput(i, 0, 1);
		setTemperatureInput(i, 0, 0);
	}

	// Status fields
	$("#td_x, #td_y, #td_z").text("n/a");
	for(var i=1; i<=numExtruderDrives; i++) {
		$("#td_extr_" + i).text("n/a");
	}
	$("#td_extr_total").text("n/a");
	setProbeValue(-1, undefined);
	$("#td_fanrpm, #td_cputemp").text("n/a");
	$(".cpu-temp").addClass("hidden");

	// Control page
	setAxesHomed([1,1,1]);
	setATXPower(false);
	$('#slider_fan_control').slider("setValue", 35);

	// Print Status
	$(".row-progress").addClass("hidden");
	setProgress(100, "", "");
	$("#override_fan, #auto_sleep").prop("checked", false);
	$('#slider_fan_print').slider("setValue", 35);
	$("#page_print dd, #panel_print_info table td, #table_estimations td").html("n/a");
	$('#slider_speed').slider("setValue", 100);
	for(var extr=1; extr<=6; extr++) {
		$("#slider_extr_" + extr).slider("setValue", 100);
	}

	// G-Code Console is not cleared automatically

	// G-Code Files
	updateGCodeFiles();

	// Macro Files
	updateMacroFiles();

	// Settings
	$("#firmware_name, #firmware_version").html("n/a");
	$("#page_machine td:not(:first-child), #page_machine dd").html("n/a");
	$("#div_config > h1").removeClass("hidden").html(T("Connect to your Duet to display the configuration file"));
	$("#text_config").addClass("hidden");

	// Modal dialogs
	$("#modal_upload").modal("hide");
}

function updateWebcam(externalTrigger) {
	if (externalTrigger && webcamUpdating) {
		// When the settings are applied, make sure this only runs once
		return;
	}

	if (settings.webcamURL == "") {
		webcamUpdating = false;
	} else {
		var newURL = settings.webcamURL;
		if (newURL.indexOf("?") == -1) {
			newURL += "?dummy=" + Math.random();
		} else {
			newURL += "&dummy=" + Math.random();
		}
		$("#img_webcam").attr("src", newURL);

		webcamUpdating = true;
		setTimeout(function() {
			updateWebcam(false);
		}, settings.webcamInterval);
	}
}

/* Dynamic GUI Events */

$("body").on("click", ".bed-temp", function(e) {
	if ($(this).parents("#tr_bed").length > 0) {
		sendGCode("M140 S" + $(this).data("temp"));
	} else {
		sendGCode("M141 S" + $(this).data("temp"));
	}
	e.preventDefault();
});

$("body").on("click", ".btn-macro", function(e) {
	var directory = $(this).data("directory");
	if (directory != undefined) {
		var dropdown = $(this).parent().children("ul");
		dropdown.css("width", $(this).outerWidth());
		loadMacroDropdown(directory, dropdown);
		dropdown.dropdown();
	} else {
		sendGCode("M98 P" + $(this).data("macro"));
	}
	e.preventDefault();
});

$("body").on("click", ".btn-print-file, .gcode-file", function(e) {
	var file = $(this).parents("tr").data("file");
	showConfirmationDialog(T("Start Print"), T("Do you want to print <strong>{0}</strong>?", file), function() {
		waitingForPrintStart = true;
		if (currentGCodeDirectory == "/gcodes") {
			sendGCode("M32 " + file);
		} else {
			sendGCode("M32 " + currentGCodeDirectory.substring(8) + "/" + file);
		}
	});
	e.preventDefault();
});

$("body").on("click", ".btn-delete-file", function(e) {
	var row = $(this).parents("tr");
	var file = row.data("file");
	showConfirmationDialog(T("Delete File"), T("Are you sure you want to delete <strong>{0}</strong>?", file), function() {
		$.ajax("rr_delete?name=" + encodeURIComponent(currentGCodeDirectory + "/" + file), {
			dataType: "json",
			row: row,
			file: file,
			success: function(response) {
				if (response.err == 0) {
					this.row.remove();
					if ($("#table_gcode_files tbody").children().length == 0) {
						gcodeUpdateIndex = -1;
						updateGCodeFiles();
					}
				} else {
					showMessage("warning", T("Deletion failed"), T("<strong>Warning:</strong> Could not delete file <strong>{0}</strong>!", this.file));
				}
			}
		});
	});
	e.preventDefault();
});

$("body").on("click", ".btn-delete-gcode-directory", function(e) {
	var row = $(this).parents("tr");
	var directory = row.data("directory");
	$.ajax("rr_delete?name=" + encodeURIComponent(currentGCodeDirectory + "/" + directory), {
		dataType: "json",
		row: row,
		directory: directory,
		success: function(response) {
			if (response.err == 0) {
				this.row.remove();
				if ($("#table_gcode_files tbody").children().length == 0) {
					gcodeUpdateIndex = -1;
					updateGCodeFiles();
				}
			} else {
				showMessage("warning", T("Deletion failed"), T("<strong>Warning:</strong> Could not delete directory <strong>{0}</strong>!<br/><br/>Perhaps it isn't empty?", this.directory));
			}
		}
	});
	e.preventDefault();
});

$("body").on("click", ".btn-delete-parent", function(e) {
	$(this).parents("tr, li").remove();
	e.preventDefault();
});

$("body").on("click", ".btn-delete-macro-directory", function(e) {
	var row = $(this).parents("tr");
	var directory = row.data("directory");
	$.ajax("rr_delete?name=" + encodeURIComponent(currentMacroDirectory + "/" + directory), {
		dataType: "json",
		row: row,
		directory: directory,
		success: function(response) {
			if (response.err == 0) {
				if (currentMacroDirectory == "/macros") {
					var button = $("#panel_macro_buttons button[data-macro='" + currentMacroDirectory + "/" + this.directory + "']");
					button.remove();
				}
				this.row.remove();
				if ($("#table_macro_files tbody").children().length == 0) {
					macroUpdateIndex = -1;
					updateMacroFiles();
				}
			} else {
				showMessage("warning", T("Deletion failed"), T("<strong>Warning:</strong> Could not delete directory <strong>{0}</strong>!<br/><br/>Perhaps it isn't empty?", this.directory));
			}
		}
	});
	e.preventDefault();
});

$("body").on("click", ".btn-delete-macro", function(e) {
	var macroFile = $(this).parents("tr").data("file");
	showConfirmationDialog("Delete File", "Are you sure you want to delete <strong>" + macroFile + "</strong>?", function() {
		$.ajax("rr_delete?name=" + encodeURIComponent(currentMacroDirectory + "/" + macroFile), {
			dataType: "json",
			macro: macroFile,
			success: function(response) {
				if (response.err == 0) {
					$("#table_macro_files tr[data-item='" + macroFile + "'], #panel_macro_buttons button[data-macro='" + currentMacroDirectory + "/" + macroFile + "']").remove();
					if ($("#table_macro_files tbody").children().length == 0) {
						macroUpdateIndex = -1;
						updateMacroFiles();
					}
				} else {
					showMessage("warning", T("Deletion failed"), T("<strong>Warning:</strong> Could not delete macro <strong>{0}</strong>!", this.macro));
				}
			}
		});
	});
	e.preventDefault();
});

$("body").on("click", ".btn-remove-tool", function(e) {
	var tool = $(this).parents("div.panel-body").data("tool");
	showConfirmationDialog(T("Delete Tool"), T("Are you sure you wish to remove tool {0}?", tool), function() {
		sendGCode("M563 P" + tool + " D-1 H-1");
		extendedStatusCounter = settings.extendedStatusInterval;
	});
	e.preventDefault();
});

$("body").on("click", ".btn-run-macro", function(e) {
	sendGCode("M98 P" + currentMacroDirectory + "/" + $(this).parents("tr").data("file"));
	e.preventDefault();
});

$("body").on("click", ".btn-select-tool", function(e) {
	var tool = $(this).parents("div.panel-body").data("tool");
	if (lastStatusResponse != undefined && lastStatusResponse.currentTool == tool) {
		changeTool(-1);
	} else {
		changeTool(tool);
	}
	e.preventDefault();
});

$("body").on("click", "#dropdown_language a", function(e) {
	$("#btn_language > span:first-child").text($(this).text());
	$("#btn_language").data("language", $(this).data("language"));
	e.preventDefault();
});

$("body").on("click", ".filament-usage", function(e) {
	if (window.matchMedia('(max-width: 991px)').matches) {
		// Display filament usage for small devices on click
		showMessage("info", T("Filament usage"), $(this).attr("title"));
	}
	e.preventDefault();
});

$("body").on("click", ".gcode", function(e) {
	if (isConnected) {
		// If this G-Code isn't performed by a button, treat it as a manual input
		sendGCode($(this).data("gcode"), !($(this).is(".btn")));
	}
	e.preventDefault();
});

$("body").on("click", ".gcode-directory", function(e) {
	setGCodeDirectory(currentGCodeDirectory + "/" + $(this).parents("tr").data("directory"));
	gcodeUpdateIndex = -1;
	updateGCodeFiles();
	e.preventDefault();
});

$("body").on("click", ".heater-temp", function(e) {
	var inputElement = $(this).parents("div.input-group").find("input");
	var activeOrStandby = (inputElement.prop("id").match("active$")) ? "S" : "R";
	var temperature = $(this).data("temp");

	if (inputElement.prop("id").indexOf("all") == -1) {
		var heater = inputElement.prop("id").match("_h(.)_")[1];

		var gcode = "";
		getToolsByHeater(heater).forEach(function(tool) {
			gcode += "G10 P" + tool + " " + activeOrStandby + temperature + "\n";
		});
		sendGCode(gcode);
	} else {
		if (toolMapping != undefined) {
			var gcode = "";
			for(var i=0; i<toolMapping.length; i++) {
				var number = (toolMapping[i].hasOwnProperty("number") ? toolMapping[i].number : i + 1);
				if ($.inArray(0, toolMapping[i].heaters) == -1) {
					// Make sure we don't set temperatures for the heated bed
					gcode += "G10 P" + number + " " + activeOrStandby + $(this).val() + "\n";
				}
			}
			sendGCode(gcode);
		}
	}

	e.preventDefault();
});

$("body").on("click", ".macro-directory", function(e) {
	setMacroDirectory(currentMacroDirectory + "/" + $(this).parents("tr").data("directory"));
	macroUpdateIndex = -1;
	updateMacroFiles();
	e.preventDefault();
});

$("body").on("click", "#ol_gcode_directory a", function(e) {
	setGCodeDirectory($(this).data("directory"));
	gcodeUpdateIndex = -1;
	updateGCodeFiles();
	e.preventDefault();
});

$("body").on("click", "#ol_macro_directory a", function(e) {
	setMacroDirectory($(this).data("directory"));
	macroUpdateIndex = -1;
	updateMacroFiles();
	e.preventDefault();
});

var draggingObject;

function fileDragStart(e) {
	draggingObject = $(this).closest("tr");
	// The following doesn't work, although it's recommended. I wonder who invented this crappy API...
	//e.dataTransfer.setData('text/html', this.innerHTML);
}

function fileDragEnd(e) {
	draggingObject = undefined;
}

$("#table_gcode_files, #table_macro_files").on("dragover", "tr", function(e) {
	var row = $(e.target).closest("tr");
	if (draggingObject != undefined && row != undefined) {
		var dir = row.data("directory");
		if (dir != undefined && dir != draggingObject.data("directory")) {
			e.stopPropagation();
			e.preventDefault();
		}
	}
});

$("#table_gcode_files, #table_macro_files").on("drop", "tr", function(e) {
	if (draggingObject == undefined) {
		return;
	}

	var sourcePath = draggingObject.data("item");
	var targetPath = $(e.target).closest("tr").data("directory");
	if (targetPath != undefined && sourcePath != targetPath) {
		if (currentPage == "files") {
			targetPath = currentGCodeDirectory + "/" + targetPath + "/" + sourcePath;
			sourcePath = currentGCodeDirectory + "/" + sourcePath;
		} else {
			targetPath = currentMacroDirectory + "/" + targetPath + "/" + sourcePath;
			sourcePath = currentMacroDirectory + "/" + sourcePath;
		}

		$.ajax("rr_move?old=" + encodeURIComponent(sourcePath) + "&new=" + encodeURIComponent(targetPath), {
			dataType: "json",
			row: draggingObject,
			success: function(response) {
				if (response.err == 0) {
					// We can never run out of files/dirs if we move FSOs to sub-directories...
					this.row.remove();
				}
			}
		});

		e.stopPropagation();
		e.preventDefault();
	}
});

$("#ol_gcode_directory, #ol_macro_directory").on("dragover", "a", function(e) {
	if (draggingObject != undefined) {
		e.stopPropagation();
		e.preventDefault();
	}
});

$("#ol_gcode_directory, #ol_macro_directory").on("drop", "a", function(e) {
	if (draggingObject == undefined) {
		return;
	}
	draggingObject = draggingObject.closest("tr");

	var targetDir = $(e.target).data("directory");
	if (targetDir != undefined) {
		var sourcePath = draggingObject.data("item");
		var targetPath, fileTable;
		if (currentPage == "files") {
			targetPath = targetDir + "/" + sourcePath;
			sourcePath = currentGCodeDirectory + "/" + sourcePath;
			fileTable = $("#table_gcode_files > tbody");
		} else {
			targetPath = targetDir + "/" + sourcePath;
			sourcePath = currentMacroDirectory + "/" + sourcePath;
			fileTable = $("#table_macro_files > tbody");
		}

		$.ajax("rr_move?old=" + encodeURIComponent(sourcePath) + "&new=" + encodeURIComponent(targetPath), {
			dataType: "json",
			row: draggingObject,
			table: fileTable,
			success: function(response) {
				if (response.err == 0) {
					this.row.remove();
					if (this.table.children().length == 0) {
						if (currentPage == "files") {
							gcodeUpdateIndex = -1;
							updateGCodeFiles();
						} else {
							macroUpdateIndex = -1;
							updateMacroFiles();
						}
					}
				}
			}
		});

		e.stopPropagation();
		e.preventDefault();
	}
});

$("body").on("click", ".tool", function(e) {
	changeTool($(this).data("tool"));
	e.preventDefault();
});

$("body").on("hidden.bs.popover", function() {
	$(this).popover("destroy");
});

/* Static GUI Events */

$("#a_heaters_off").click(function(e) {
	if (isConnected) {
		var gcode = "";

		// Turn off nozzle heaters
		if (toolMapping != undefined) {
			for(var i=0; i<toolMapping.length; i++) {
				var number = (toolMapping[i].hasOwnProperty("number") ? toolMapping[i].number : i + 1);

				var temps = [], tempString = "";
				toolMapping[i].heaters.forEach(function() { temps.push("-273.15"); });
				tempString = temps.reduce(function(a, b) { return a + ":" + b; });

				gcode = "G10 P" + number + " R" + tempString + " S" + tempString + "\n";
			}
		}

		// Turn off bed
		if (heatedBed) {
			gcode += "M140 S-273.15\n";
		}

		// Turn off chamber
		if (chamber) {
			gcode += "M141 S-273.15\n";
		}

		sendGCode(gcode);
	}
	e.preventDefault();
});

$("#a_webcam").click(function(e) {
	if ($("#img_webcam").hasClass("hidden")) {
		$("#span_webcam").removeClass("glyphicon-menu-up").addClass("glyphicon-menu-down");
		$("#img_webcam").removeClass("hidden");
	} else {
		$("#span_webcam").removeClass("glyphicon-menu-down").addClass("glyphicon-menu-up");
		$("#img_webcam").addClass("hidden");
	}
	$(this).blur();
	e.preventDefault();
});

$("#btn_add_gcode").click(function(e) {
	var item =	'<tr><td><label class="label label-primary">' + $("#input_gcode").val().trim() + '</label></td><td>' + $("#input_gcode_description").val().trim() + '</td><td>';
	item +=		'<button class="btn btn-sm btn-danger btn-delete-parent" title="' + T("Delete this G-Code item") + '">';
	item += 	'<span class="glyphicon glyphicon-trash"></span></button></td></tr>';
	$("#table_gcodes").append(item);

	e.preventDefault();
});

$("#btn_add_head_temp").click(function(e) {
	var temperature = checkBoundaries($("#input_add_head_temp").val(), 0, -273.15, 300);
	var type = $('input[name="temp_selection"]:checked').val();

	var item =	'<li class="list-group-item col-xs-6 col-lg-3" data-temperature="' + temperature + '">' + temperature + ' °C';
	item +=		'<button class="btn btn-danger btn-sm btn-delete-parent pull-right" title="' + T("Delete this temperature item") + '">';
	item +=		'<span class="glyphicon glyphicon-trash"></span></button></li>';
	$("#ul_" + type + "_temps").append(item);

	e.preventDefault();
});

$("#btn_add_bed_temp").click(function(e) {
	var temperature = checkBoundaries($("#input_add_bed_temp").val(), 0, -273.15, 180);

	var item =	'<li class="list-group-item col-md-6" data-temperature="' + temperature + '">' + temperature + ' °C';
	item +=		'<button class="btn btn-danger btn-sm btn-delete-parent pull-right" title="' + T("Delete this temperature item") + '">';
	item +=		'<span class="glyphicon glyphicon-trash"></span></button></li>';
	$("#ul_bed_temps").append(item);

	e.preventDefault();
});

$("#btn_add_tool").click(function(e) {
	var gcode = "M563 P" + $("#input_tool_number").val();

	var drives = $("input[name='tool_drives']:checked");
	if (drives != undefined) {
		var driveList = [];
		drives.each(function() { driveList.push($(this).val()); });
		gcode += " D" + driveList.reduce(function(a, b) { return a + ":" + b; });
	}

	var heaters = $("input[name='tool_heaters']:checked");
	if (heaters != undefined) {
		var heaterList = [];
		heaters.each(function() { heaterList.push($(this).val()); });
		gcode += " H" + heaterList.reduce(function(a, b) { return a + ":" + b; });
	}

	sendGCode(gcode);
	extendedStatusCounter = settings.extendedStatusInterval;

	e.preventDefault();
});

$("#btn_cancel").click(function() {
	sendGCode("M0 H1");	// Stop / Cancel Print, but leave all the heaters on
	$(this).addClass("disabled");
});

$("#btn_cancel_upload").click(function() {
	cancelUpload();
});

$(".btn-connect").click(function() {
	if (!isConnected) {
		// Attempt to connect with the last-known password first
		connect(sessionPassword, true);
	} else {
		disconnect();
	}
});

$(".btn-emergency-stop").click(function() {
	if (settings.confirmStop) {
		showConfirmationDialog(T("Emergency STOP"), T("This will turn off everything and perform a software reset.<br/><br/>Are you REALLY sure you want to do this?"), function() {
			sendGCode("M112\nM999");
		});
	} else {
		sendGCode("M112\nM999");
	}
});

$("#btn_fw_diagnostics").click(function() {
	if (isConnected) {
		sendGCode("M122");
		showPage("console");
	}
});

$("#btn_clear_log").click(function(e) {
	$("#console_log").html("");
	log("info", "<strong>" + T("Message Log cleared!") + "</strong>");
	e.preventDefault();
});

// TODO: deal with mixing drives
$("#btn_extrude").click(function(e) {
	var feedrate = $("#panel_extrude input[name=feedrate]:checked").val() * 60;
	var amount = $("#panel_extrude input[name=feed]:checked").val();
	sendGCode("M120\nM83\nG1 E" + amount + " F" + feedrate + "\nM121");
});
$("#btn_retract").click(function(e) {
	var feedrate = $("#panel_extrude input[name=feedrate]:checked").val() * 60;
	var amount = $("#panel_extrude input[name=feed]:checked").val();
	sendGCode("M120\nM83\nG1 E-" + amount + " F" + feedrate + "\nM121");
});

$(".btn-hide-info").click(function() {
	if ($(this).hasClass("active")) {
		$("#row_info").addClass("hidden-xs hidden-sm");
		$("#main_content").addClass("content-collapsed-padding");
		setTimeout(function() {
			$(".btn-hide-info").removeClass("active");
		}, 100);
	} else {
		$("#row_info").removeClass("hidden-xs hidden-sm");
		$("#main_content").removeClass("content-collapsed-padding");
		setTimeout(function() {
			$(".btn-hide-info").addClass("active");
		}, 100);
	}
	$(this).blur();
});

$(".btn-home-x").resize(function() {
	if (geometry != "delta") {
		var width = $(this).parent().width();
		if (width > 0) {
			$("#btn_homeall").css("min-width", width);
		}
	}
}).resize();

$("#mobile_home_buttons button, #btn_homeall, #table_move_head a").click(function(e) {
	$this = $(this);
	if ($this.data("home") != undefined) {
		if ($this.data("home") == "all") {
			sendGCode("G28");
		} else {
			sendGCode("G28 " + $this.data("home"));
		}
	} else {
		var moveString = "M120\nG91\nG1";
		if ($this.data("x") != undefined) {
			moveString += " X" + $this.data("x");
		}
		if ($this.data("y") != undefined) {
			moveString += " Y" + $this.data("y");
		}
		if ($this.data("z") != undefined) {
			moveString += " Z" + $this.data("z");
		}
		moveString += " F" + settings.moveFeedrate + "\nM121";
		sendGCode(moveString);
	}
	e.preventDefault();
});

$("#btn_new_gcode_directory").click(function() {
	showTextInput(T("New directory"), T("Please enter a name:"), function(value) {
		$.ajax("rr_mkdir?dir=" + currentGCodeDirectory + "/" + value, {
			dataType: "json",
			success: function(response) {
				if (response.err == 0) {
					gcodeUpdateIndex = -1;
					updateGCodeFiles();
				} else {
					showMessage("warning", T("Error"), T("Could not create this directory!"));
				}
			}
		});
	});
});

$("#btn_new_macro_directory").click(function() {
	showTextInput(T("New directory"), T("Please enter a name:"), function(value) {
		$.ajax("rr_mkdir?dir=" + currentMacroDirectory + "/" + value, {
			dataType: "json",
			success: function(response) {
				if (response.err == 0) {
					macroUpdateIndex = -1;
					updateMacroFiles();
				} else {
					showMessage("warning", T("Error"), T("Could not create this directory!"));
				}
			}
		});
	});
});

$("#btn_pause").click(function() {
	if (isPaused) {
		sendGCode("M24");	// Resume
	} else if (isPrinting) {
		sendGCode("M25");	// Pause
	}
	$(this).addClass("disabled");
});

$(".btn-upload").click(function(e) {
	if (!$(this).is(".disabled")) {
		$("#input_file_upload").data("type", $(this).data("type")).click();
	}
	e.preventDefault();
});

$("#btn_reset_settings").click(function(e) {
	showConfirmationDialog(T("Reset Settings"), T("Are you sure you want to revert to Factory Settings?"), function() {
		if (defaultSettings.language != settings.language) {
			showMessage("info", T("Language has changed"), T("You have changed the current language. Please reload the web interface to apply this change."), 0);
		}
		settings = jQuery.extend(true, {}, defaultSettings);
		$("#btn_language").data("language", "en").children("span:first-child").text("English");
		saveSettings();
		applySettings();
	});
	e.preventDefault();
});

["print", "gcode", "macro", "generic"].forEach(function(type) {
	var child = $(".btn-upload[data-type='" + type + "']");

	// Drag Enter
	child.on("dragover", function(e) {
		$(this).removeClass($(this).data("style")).addClass("btn-success");
		e.preventDefault();  
		e.stopPropagation();
	});

	// Drag Leave
	child.on("dragleave", function(e) {
		$(this).removeClass("btn-success").addClass($(this).data("style"));
		e.preventDefault();  
		e.stopPropagation();
	});

	// Drop
	child.on("drop", function(e) {
		$(this).removeClass("btn-success").addClass($(this).data("style"));
		e.preventDefault();
		e.stopPropagation();

		var files = e.originalEvent.dataTransfer.files;
		if (!$(this).is(".disabled") && files != null && files.length > 0) {
			// Start new file upload
			startUpload($(this).data("type"), files, false);
		}
	});
});

$(".gcode-input").submit(function(e) {
	if (isConnected) {
		var gcode = $(this).find("input").val();
		if (settings.uppercaseGCode) {
			gcode = gcode.toUpperCase();
		}
		sendGCode(gcode, true);
		$(this).find("input").select();
	}
	e.preventDefault();
});

// Make the auto-complete dropdown items look proper.
// This should be replaced by proper CSS someday, but
// for now we only check which elements may float around.
$(".div-gcodes").bind("shown.bs.dropdown", function() {
	var maxWidth = 0;
	$(this).find("ul > li > a").each(function() {
		var rowWidth = 0;
		$(this).find("span").each(function() {
			rowWidth += $(this).width();
		});

		if (rowWidth > maxWidth) {
			maxWidth = rowWidth;
		}
	});

	if (maxWidth > 0) {
		$(this).find("ul > li > a").each(function() {
			var rowWidth = 0;
			$(this).find("span").each(function() {
				rowWidth += $(this).width();
			});

			if (rowWidth < maxWidth) {
				$(this).addClass("gcode-float");
			}
		});
	}
});

$("#frm_settings").submit(function(e) {
	saveSettings();
	applySettings();
	e.preventDefault();
});

$("#frm_settings > ul > li a").on("shown.bs.tab", function(e) {
	$("#frm_settings > ul li").removeClass("active");
	var links = $('#frm_settings > ul > li a[href="' + $(this).attr("href") + '"]');
	$.each(links, function() {
		$(this).parent().addClass("active");
	});
});

$("input[type='number']").focus(function() {
	var input = $(this);
	setTimeout(function() {
		input.select();
	}, 10);
});

$("input[name='temp_selection']:radio").change(function() {
	if ($(this).val() == "active") {
		$("#ul_active_temps").removeClass("hidden");
		$("#ul_standby_temps").addClass("hidden");
	} else {
		$("#ul_standby_temps").removeClass("hidden");
		$("#ul_active_temps").addClass("hidden");
	}
});

$("#input_file_upload").change(function(e) {
	if (this.files.length > 0) {
		// For POST uploads, we need file blobs
		startUpload($(this).data("type"), this.files, false);
	}
});

$("#input_temp_bed").keydown(function(e) {
	var enterKeyPressed = (e.which == 13);
	enterKeyPressed |= (e.which == 9 && window.matchMedia('(max-width: 991px)').matches); // need this for Android
	if (isConnected && enterKeyPressed) {
		sendGCode("M140 S" + $(this).val());
		$(this).select();

		e.preventDefault();
	}
});

$("#input_temp_chamber").keydown(function(e) {
	var enterKeyPressed = (e.which == 13);
	enterKeyPressed |= (e.which == 9 && window.matchMedia('(max-width: 991px)').matches); // need this for Android
	if (isConnected && enterKeyPressed) {
		sendGCode("M141 S" + $(this).val());
		$(this).select();

		e.preventDefault();
	}
});

$("input[id^='input_temp_h']").keydown(function(e) {
	var enterKeyPressed = (e.which == 13);
	enterKeyPressed |= (e.which == 9 && window.matchMedia('(max-width: 991px)').matches); // need this for Android
	if (isConnected && enterKeyPressed) {
		var activeOrStandby = ($(this).prop("id").match("active$")) ? "S" : "R";
		var heater = $(this).prop("id").match("_h(.)_")[1];
		var temperature = $(this).val();

		var gcode = "";
		getToolsByHeater(heater).forEach(function(toolNumber) {
			gcode += "G10 P" + toolNumber + " " + activeOrStandby + temperature + "\n";
		});
		sendGCode(gcode);

		$(this).select();
		e.preventDefault();
	}
});

$("#input_temp_all_active, #input_temp_all_standby").keydown(function(e) {
	if (isConnected && e.which == 13) {
		if (toolMapping != undefined) {
			var activeOrStandby = ($(this).prop("id").match("active$")) ? "S" : "R";
			var temperature = $(this).val();

			var gcode = "";
			for(var i=0; i<toolMapping.length; i++) {
				var number = (toolMapping[i].hasOwnProperty("number") ? toolMapping[i].number : i + 1);
				if ($.inArray(0, toolMapping[i].heaters) == -1) {
					// Make sure we don't set temperatures for the heated bed
					gcode += "G10 P" + number + " " + activeOrStandby + $(this).val() + "\n";
				}
			}
			sendGCode(gcode);
		}

		e.preventDefault();
	}
});

$(".navbar-brand").click(function(e) { e.preventDefault(); });
$(".navbar-brand.visible-xs > abbr").click(function(e) {
	showMessage("warning", T("Warning"), T("Some axes are not homed"));
	e.preventDefault();
});

$(".navlink").click(function(e) {
	$(this).blur();
	showPage($(this).data("target"));
	e.preventDefault();
});

$("#page_listitems input").on("input", function() {
	// Validate form controls
	$("#btn_add_gcode").toggleClass("disabled", $("#input_gcode").val().trim() == "" || $("#input_gcode_description").val().trim() == "");
	$("#btn_add_head_temp").toggleClass("disabled", isNaN(parseFloat($("#input_add_head_temp").val())));
	$("#btn_add_bed_temp").toggleClass("disabled", isNaN(parseFloat($("#input_add_bed_temp").val())));
});

$("#page_listitems input").keydown(function(e) {
	if (e.which == 13) {
		var button = $(this).parents("div:not(.input-group):eq(0)").find("button");
		if (!button.hasClass("disabled")) {
			button.click();
		}
		e.preventDefault();
	}
});

$("#panel_control_misc label.btn").click(function() {
	if ($(this).find("input").val() == 1) {		// ATX on
		sendGCode("M80");
	} else {									// ATX off
		showConfirmationDialog(T("ATX Power"), T("Do you really want to turn off ATX power?<br/><br/>This will turn off all drives, heaters and fans."), function() {
			sendGCode("M81");
		});
	}
});

$("#panel_extrude label.btn").click(function() {
	$(this).parent().find("label.btn").removeClass("btn-primary").addClass("btn-default");
	$(this).removeClass("btn-default").addClass("btn-primary");
});

$(".span-refresh-files").click(function() {
	gcodeUpdateIndex = -1;
	updateGCodeFiles();
	$(".span-refresh-files").addClass("hidden");
});

$(".span-refresh-macros").click(function() {
	macroUpdateIndex = -1;
	updateMacroFiles();
	$(".span-refresh-macros").addClass("hidden");
});

$(document).delegate('#text_config', 'keydown', function(e) {
	var keyCode = e.keyCode || e.which;

	if (keyCode == 9) {
		e.preventDefault();
		var start = $(this).get(0).selectionStart;
		var end = $(this).get(0).selectionEnd;

		// set textarea value to: text before caret + tab + text after caret
		$(this).val($(this).val().substring(0, start)
				+ "\t"
				+ $(this).val().substring(end));

		// put caret at right position again
		$(this).get(0).selectionStart = $(this).get(0).selectionEnd = start + 1;
	}
});

$(".panel-chart").resize(function() {
	resizeCharts();
});

$('a[href="#page_general"], a[href="#page_listitems"]').on('shown.bs.tab', function () {
	$("#row_save_settings, #btn_reset_settings").removeClass("hidden");
});

$('a[href="#page_config"]').on('shown.bs.tab', function() {
	$("#row_save_settings, #btn_reset_settings").addClass("hidden");
	$("#text_config").trigger("input");
	getConfigFile();
});

$('a[href="#page_machine"], a[href="#page_tools"]').on('shown.bs.tab', function () {
	$("#row_save_settings").addClass("hidden");
});

$("#table_heaters a").click(function(e) {
	if (isConnected && lastStatusResponse != undefined) {
		if ($(this).parents("#tr_bed").length > 0) {
			var bedState = lastStatusResponse.temps.bed.state;
			if (bedState == 3) {
				showMessage("danger", T("Heater Fault"), T("<strong>Error:</strong> A heater fault has occured on this particular heater.<br/><br/>Please turn off your machine and check your wiring for loose connections."));
			} else if (bedState == 2) {
				// Put bed into standby mode
				sendGCode("M144");
			} else {
				// Bed is either off or in standby mode, send M140 to turn it back on
				sendGCode("M140 S" + $("#input_temp_bed").val());
			}
		} else {
			var heater = $(this).parents("tr").index();
			var heaterState = lastStatusResponse.temps.heads.state[heater - 1];
			if (heaterState == 3) {
				showMessage("danger", T("Heater Fault"), T("<strong>Error:</strong> A heater fault has occured on this particular heater.<br/><br/>Please turn off your machine and check your wiring for loose connections."));
			} else {
				var tools = getToolsByHeater(heater), hasToolSelected = false;
				tools.forEach(function(tool) {
					if (tool == lastStatusResponse.currentTool) {
						hasToolSelected = true;
					}
				});

				if (hasToolSelected) {
					changeTool(-1);
					$(this).blur();
				} else if (tools.length == 1) {
					changeTool(tools[0]);
					$(this).blur();
				} else if (tools.length > 0) {
					var popover = $(this).parent().children("div.popover");
					if (popover.length) {
						$(this).popover("hide");
						$(this).blur();
					} else {
						var content = '<div class="btn-group-vertical btn-group-vertical-justified">';
						tools.forEach(function(toolNumber) {
							content += '<div class="btn-group"><a href="#" class="btn btn-default btn-sm tool" data-tool="' + toolNumber + '">T' + toolNumber + '</a></div>';
						});
						content += '</div>';

						$(this).popover({ 
							content: content,
							html: true,
							title: T("Select Tool"),
							trigger: "manual",
							placement: "bottom",
						}).popover("show");
					}
				}
			}
		}
	}
	e.preventDefault();
});

$("#table_define_tool input[type='checkbox']").change(function() {
	var isChecked = $(this).is(":checked");
	$(this).parents("label").toggleClass("btn-primary", isChecked).toggleClass("btn-default", !isChecked);

	validateAddTool();
});

$("#table_define_tool input[type='number']").on("input", function() {
	validateAddTool();
});

function validateAddTool() {
	var toolNumber = parseInt($("#input_tool_number").val());
	var disableButton =	(!isConnected) ||
		(isNaN(toolNumber) || toolNumber < 0 || toolNumber > 255) ||
		(toolMapping != undefined && getTool(toolNumber) != undefined) ||
		($("input[name='tool_heaters']:checked").length + $("input[name='tool_drives']:checked").length == 0);
	$("#btn_add_tool").toggleClass("disabled", disableButton);
}

$("#table_define_tool input[type='number']").keydown(function(e) {
	if (e.which == 13) {
		if (!$("#btn_add_tool").hasClass("disabled")) {
			$("#btn_add_tool").click();
		}
		e.preventDefault();
	}
});

$("#table_heaters tr > th:first-child > a").blur(function() {
	$(this).popover("hide");
});

$("#ul_control_dropdown").click(function(e) {
	$(this).find(".dropdown").removeClass("open");
	if ($(e.target).is("a")) {
		$(this).parents(".dropdown").removeClass("open");
	} else {
		e.stopPropagation();
	}
});

$("#ul_control_dropdown .btn-active-temp, #ul_control_dropdown .btn-standby-temp").click(function(e) {
	$(this).parent().toggleClass("open");
	e.stopPropagation();
});

/* Temperature charts */

function addLayerData(lastLayerTime, updateGui) {
	layerData.push([layerData.length + 1, lastLayerTime]);
	if (lastLayerTime > maxLayerTime) {
		maxLayerTime = lastLayerTime;
	}

	if (updateGui) {
		$("#td_last_layertime").html(convertSeconds(lastLayerTime)).addClass("layer-done-animation");
		setTimeout(function() {
			$("#td_last_layertime").removeClass("layer-done-animation");
		}, 2000);

		drawPrintChart();
	}
}

function drawTemperatureChart()
{
	// Only draw the chart if it's possible
	if ($("#chart_temp").width() === 0) {
		refreshTempChart = true;
		return;
	}

	// Prepare the data
	var preparedBedTemps = [];
	for(var i=0; i<recordedBedTemperatures.length; i++) {
		preparedBedTemps.push([i, recordedBedTemperatures[i]]);
	}
	var preparedChamberTemps = [];
	for(var i=0; i<recordedChamberTemperatures.length; i++) {
		preparedChamberTemps.push([i, recordedChamberTemperatures[i]]);
	}
	var preparedHeadTemps = [[], [], [], [], [], []], heaterSamples;
	for(var head=0; head<recordedHeadTemperatures.length; head++) {
		heaterSamples = recordedHeadTemperatures[head];
		for(var k=0; k<heaterSamples.length; k++) {
			preparedHeadTemps[head].push([k, heaterSamples[k]]);
		}
	}

	// Draw it
	if (tempChart == undefined) {
		tempChart = $.plot("#chart_temp", [preparedBedTemps, preparedHeadTemps[0], preparedHeadTemps[1], preparedHeadTemps[2], preparedHeadTemps[3], preparedHeadTemps[4], preparedHeadTemps[5], preparedChamberTemps], tempChartOptions);
	} else {
		tempChart.setData([preparedBedTemps, preparedHeadTemps[0], preparedHeadTemps[1], preparedHeadTemps[2], preparedHeadTemps[3], preparedHeadTemps[4], preparedHeadTemps[5], preparedChamberTemps]);
		tempChart.setupGrid();
		tempChart.draw();
	}

	refreshTempChart = false;
}

function drawPrintChart() {
	// Only draw the chart if it's possible
	if ($("#chart_print").width() === 0) {
		refreshPrintChart = true;
		return;
	}

	// Find absolute maximum values for the X axis
	var maxX = 25, maxPanX = 25;
	if (layerData.length < 21) {
		maxX = maxPanX = 20;
	} else if (layerData.length < 25) {
		maxX = maxPanX = layerData.length;
	} else {
		maxPanX = layerData.length;
	}
	printChartOptions.xaxis.max = maxX;
	printChartOptions.xaxis.panRange = [1, maxPanX];
	printChartOptions.xaxis.zoomRange = [25, maxPanX];

	// Find max visible value for Y axis
	var maxY = 30;
	if (layerData.length > 2) {
		for(var i=(layerData.length > 26) ? layerData.length - 25 : 1; i < layerData.length; i++) {
			if (maxY < layerData[i][1] * 1.1) {
				maxY = layerData[i][1] * 1.1;
			}
		}
	} else if (layerData.length > 0) {
		maxY = layerData[0][1] * 1.1;
	}
	printChartOptions.yaxis.max = maxY;
	printChartOptions.yaxis.panRange = [0, (maxLayerTime < maxY) ? maxY : maxLayerTime];
	printChartOptions.yaxis.zoomRange = [30, (maxLayerTime < maxY) ? maxY : maxLayerTime];

	// Update chart and pan to the right
	printChart = $.plot("#chart_print", [layerData], printChartOptions);
	printChart.pan({ left: 99999 });
	refreshPrintChart = false;

	// Add hover events to chart
	$("#chart_print").unbind("plothover").bind("plothover", function (event, pos, item) {
		if (item) {
			var layer = item.datapoint[0];
			if (layer == 0) {
				layer = 1;
			}

			$("#layer_tooltip").html(T("Layer {0}: {1}", layer, convertSeconds(item.datapoint[1])))
				.css({top: item.pageY + 5, left: item.pageX + 5})
				.fadeIn(200);
		} else {
			$("#layer_tooltip").hide();
		}
	});
}

function resizeCharts() {
	var headsHeight = $("#table_heaters").height();
	var statusHeight = 0;
	$("#div_status table").each(function() {
		statusHeight += $(this).outerHeight();
	});

	var max = (headsHeight > statusHeight) ? headsHeight : statusHeight;
	max -= tempChartPadding;

	if (max > 0) {
		$("#chart_temp").css("height", max);
	}

	if (refreshTempChart) {
		drawTemperatureChart();
	}
	if (refreshPrintChart) {
		drawPrintChart();
	}
}

/* Sliders */

$('#slider_fan_control').slider({
	enabled: false,
	id: "fan_control",
	min: 0,
	max: 100,
	step: 1,
	value: 35,
	tooltip: "always",

	formatter: function(value) {
		return value + " %";
	}
}).on("slideStart", function() {
	fanSliderActive = true;
}).on("slideStop", function(slideEvt) {
	if (isConnected && !isNaN(slideEvt.value)) {
		sendGCode("M106 S" + (slideEvt.value / 100.0));
		$("#slider_fan_print").slider("setValue", slideEvt.value);
	}
	fanSliderActive = false;
});

$('#slider_fan_print').slider({
	enabled: false,
	id: "fan_print",
	min: 0,
	max: 100,
	step: 1,
	value: 35,
	tooltip: "always",

	formatter: function(value) {
		return value + " %";
	}
}).on("slideStart", function() {
	fanSliderActive = true;
}).on("slideStop", function(slideEvt) {
	if (isConnected && !isNaN(slideEvt.value)) {
		sendGCode("M106 S" + (slideEvt.value / 100.0));
		$("#slider_fan_control").slider("setValue", slideEvt.value);
	}
	fanSliderActive = false;
});

for(var extr=1; extr<=6; extr++) {
	$('#slider_extr_' + extr).slider({
		enabled: false,
		id: "extr-" + extr,
		min: 50,
		max: 150,
		step: 1,
		value: 100,
		tooltip: "always",

		formatter: function(value) {
			return value + " %";
		}
	}).on("slideStart", function() {
		extrSliderActive = true;
	}).on("slideStop", function(slideEvt) {
		if (isConnected && !isNaN(slideEvt.value)) {
			sendGCode("M221 D" + $(this).data("drive") + " S" + slideEvt.value);
		}
		extrSliderActive = false;
	});
}

$('#slider_speed').slider({
	enabled: false,
	id: "speed",
	min: 20,
	max: 200,
	step: 1,
	value: 100,
	tooltip: "always",

	formatter: function(value) {
		return value + " %";
	}
}).on("slideStart", function() {
	speedSliderActive = true;
}).on("slideStop", function(slideEvt) {
	if (isConnected && !isNaN(slideEvt.value)) {
		sendGCode("M220 S" + slideEvt.value);
	}
	speedSliderActive = false;
});

/* Modals */

function showConfirmationDialog(title, message, action) {
	$("#modal_confirmation h4").html('<span class="glyphicon glyphicon-question-sign"></span> ' + title);
	$("#modal_confirmation p").html(message);
	$("#modal_confirmation button:first-child").off().one("click", action);
	$("#modal_confirmation").modal("show");
	$("#modal_confirmation .btn-success").focus();
}

function showTextInput(title, message, action) {
	$("#modal_textinput h4").html(title);
	$("#modal_textinput p").html(message);
	$("#modal_textinput input").val("");
	$("#modal_textinput form").off().submit(function(e) {
		$("#modal_textinput").modal("hide");
		var value = $("#modal_textinput input").val();
		if (value.trim() != "") {
			action(value);
		}
		e.preventDefault();
	});
	$("#modal_textinput").modal("show");
}

function showMessage(type, title, message, timeout, allowDismiss) {
	// Find a suitable icon
	var icon = "glyphicon glyphicon-info-sign";
	if (type == "warning") {
		icon = "glyphicon glyphicon-warning-sign";
	} else if (type == "danger") {
		icon = "glyphicon glyphicon-exclamation-sign";
	}

	// Check if the title can be displayed as bold text
	if (title != "")
	{
		if (title.indexOf("<strong>") == -1)
		{
			title = "<strong>" + title + "</strong>";
		}
		title += "<br/><br/>";
	}

	// Show the notification
	var notifySettings = { icon: "glyphicon glyphicon-" + icon,
		title: title,
		message: message};
	var options = { type: type,
		allow_dismiss: (allowDismiss == undefined) ? true : allowDismiss,
		delay: (timeout == undefined) ? settings.notificationTimeout : timeout };
	return $.notify(notifySettings, options);
}

function showPasswordPrompt() {
	$('#input_password').val("");
	$("#modal_pass_input").modal("show");
	$("#modal_pass_input").one("hide.bs.modal", function() {
		// The network request will take a few ms anyway, so no matter if the user has
		// cancelled the password input, we can reset the Connect button here...
		$(".btn-connect").removeClass("btn-warning disabled").addClass("btn-info").find("span:not(.glyphicon)").text(T("Connect"));
	});
}

function showUpdateMessage() {
	var notifySettings = { icon: "glyphicon glyphicon-time",
		title: "<strong>" + T("Updating Firmware...") + "</strong><br/><br/>",
		message: T("Please wait while the firmware is being updated..."),
		progress: settings.updateReconnectDelay / 100,
	};
	var options = { type: "success",
		allow_dismiss: false,
		delay: settings.updateReconnectDelay,
		timeout: 1000,
		showProgressbar: true
	};
	return $.notify(notifySettings, options);
}

$("#modal_pass_input, #modal_textinput").on('shown.bs.modal', function() {
	$(this).find("input").focus()
});

$(".modal").on("hidden.bs.modal", function() {
	// Bootstrap bug: Padding is added to the right, but never cleaned
	$("body").css("padding-right", "");
});

$("#form_password").submit(function(e) {
	$("#modal_pass_input").off("hide.bs.modal").modal("hide");
	connect($("#input_password").val(), false);
	e.preventDefault();
});

/* GUI Helpers */

function addBedTemperature(temperature) {
	// Drop-Down item
	$(".ul-bed-temp").append('<li><a href="#" class="bed-temp" data-temp="' + temperature + '">' + temperature + ' °C</a></li>');
	$(".btn-bed-temp").removeClass("disabled");

	// Entry on Settings page
	var item =	'<li class="list-group-item col-md-6" data-temperature="' + temperature + '">' + temperature + '  °C';
	item +=		'<button class="btn btn-danger btn-sm btn-delete-parent pull-right" title="' + T("Delete this temperature item") + '">';
	item +=		'<span class="glyphicon glyphicon-trash"></span></button></li>';
	$("#ul_bed_temps").append(item);
}

function addDefaultGCode(label, gcode) {
	// Drop-Down item
	var item =	'<li><a href="#" class="gcode" data-gcode="' + gcode + '">';
	item +=		'<span>' + label + '</span>';
	item +=		'<span class="label label-primary">' + gcode + '</span>';
	item +=		'</a></li>';

	$(".ul-gcodes").append(item);
	$(".btn-gcodes").removeClass("disabled");

	// Entry on Settings page
	item =	'<tr><td><label class="label label-primary">' + gcode + '</label></td><td>' + label + '</td><td>';
	item +=	'<button class="btn btn-sm btn-danger btn-delete-parent" title="' + T("Delete this G-Code item") + '">';
	item += '<span class="glyphicon glyphicon-trash"></span></button></td></tr>';
	$("#table_gcodes").append(item);
}

function addGCodeFile(filename) {
	$("#page_files h1").addClass("hidden");

	var row =	'<tr data-item="' + filename + '"><td></td><td></td>';
	row +=		'<td><span class="glyphicon glyphicon-asterisk"></span>' + filename + '</td>';
	row +=		'<td class="hidden-xs">' + T("loading") + '</td>';
	row +=		'<td>loading</td>';
	row +=		'<td>loading</td>';
	row +=		'<td class="hidden-xs">loading</td>';
	row +=		'<td class="hidden-xs hidden-sm">' + T("loading") + '</td></tr>';

	$("#table_gcode_files").removeClass("hidden");
	return $(row).appendTo("#table_gcode_files");
}

function addMacroFile(filename) {
	// Control Page
	if (currentMacroDirectory == "/macros") {
		var label = stripMacroFilename(filename);
		var macroButton =	'<div class="btn-group">';
		macroButton +=		'<button class="btn btn-default btn-macro btn-sm" data-macro="/macros/' + filename + '">' + label + '</button>';
		macroButton +=		'</div>';

		$("#panel_macro_buttons h4").addClass("hidden");
		$("#panel_macro_buttons .btn-group-vertical").append(macroButton);
	}

	// Macro Page
	var row =	'<tr data-item="' + filename + '"><td></td><td></td>';
	row +=		'<td><span class="glyphicon glyphicon-asterisk"></span>' + filename + '</td>';
	row +=		'<td>' + T("loading") + '</td></tr>';

	$("#page_macros h1").addClass("hidden");
	$("#table_macro_files").removeClass("hidden");
	return $(row).appendTo("#table_macro_files");
}

var audioContext = new (window.AudioContext || window.webkitAudioContext);
function beep(frequency, duration) {
	var oscillator = audioContext.createOscillator();

	oscillator.type = 'sine';
	oscillator.frequency.value = frequency;
	oscillator.connect(audioContext.destination);
	oscillator.start();

	setTimeout(function() {
		oscillator.disconnect();
	}, duration);
}

function addHeadTemperature(temperature, type) {
	// Drop-Down item
	$(".ul-" + type + "-temp").append('<li><a href="#" class="heater-temp" data-temp="' + temperature + '">' + temperature + ' °C</a></li>');
	$(".btn-" + type + "-temp").removeClass("disabled");

	// Entry on Settings page
	var item =	'<li class="list-group-item col-xs-6 col-lg-3" data-temperature="' + temperature + '">' + temperature + ' °C';
	item +=		'<button class="btn btn-danger btn-sm btn-delete-parent pull-right" title="' + T("Delete this temperature item") + '">';
	item +=		'<span class="glyphicon glyphicon-trash"></span></button></li>';
	$("#ul_" + type + "_temps").append(item);
}

function changeTool(tool)
{
	if (tool >= 0) {
		if (getTool(tool).heaters.length > 0) {
			// Tool macros are likely to block the HTTP G-Code processor, so we first turn off
			// the appropriate heater, change the tool and then turn it back on.
			var firstToolHeater = getTool(tool).heaters;
			var gcode = "G10 P" + tool + " S-273\n" +
				"T" + tool + "\n" +
				"G10 P" + tool + " S" + $("#input_temp_h" + firstToolHeater + "_active").val();
			sendGCode(gcode);
		} else {
			sendGCode("T" + tool);
		}
	} else {
		sendGCode("T-1");
	}
}

function clearBedTemperatures() {
	$(".ul-bed-temp, #ul_bed_temps").html("");
	$(".btn-bed-temp").addClass("disabled");
}

function clearGCodeDirectory() {
	$("#ol_gcode_directory").children(":not(:last-child)").remove();
	$("#ol_gcode_directory").prepend('<li class="active"><span class="glyphicon glyphicon-folder-open"></span> ' + T("G-Codes Directory") + '</li>');
}

function clearGCodeFiles() {
	$("#table_gcode_files > tbody").remove();
	$("#table_gcode_files").addClass("hidden");
	$("#page_files h1").removeClass("hidden");
	if (isConnected) {
		$("#page_files h1").text(T("No Files or Directories found"));
	} else {
		$("#page_files h1").text(T("Connect to your Duet to display G-Code files"));
	}
}

function clearDefaultGCodes() {
	$(".ul-gcodes").html("");
	$("#table_gcodes > tbody").remove();
	$(".btn-gcodes").addClass("disabled");
}

function clearHeadTemperatures() {
	$(".ul-active-temp, .ul-standby-temp, #ul_active_temps, #ul_standby_temps").html("");
	$(".btn-active-temp, .btn-standby-temp").addClass("disabled");
}

function clearMacroDirectory() {
	$("#ol_macro_directory").children(":not(:last-child)").remove();
	$("#ol_macro_directory").prepend('<li class="active"><span class="glyphicon glyphicon-folder-open"></span> ' + T("Macros Directory") + '</li>');
}

function clearMacroFiles() {
	// Control Page
	if (currentMacroDirectory == "/macros") {
		$("#panel_macro_buttons .btn-group-vertical").html("");
		$("#panel_macro_buttons h4").removeClass("hidden");
	}

	// Macros Page
	$("#table_macro_files > tbody").remove();
	$("#table_macro_files").addClass("hidden");
	$("#page_macros h1").removeClass("hidden");
	if (isConnected) {
		$("#page_macros h1").text(T("No Macro Files found"));
	} else {
		$("#page_macros h1").text(T("Connect to your Duet to display Macro files"));
	}
}

function convertSeconds(value) {
	value = Math.round(value);
	if (value < 0) {
		value = 0;
	}

	var timeLeft = [], temp;
	if (value >= 3600) {
		temp = Math.floor(value / 3600);
		if (temp > 0) {
			timeLeft.push(temp + "h");
			value = value % 3600;
		}
	}
	if (value >= 60) {
		temp = Math.floor(value / 60);
		if (temp > 0) {
			timeLeft.push((temp > 9 ? temp : "0" + temp) + "m");
			value = value % 60;
		}
	}
	value = value.toFixed(0);
	timeLeft.push((value > 9 ? value : "0" + value) + "s");

	return timeLeft.reduce(function(a, b) { return a + " " + b; });
}

function formatSize(bytes) {
	if (settings.useKiB) {
		if (bytes > 1073741824) { // GiB
			return (bytes / 1073741824).toFixed(1) + " GiB";
		}
		if (bytes > 1048576) { // MiB
			return (bytes / 1048576).toFixed(1) + " MiB";
		}
		if (bytes > 1024) { // KiB
			return (bytes / 1024).toFixed(1) + " KiB";
		}
	} else {
		if (bytes > 1000000000) { // GB
			return (bytes / 1000000000).toFixed(1) + " GB";
		}
		if (bytes > 1000000) { // MB
			return (bytes / 1000000).toFixed(1) + " MB";
		}
		if (bytes > 1000) { // KB
			return (bytes / 1000).toFixed(1) + " KB";
		}
	}
	return bytes + " B";
}

function loadMacroDropdown(directory, dropdown) {
	$.ajax("rr_files?dir=" + encodeURIComponent(directory), {
		dataType: "json",
		directory: directory,
		dropdown: dropdown,
		success: function(response) {
			if (response.files.length == 0) {
				dropdown.html('<li><a href="#" class="disabled" style="color:#777;">' + T("No files found!") + '</a></li>');
			} else {
				dropdown.html('<li><a href="#" class="disabled" style="color:#777;">' + directory + '</a></li><li class="divider"></li>');
				var files = response.files.sort(function (a, b) {
					return a.toLowerCase().localeCompare(b.toLowerCase());
				});

				files.forEach(function(filename) {
					$.ajax("rr_fileinfo?name=" + encodeURIComponent(directory + "/" + filename), {
						dataType: "json",
						directory: directory,
						dropdown: dropdown,
						filename: filename,
						success: function(fileinfo) {
							var label = stripMacroFilename(filename);
							if (fileinfo.err == 0) {
								dropdown.append('<li><a href="#" class="macro-file-entry" data-macro="' + directory + '/' + filename + '">' + label + '</a></li>');
							} else {
								dropdown.append('<li><a href="#" class="macro-dir-entry" data-directory="' + directory + '/' + filename + '">' + label + ' <span class="caret"></span></a></li>');
							}

							$(".macro-file-entry").off("click").click(function(e) {
								sendGCode("M98 P" + $(this).data("macro"));
								e.preventDefault();
							});

							$(".macro-dir-entry").off("click").click(function(e) {
								var dropdown = $(this).parents("ul");
								loadMacroDropdown($(this).data("directory"), dropdown);
								e.stopPropagation();
								e.preventDefault();
							});
						}
					});
				});
			}
		}
	});
}

function log(style, message) {
	var entry =		'<div class="row alert-' + style + '">';
	entry +=		'<div class="col-xs-2 col-sm-2 col-md-2 col-lg-1 text-center"><strong>' + (new Date()).toLocaleTimeString() + '</strong></div>';
	entry +=		'<div class="col-xs-10 col-sm-10 col-md-10 col-lg-11">' + message + '</div></div>';
	$("#console_log").prepend(entry);
}

function recordHeaterTemperatures(bedTemp, chamberTemp, headTemps) {
	// Add bed temperature
	if (heatedBed) {
		recordedBedTemperatures.push(bedTemp);
	} else {
		recordedBedTemperatures = [];
	}
	if (recordedBedTemperatures.length > maxTemperatureSamples) {
		recordedBedTemperatures.shift();
	}

	// Add chamber temperature
	if (chamber) {
		recordedChamberTemperatures.push(chamberTemp);
	} else {
		recordedChamberTemperatures = [];
	}
	if (recordedChamberTemperatures.length > maxTemperatureSamples) {
		recordedChamberTemperatures.shift();
	}

	// Add heater temperatures
	for(var i=0; i<headTemps.length; i++) {
		recordedHeadTemperatures[i].push(headTemps[i]);
		if (recordedHeadTemperatures[i].length > maxTemperatureSamples) {
			recordedHeadTemperatures[i].shift();
		}
	}

	// Remove invalid data (in case the number of heads has changed)
	for(var i=headTemps.length; i<6; i++) {
		recordedHeadTemperatures[i] = [];
	}
}

function setAxesHomed(axes) {
	// Set button colors
	var unhomedAxes = "";
	if (axes[0]) {
		$(".btn-home-x").removeClass("btn-warning").addClass("btn-primary");
	} else {
		unhomedAxes = (geometry == "delta") ? ", A" : ", X";
		$(".btn-home-x").removeClass("btn-primary").addClass("btn-warning");
	}
	if (axes[1]) {
		$(".btn-home-y").removeClass("btn-warning").addClass("btn-primary");
	} else {
		unhomedAxes += (geometry == "delta") ? ", B" : ", Y";
		$(".btn-home-y").removeClass("btn-primary").addClass("btn-warning");
	}
	if (axes[2]) {
		$(".btn-home-z").removeClass("btn-warning").addClass("btn-primary");
	} else {
		unhomedAxes += (geometry == "delta") ? ", C" : ", Z";
		$(".btn-home-z").removeClass("btn-primary").addClass("btn-warning");
	}

	// Set home alert visibility
	if (unhomedAxes == "") {
		$(".home-warning").addClass("hidden");
	} else {
		if (unhomedAxes.length > 3) {
			$("#unhomed_warning").html(T("The following axes are not homed: <strong>{0}</strong>", unhomedAxes.substring(2)));
		} else {
			$("#unhomed_warning").html(T("The following axis is not homed: <strong>{0}</strong>", unhomedAxes.substring(2)));
		}
		$(".home-warning").removeClass("hidden");
	}
}

function setATXPower(value) {
	$("input[name='atxPower']").prop("checked", false).parent().removeClass("active");
	$("input[name='atxPower'][value='" + (value ? "1" : "0") + "']").prop("checked", true).parent().addClass("active");
}

function setCurrentTemperature(heater, temperature) {
	var field = "#tr_head_" + heater + " td";
	if (heater == "bed") {
		field = "#tr_bed td";
	} else if (heater == "chamber") {
		field = "#tr_chamber td";
	}

	if (temperature == undefined) {
		$(field).first().html("n/a");
	} else if (temperature < -270) {
		$(field).first().html("error");
	} else {
		$(field).first().html(temperature.toFixed(1) + " °C");
	}

	if (heater != "bed" && heater != "chamber" && lastStatusResponse != undefined) {
		if (lastStatusResponse.currentTool != -1) {
			var isActiveHead = false;
			getToolsByHeater(heater).forEach(function(tool) { if (tool == lastStatusResponse.currentTool) { isActiveHead = true; } });
			if (isActiveHead) {
				if (temperature >= coldRetractTemp) {
					$(".btn-retract").removeClass("disabled");
				} else {
					$(".btn-retract").addClass("disabled");
				}

				if (temperature >= coldExtrudeTemp) {
					$(".btn-extrude").removeClass("disabled");
				} else {
					$(".btn-extrude").addClass("disabled");
				}
			}
		} else {
			$(".btn-retract, .btn-extrude").addClass("disabled");
		}
	}
}

function setGeometry(g) {
	// The following may be extended in future versions
	if (g == "delta") {
		$("#btn_bed_compensation").text(T("Auto Delta Calibration"));
		$(".home-buttons div, div.home-buttons").addClass("hidden");
		$("td.home-buttons").css("padding-right", "0px");
		$("#btn_homeall").css("min-width", "");
	} else {
		$("#btn_bed_compensation").text(T("Auto Bed Compensation"));
		$(".home-buttons div, div.home-buttons").removeClass("hidden");
		$("td.home-buttons").css("padding-right", "");
		$("#btn_homeall").resize();
	}
	$("#dd_geometry").text(T(g.charAt(0).toUpperCase() + g.slice(1)));
	geometry = g;
}

function setGCodeFileItem(row, size, height, firstLayerHeight, layerHeight, filamentUsage, generatedBy) {
	row.data("file", row.data("item"));
	row.children().eq(0).html('<button class="btn btn-success btn-print-file btn-sm" title="' + T("Print this file (M32)") + '"><span class="glyphicon glyphicon-print"></span></button>').attr("colspan", "");
	row.children().eq(1).html('<button class="btn btn-danger btn-delete-file btn-sm" title="' + T("Delete this file") + '"><span class="glyphicon glyphicon-trash"></span></button>');

	var linkCell = row.children().eq(2);
	linkCell.find("span").removeClass("glyphicon-asterisk").addClass("glyphicon-file");
	linkCell.html('<a href="#" class="gcode-file">' + linkCell.html() + '</a>');
	linkCell.find("a")[0].addEventListener("dragstart", fileDragStart, false);
	linkCell.find("a")[0].addEventListener("dragend", fileDragEnd, false);

	row.children().eq(3).html(formatSize(size));

	if (height > 0) {
		row.children().eq(4).html(height + " mm");
	} else {
		row.children().eq(4).html("n/a");
	}

	if (layerHeight > 0) {
		if (firstLayerHeight == undefined) {
			row.children().eq(5).html(layerHeight + " mm");
		} else {
			row.children().eq(5).html(firstLayerHeight + " / " + layerHeight + " mm");
		}
	} else {
		row.children().eq(5).html("n/a");
	}

	if (filamentUsage.length > 0) {
		var totalUsage = filamentUsage.reduce(function(a, b) { return a + b; }).toFixed(1) + " mm";
		if (filamentUsage.length == 1) {
			row.children().eq(6).html(totalUsage);
		} else {
			var individualUsage = filamentUsage.reduce(function(a, b) { return a + " mm, " + b; }) + " mm";
			var filaUsage = "<abbr class=\"filament-usage\" title=\"" + individualUsage + "\">" + totalUsage + "</abbr>";
			row.children().eq(6).html(filaUsage);
		}
	} else {
		row.children().eq(6).html("n/a");
	}

	if (generatedBy != "") {
		row.children().eq(7).html(generatedBy);
	} else {
		row.children().eq(7).html("n/a");
	}
}

function setGCodeDirectoryItem(row) {
	row.data("directory", row.data("item"));
	row.children().eq(1).html('<button class="btn btn-danger btn-delete-gcode-directory btn-sm" title="' + T("Delete this directory") + '"><span class="glyphicon glyphicon-trash"></span></button>');

	var linkCell = row.children().eq(2);
	linkCell.find("span").removeClass("glyphicon-asterisk").addClass("glyphicon-folder-open");
	linkCell.html('<a href="#" class="gcode-directory">' + linkCell.html() + '</a>').prop("colspan", 6);
	linkCell.find("a")[0].addEventListener("dragstart", fileDragStart, false);
	linkCell.find("a")[0].addEventListener("dragend", fileDragEnd, false);

	for(var i=8; i>=3; i--) {
		row.children().eq(i).remove();
	}

	if (gcodeLastDirectory == undefined) {
		var firstRow = $("#table_gcode_files tbody > tr:first-child");
		if (firstRow != row) {
			row.insertBefore(firstRow);
		}
	} else {
		row.insertAfter(gcodeLastDirectory);
	}
	gcodeLastDirectory = row;
}

function setGCodeDirectory(directory) {
	currentGCodeDirectory = directory;

	$("#ol_gcode_directory").children(":not(:last-child)").remove();
	if (directory == "/gcodes") {
		$("#ol_gcode_directory").prepend('<li class="active"><span class="glyphicon glyphicon-folder-open"></span> ' + T("G-Codes Directory") + '</li>');
		$("#ol_gcode_directory li:last-child").html('<span class="glyphicon glyphicon-level-up"></span> ' + T("Go Up"));
	} else {
		var listContent = '<li><a href="#" data-directory="/gcodes"><span class="glyphicon glyphicon-folder-open"></span> ' + T("G-Codes Directory") + '</a></li>';
		var directoryItems = directory.split("/"), directoryPath = "/gcodes";
		for(var i=2; i<directoryItems.length -1; i++) {
			directoryPath += "/" + directoryItems[i];
			listContent += '<li class="active"><a href="#" data-directory="' + directoryPath + '">' + directoryItems[i] + '</a></li>';
		}
		listContent += '<li class="active">' + directoryItems[directoryItems.length -1] + '</li>';
		$("#ol_gcode_directory").prepend(listContent);
		$("#ol_gcode_directory li:last-child").html('<a href="#" data-directory="' + directoryPath + '"><span class="glyphicon glyphicon-level-up"></span> ' + T("Go Up") + '</a>');
	}
}

function setHeaterState(heater, status, currentTool) {
	var statusText = "n/a";
	switch (status) {
		case 0:
			statusText = T("off");
			break;

		case 1:
			statusText = T("standby");
			break;

		case 2:
			statusText = T("active");
			break;

		case 3:
			statusText = T("fault");
			break;
	}

	if (heater == "bed") {
		$("#tr_bed > th:first-child > span").text(statusText);
	} else if (heater == "chamber") {
		$("#tr_chamber > th:first-child > span").text(statusText);
	} else {
		var tools = getToolsByHeater(heater);
		if (tools.length != 0) {
			statusText += " (T" + tools.reduce(function(a, b) { return a + ", T" + b; }) + ")";
			statusText = statusText.replace("T" + currentTool, "<u>T" + currentTool + "</u>");
			if (tools.length > 1) {
				$("#table_heaters tr > th:first-child").eq(heater).find(".caret").removeClass("hidden");
			} else {
				$("#table_heaters tr > th:first-child").eq(heater).find(".caret").addClass("hidden");
			}
		} else {
			$("#table_heaters tr > th:first-child").eq(heater).find(".caret").addClass("hidden");
		}
		$("#table_heaters tr > th:first-child").eq(heater).children("span").html(statusText);
	}
}

function setMacroFileItem(row, size) {
	row.data("file", row.data("item"));
	row.children().eq(0).html('<button class="btn btn-success btn-run-macro btn-sm" title="' + T("Run this macro file (M98)") + '"><span class="glyphicon glyphicon-play"></span></button>');
	row.children().eq(1).html('<button class="btn btn-danger btn-delete-macro btn-sm" title="' + T("Delete this macro") + '"><span class="glyphicon glyphicon-trash"></span></button>');

	var linkCell = row.children().eq(2);
	linkCell.find("span").removeClass("glyphicon-asterisk").addClass("glyphicon-file");
	linkCell.html('<a href="#" class="btn-run-macro">' + linkCell.html() + '</a>');
	linkCell.find("a")[0].addEventListener("dragstart", fileDragStart, false);
	linkCell.find("a")[0].addEventListener("dragend", fileDragEnd, false);

	row.children().eq(3).html(formatSize(size));
}

function setMacroDirectoryItem(row) {
	if (currentMacroDirectory == "/macros") {
		var button = $("#panel_macro_buttons button[data-macro='" + currentMacroDirectory + "/" + row.data("item") + "']");
		button.html(button.html() + ' <span class="caret"></span>');
		button.data("directory", button.data("macro")).attr("data-toggle", "dropdown");
		button.removeData("macro");
		button.parent().append('<ul class="dropdown-menu"></ul>');
	}

	row.data("directory", row.data("item"));
	row.children().eq(1).html('<button class="btn btn-danger btn-delete-macro-directory btn-sm" title="' + T("Delete this directory") + '"><span class="glyphicon glyphicon-trash"></span></button>');

	var linkCell = row.children().eq(2);
	linkCell.find("span").removeClass("glyphicon-asterisk").addClass("glyphicon-folder-open");
	linkCell.html('<a href="#" class="macro-directory">' + linkCell.html() + '</a>').prop("colspan", 2);
	linkCell.find("a")[0].addEventListener("dragstart", fileDragStart, false);
	linkCell.find("a")[0].addEventListener("dragend", fileDragEnd, false);

	row.children().eq(3).remove();

	if (macroLastDirectory == undefined) {
		var firstRow = $("#table_macro_files tbody > tr:first-child");
		if (firstRow != row) {
			row.insertBefore(firstRow);
		}
	} else {
		row.insertAfter(macroLastDirectory);
	}
	macroLastDirectory = row;
}

function setMacroDirectory(directory) {
	currentMacroDirectory = directory;

	$("#ol_macro_directory").children(":not(:last-child)").remove();
	if (directory == "/macros") {
		$("#ol_macro_directory").prepend('<li class="active"><span class="glyphicon glyphicon-folder-open"></span> ' + T("Macros Directory") + '</li>');
		$("#ol_macro_directory li:last-child").html('<span class="glyphicon glyphicon-level-up"></span> ' + T("Go Up"));
	} else {
		var listContent = '<li><a href="#" data-directory="/macros"><span class="glyphicon glyphicon-folder-open"></span> ' + T("Macros Directory") + '</a></li>';
		var directoryItems = directory.split("/"), directoryPath = "/macros";
		for(var i=2; i<directoryItems.length -1; i++) {
			directoryPath += "/" + directoryItems[i];
			listContent += '<li class="active"><a href="#" data-directory="' + directoryPath + '">' + directoryItems[i] + '</a></li>';
		}
		listContent += '<li class="active">' + directoryItems[directoryItems.length -1] + '</li>';
		$("#ol_macro_directory").prepend(listContent);
		$("#ol_macro_directory li:last-child").html('<a href="#" data-directory="' + directoryPath + '"><span class="glyphicon glyphicon-level-up"></span> ' + T("Go Up") + '</a>');
	}
}

function setPauseStatus(paused) {
	if (paused == isPaused) {
		return;
	}

	if (paused) {
		$("#btn_cancel").removeClass("disabled").parents("div.btn-group").removeClass("hidden");
		$("#btn_pause").removeClass("btn-warning disabled").addClass("btn-success").text(T("Resume")).attr("title", T("Resume paused print (M24)"));
	} else {
		$("#btn_cancel").parents("div.btn-group").addClass("hidden");
		$("#btn_pause").removeClass("btn-success").addClass("btn-warning").text(T("Pause Print")).attr("title", T("Pause current print (M25)"));
		if (isPrinting) {
			$("#btn_pause").removeClass("disabled");
		}
	}
	isPaused = paused;
}

function setPrintStatus(printing) {
	if (printing == isPrinting) {
		return;
	}

	if (printing) {
		if (justConnected) {
			showPage("print");
		}

		layerData = [];
		currentLayerTime = maxLayerTime = lastLayerPrintDuration = 0;
		drawPrintChart();
		printHasFinished = false;

		$(".btn-upload").addClass("disabled");
		$("#page_general .btn-upload").removeClass("disabled");

		$("#btn_pause").removeClass("disabled");
		$(".row-progress").removeClass("hidden");
		$("#td_last_layertime").html("n/a");

		requestFileInfo();
	} else {
		$("#btn_pause").addClass("disabled");
		$(".btn-upload").removeClass("disabled");

		if (!isConnected || fileInfo == undefined) {
			$(".row-progress").addClass("hidden");
		}

		if (isConnected) {
			if ($("#auto_sleep").is(":checked")) {
				sendGCode("M1");
				$("#auto_sleep").prop("checked", false);
			}

			if (!printHasFinished && currentLayerTime > 0) {
				addLayerData(currentLayerTime, true);
				$("#td_layertime").html("n/a");
			}
			printHasFinished = true;

			if (fileInfo != undefined) {
				setProgress(100, T("Printed {0}, 100% Complete", fileInfo.fileName), undefined);
			} else {
				setProgress(100, T("Print Complete!"), undefined);
			}

			["filament", "layer", "file"].forEach(function(id) {
				if ($("#tl_" + id).html() != "n/a") {
					$("#tl_" + id).html("00s");
					$("#et_" + id).html((new Date()).toLocaleTimeString());
				}
			});
		}

		fileInfo = undefined;
	}

	isPrinting = printing;

	if (waitingForPrintStart && printing) {
		$("#modal_upload").modal("hide");
		showPage("print");
		waitingForPrintStart = false;
	}
}

function setProbeValue(value, secondaryValue) {
	if (value < 0) {
		$("#td_probe").html("n/a");
	} else if (secondaryValue == undefined) {
		$("#td_probe").html(value);
	} else {
		$("#td_probe").html(value + " (" + secondaryValue.reduce(function(a, b) { return a + b; }) + ")");
	}

	if (probeTriggerValue != undefined && value > probeTriggerValue && !isPrinting) {
		$("#td_probe").css("background-color", probeTriggerColor);
	} else if (probeSlowDownValue != undefined && value > probeSlowDownValue && !isPrinting) {
		$("#td_probe").css("background-color", probeSlowDownColor);
	} else {
		$("#td_probe").css("background-color", "");
	}
}

function setProgress(progress, labelLeft, labelRight) {
	if (labelLeft != undefined) {
		$("#span_progress_left").text(labelLeft);
	}
	if (labelRight != undefined) {
		$("#span_progress_right").text(labelRight);
	}
	$("#progress").css("width", progress + "%");
}

function setStatusLabel(text, style) {
	text = T(text);
	$(".label-status").removeClass("label-default label-danger label-info label-warning label-success").addClass("label-" + style).text(text);
}

function setTemperatureInput(head, value, active) {
	var tempInputId;

	if (head == "bed") {
		tempInputId = "#input_temp_bed";
	} else if (head == "chamber") {
		tempInputId = "#input_temp_chamber";
	} else {
		tempInputId = "#input_temp_h" + head + "_";
		tempInputId += (active) ? "active": "standby";
	}

	if (!$(tempInputId).is(":focus")) {
		$(tempInputId).val((value < 0) ? 0 : value);
	}
}

function setTimeLeft(field, value) {
	if (value == undefined || (value == 0 && isPrinting)) {
		$("#et_" + field + ", #tl_" + field).html("n/a");
	} else {
		// Estimate end time
		var now = new Date();
		var estEndTime = new Date(now.getTime() + value * 1000); // Date uses ms
		$("#et_" + field).html(estEndTime.toLocaleTimeString());

		// Estimate time left
		$("#tl_" + field).html(convertSeconds(value));
	}
}

function setTitle(title) {
	$(".machine-name").html(title);
	$("head > title").text(title);
}

function showPage(name) {
	$(".navitem, .page").removeClass("active");
	$(".navitem-" + name + ", #page_" + name).addClass("active");

	if (name != currentPage) {
		if (name == "control") {
			$("#slider_fan_control").slider("relayout");
			if (macroUpdateIndex != knownMacroFiles.length) {
				updateMacroFiles();
			}
		}

		if (name == "print") {
			$("#slider_speed").slider("relayout");
			$("#slider_fan_print").slider("relayout");
			for(var extr=1; extr<=6; extr++) {
				$("#slider_extr_" + extr).slider("relayout");
			}
			if (refreshPrintChart) {
				drawPrintChart();
			}
			waitingForPrintStart = false;
		}

		if (name == "console") {
			currentPage = "console";
			if (window.matchMedia('(min-width: 992px)').matches) {
				$("#page_console input").focus();
			}
		}

		if (name == "files") {
			if (gcodeUpdateIndex == knownGCodeFiles.length) {
				$(".span-refresh-files").removeClass("hidden");
			} else {
				updateGCodeFiles();
			}
		} else {
			$(".span-refresh-files").addClass("hidden");
		}

		if (name == "macros") {
			if (macroUpdateIndex == knownMacroFiles.length) {
				$(".span-refresh-macros").removeClass("hidden");
			} else {
				updateMacroFiles();
			}
		} else {
			$(".span-refresh-macros").addClass("hidden");
		}

		if (name == "settings" && isConnected) {
			getConfigResponse();
			if ($("#page_config").is(".active")) {
				getConfigFile();
			}
		}
	}

	// Scroll to top of the main content on small devices
	if (window.matchMedia('(max-width: 991px)').matches) {
		var offset = (name != "control") ? -9 : 0;
		$('html, body').animate({
			scrollTop: ($('#main_content').offset().top + offset)
		}, 500);
	}
	currentPage = name;
}

function stripMacroFilename(filename) {
	var match = filename.match(/(.*)\.\w+/);
	if (match == null) {
		label = filename;
	} else {
		label = match[1];
	}

	match = label.match(/\d+_(.*)/);
	if (match != null) {
		label = match[1];
	}

	return label;
}

/* Other */

function applyThemeColors(themeActive) {
	// Update temp chart colors and repaint it
	tempChartOptions.colors[0] = $("#tr_bed a").css("color");
	tempChartOptions.colors[1] = $("#tr_head_1 a").css("color");
	tempChartOptions.colors[2] = $("#tr_head_2 a").css("color");
	tempChartOptions.colors[3] = $("#tr_head_3 a").css("color");
	tempChartOptions.colors[4] = $("#tr_head_4 a").css("color");
	tempChartOptions.colors[5] = $("#tr_head_5 a").css("color");
	tempChartOptions.colors[6] = $("#tr_chamber th").css("color");
	if (tempChart != undefined) {
		tempChart.destroy();
		tempChart = undefined;
		drawTemperatureChart();
	}

	// Update layer times chart
	printChartOptions.colors[0] = getColorFromCSS("chart-print-line");
	if (printChart != undefined) {
		printChart.destroy();
		printChart = undefined;
		drawPrintChart();
	}

	if (themeActive) {
		$("#layer_tooltip").css("background-color", $("#panel_print_info").css("background-color"));
	} else {
		$("#layer_tooltip").css("background-color", "");
	}

	// Update Z-probe colors
	probeSlowDownColor = getColorFromCSS("probe-slow-down");
	probeTriggerColor = getColorFromCSS("probe-trigger");
}

function getColorFromCSS(classname)
{
	var ghostSpan = $('<span class="hidden ' + classname + '"></span>');
	ghostSpan.appendTo("body");
	var color = ghostSpan.css("color");
	ghostSpan.remove();
	return color;
}

// vim: ts=4:sw=4
