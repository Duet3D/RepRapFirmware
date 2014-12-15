/*! Reprap Ormerod Web Control | by Matt Burnett <matt@burny.co.uk>. | open license
 */
var ver = 1.04; //App version
var polling = false; 
var printing = false;
var paused = false;
var chart,chart2,ormerodIP,layerCount,currentLayer,objHeight,objTotalFilament,printStartTime,gFilename,ubuff,timerStart,storage,layerHeight,lastUpdatedTime;
var startingFilamentPos = [], currentFilamentPos = [], objUsedFilament = [];
var sFactor = 100, e1Factor = 100, e2Factor = 100;
var sSliderActive = false, e1SliderActive = false, e2SliderActive = false;
var maxUploadBuffer = 2000;
var messageSeqId = 0;
var currentTool = 0;

//Temp/Layer Chart settings
var maxDataPoints = 200;
var chartData = [[], [], []];
var maxLayerBars = 100;
var layerData = [];
var filamentData = [];
var bedColour = "#454BFF"; //blue
var head1Colour = "#FC2D2D"; //red
var head2Colour = "#00A000"; //green

var gFileData = "";
var gFileIndex = 0;
var expansionFactor = 1.33;
var macroGs = ['setbed.g'];
var chevLeft = "<span class='glyphicon glyphicon-chevron-left'></span>";
var chevRight = "<span class='glyphicon glyphicon-chevron-right'></span>";

var gcodeDir = "gcodes/";
var webDir = "www/";
var sysDir = "sys/";

jQuery.extend({
    askElle: function(reqType, code) {
        var result;
        var query = "";
        switch(reqType) {
			case 'upload_data':
                query = "?data="+code;		// 'code' has already been URI encoded
				break;
            case 'gcode':
                query = "?gcode="+encodeURIComponent(code);
                break;
 			case 'fileinfo':
				if (code == null) break;
			case 'upload_begin':
			case 'delete':
				query = "?name="+encodeURIComponent(code);
				break;
			case 'upload_end':
				query = "?size="+code;
				break;
        }
        var url = '//' + ormerodIP + '/rr_'+reqType+query;
        $.ajax(url, {async:false,dataType:"json",success:function(data){result = data;}});
        return result;
    }
});

$(document).ready(function() {
    storage=$.localStorage;
    getCookies();
    loadSettings();
    
    moveVals(['X','Y','Z']);

    ormerodIP = location.host;
    $('span#hostLocation').text(ormerodIP);

    if ($.support.fileDrop) {
        fileDrop();
    } else {
        alert('Your browser does not support file drag-n-drop :( \nYou have to Click and select a file instead');
    }

    //fill chart with dummy data
    for (var i = 0; i < maxDataPoints; i++) {
        chartData[0].push([i, 0]);
        chartData[1].push([i, 10]);
        chartData[2].push([i, 20]);
    }

    //chart line colours
    $('#bedTxt').css("color", bedColour);
    $('#head1Click').css("color", head1Colour);
    $('#head2Click').css("color", head2Colour);

    chart = $.plot("#tempchart", chartData, {
        series: {shadowSize: 0},
        colors: [bedColour, head1Colour, head2Colour],
        yaxis: {min: -20, max: 280},
        xaxis: {show: false},
        grid: {
            borderWidth: 0
        }
    });

    chart2 = $.plot("#layerChart", [{
            data: layerData,
            bars: {show: true}
        }], {
        series: {shadowSize: 0},
        xaxis: {minTickSize: 1, tickDecimals: 0, panRange: [0, null], zoomRange: [20, 50]},
        yaxis: {minTickSize: 1, min: 0, tickDecimals: 0, panRange: false},
        grid: {borderWidth: 0},
        pan: {interactive: true}
    });

    message('success', 'Page Load Complete');
    $('button#connect').removeClass('disabled');
    
    var htmVer = getHTMLver();
    $('p#htmVer').text(htmVer);
    $('p#jsVer').text(ver);
    if (htmVer < ver) {
        //pop message
        modalMessage("Update! v"+ver+" is Available",
			"The version of reprap.htm on your Duet SD card is "+getHTMLver()+", the latest version is "+ver
			+", to ensure compatibility and with the latest javascript code, new features, and correct functionality it is highly recommended that you upgrade. The newest reprap.htm can be found at <a href='https://github.com/dc42/OrmerodWebControl'>https://github.com/dc42/OrmerodWebControl</a>",
			true);
    }
    
});

$('#connect').on('click', function() {
    if (polling) {
        polling = false;
        updatePage();
    } else {
        polling = true;
		$.askElle("connect");
        updatePage();        
        listGFiles();
		
		// Get the machine name
		var resp = $.askElle("name");
		if (resp !== undefined && resp.hasOwnProperty('myName') && resp.myName.length != 0) {
			$('span#machineName').text(resp.myName);
		}
		
		// See if the machine is printing a file, if so get the details and switch to the print Status tab
		resp = $.askElle("fileinfo", null);
		if (resp != undefined && resp.hasOwnProperty('fileName')) {
			var temp = updateFileTableInfo(resp, 'slic3rMain');
			layerHeight = temp[0];
			var height = temp[1], filament = temp[2];
			$('span#gFileDisplay').html('<strong>Printing ' + resp.fileName + ' from Duet SD card</strong>');
			printStartTime = (new Date()).getTime() - resp.printDuration;
			$('span#elapsed').text(resp.printDuration.toHHMMSS());
			resetLayerData(height, filament, true);
			$('progress#printProgressBar').show();
			$('#tabs a:eq(1)').tab('show');
		}

		// Get the firmware version. The response will be captured automatically from the next status response.
        $.askElle("gcode", "M115");
        poll();
    }
});

//temp controls
$('div#bedActiveTemperature').on('click', 'a#bedActiveTempLink', function() {
    $('input#bedActiveTempInput').val($(this).text());
    $.askElle('gcode', "M140 S" + $(this).text());
});
$('div#head1ActiveTemperature').on('click', 'a#head1ActiveTempLink', function() {
    $('input#head1ActiveTempInput').val($(this).text());
    $.askElle('gcode', "G10 P1 S" + $(this).text());
});
$('div#head1StandbyTemperature').on('click', 'a#head1StandbyTempLink', function() {
    $('input#head1StandbyTempInput').val($(this).text());
    $.askElle('gcode', "G10 P1 R" + $(this).text());
});
$('div#head2ActiveTemperature').on('click', 'a#head2ActiveTempLink', function() {
    $('input#head2ActiveTempInput').val($(this).text());
	$.askElle('gcode', "G10 P2 S" + $(this).text());
});
$('div#head2StandbyTemperature').on('click', 'a#head2StandbyTempLink', function() {
    $('input#head2StandbyTempInput').val($(this).text());
    $.askElle('gcode', "G10 P2 R" + $(this).text());
});
$('input#bedActiveTempInput').keydown(function(event) {
    if (event.which === 13) {
        event.preventDefault();
        $.askElle('gcode', "M140 S" + $(this).val());
		$('input#bedActiveTempInput').blur();
    }
});
$('input#head1ActiveTempInput').keydown(function(event) {
    if (event.which === 13) {
        event.preventDefault();
        $.askElle('gcode', "G10 P1 S" + $(this).val());
		$('input#head1ActiveTempInput').blur();
    }
});
$('input#head1StandbyTempInput').keydown(function(event) {
    if (event.which === 13) {
        event.preventDefault();
        $.askElle('gcode', "G10 P1 R" + $(this).val());
		$('input#head1StandbyTempInput').blur()
    }
});
$('input#head2ActiveTempInput').keydown(function(event) {
    if (event.which === 13) {
        event.preventDefault();
        $.askElle('gcode', "G10 P2 S" + $(this).val());
		$('input#head2ActiveTempInput').blur();
    }
});
$('input#head2StandbyTempInput').keydown(function(event) {
    if (event.which === 13) {
        event.preventDefault();
        $.askElle('gcode', "G10 P2 R" + $(this).val());
		$('input#head2StandbyTempInput').blur();
    }
});
$('div#bedActiveTemperature ul').on('click', 'a#addBedActiveTemp', function() {
    addTemp($('input#bedActiveTempInput').val(), 'bed');
});
$('div#head1ActiveTemperature ul').on('click', 'a#addHead1ActiveTemp', function() {
    addTemp($('input#head1ActiveTempInput').val(), 'active');
});
$('div#head1StandbyTemperature ul').on('click', 'a#addHead1StandbyTemp', function() {
    addTemp($('input#head1StandbyTempInput').val(), 'standby');
});
$('div#head2ActiveTemperature ul').on('click', 'a#addHead2ActiveTemp', function() {
    addTemp($('input#head2ActiveTempInput').val(), 'active');
});
$('div#head2StandbyTemperature ul').on('click', 'a#addHead2StandbyTemp', function() {
    addTemp($('input#head2StandbyTempInput').val(), 'standby');
});
$('a#head1Click').on('click', function() {
	$.askElle('gcode', (currentTool == 1) ? "T0" : "T1");
});
$('a#head2Click').on('click', function() {
	$.askElle('gcode', (currentTool == 2) ? "T0" : "T2");
});

//feed controls
$('div#feed button#feed').on('click', function() {
    var amount = $(this).val();
    var dir = "";
    if ($('input[name="feeddir"]:checked').attr('id') == "reverse") {
        dir = "-";
    }
    var feedRate = " F" + $('input[name="speed"]:checked').val();
    var code = "M120\nM83\nG1 E" + dir + amount + feedRate + "\nM121";
    $.askElle('gcode', code);
});

//gcodes
$('div#sendG button#txtinput, div#sendG a').on('click', function() {    
    var code;
    if (this.nodeName === 'BUTTON') {
        code = $('input#gInput').val().toUpperCase();
    } else {
        code = $(this).text();
    }
    $.askElle('gcode', code); //send gcode
});
$('table#quicks').on('click', 'a', function() {
    var code;
    if (this.attributes.itemprop) {
        code = this.attributes.itemprop.value;
    } else {
        code = $(this).text();
    }
   $.askElle('gcode', code);
});
$('input#gInput').keydown(function(event) {
    if (event.which === 13) {
        event.preventDefault();
        $.askElle('gcode', $(this).val().toUpperCase());
    }
});

//move controls
$('table#moveHead').on('click', 'button', function() {
    var btnVal = $(this).attr('value');
    if (btnVal) {
        $.askElle('gcode', btnVal);
    } else {
        var value = $(this).text();

        var feedRate = " F4000";
        if (value.indexOf("Z") >= 0)
            feedRate = " F200";

        var movePreCode = "M120\nG91\nG1 ";
        var movePostCode = "\nM121";
        $.askElle('gcode', movePreCode + value + feedRate + movePostCode);
    }
});

//panic buttons
$('div#panicBtn button').on('click', function() {
    var btnVal = $(this).attr('value');
    switch (btnVal) {
        case "M112":
            //panic stop
            window.stop();
            paused = false;
            break;
        case "reset":
            //reset printing after pause
            printing = false;
            paused = false;
            btnVal = "T0";
            resetLayerData(0, 0, false);
			//no break
        case "M24":
            //resume
            paused = false;
            $('button#pause').removeClass('active').text('Pause').attr('value', 'M25');
            $('button#printing').text("Ready :)");
            $('button#reset').addClass('hidden');
            break;
        case "M25":
            //pause
            paused = true;
            $(this).addClass('active').text('Resume').attr('value', 'M24');
            $('button#printing').text("Paused");
            $('button#reset').removeClass('hidden');
            break;
    }
    $.askElle('gcode', btnVal);
});

//sliders
$('#sFactor').slider({orientation:'vertical', reversed:true, min:10, max:300, step:5, value:100, tooltip:'show'});
$('#e1Factor').slider({orientation:'vertical', reversed:true, min:80, max:120, step:1, value:100, tooltip:'show'});
$('#e2Factor').slider({orientation:'vertical', reversed:true, min:80, max:120, step:1, value:100, tooltip:'show'});

$("#sFactor").on('slide', function(slideEvt) {
	sSliderActive = true;
	sFactor = slideEvt.value;
	$("span#sPercent").text(sFactor);
});
$("#e1Factor").on('slide', function(slideEvt) {
	e1SliderActive = true;
	e1Factor = slideEvt.value;
	$("span#e1Percent").text(e1Factor);
});
$("#e2Factor").on('slide', function(slideEvt) {
	e2SliderActive = true;
	e2Factor = slideEvt.value;
	$("span#e2Percent").text(e2Factor);
});

$("#sFactor").on('slideStop', function(slideEvt) {
	sFactor = slideEvt.value;
	$("span#sPercent").text(sFactor);
	$.askElle('gcode', "M220 S"+sFactor);
	sSliderActive = false;
});
$("#e1Factor").on('slideStop', function(slideEvt) {
	e1Factor = slideEvt.value;
	$("span#e1Percent").text(e1Factor);
	$.askElle('gcode', "M221 D0 S"+e1Factor);
	e1SliderActive = false;
});
$("#e2Factor").on('slideStop', function(slideEvt) {
	e2Factor = slideEvt.value;
	$("span#e2Percent").text(e2Factor);
	$.askElle('gcode', "M221 D1 S"+e2Factor);
	e2SliderActive = false;
});

//g files
$("div#gFileList1, div#gFileList2, div#gFileList3")
.on('mouseover', 'span#fileDelete', function() {
    $(this).parent().parent().parent().parent().parent().addClass('file-red');
}).on('mouseout', 'span#fileDelete', function() {
    $(this).parent().parent().parent().parent().parent().removeClass('file-red');
}).on('mouseover', 'span#filePrint', function() {
    $(this).parent().parent().parent().parent().parent().addClass('file-green');
}).on('mouseout', 'span#filePrint', function() {
    $(this).parent().parent().parent().parent().parent().removeClass('file-green');
}).on('mouseover', 'span#fileInfo', function() {
    $(this).parent().parent().parent().parent().parent().addClass('file-blue');
}).on('mouseout', 'span#fileInfo', function() {
    $(this).parent().parent().parent().parent().parent().removeClass('file-blue');
}).on('click', 'span#fileDelete', function() {
    var filename = gcodeDir + $(this).parent().parent().text();
    var resp = $.askElle('delete', filename);
	if (resp.err == 0) {
		message('success', "G file [" + filename + "] deleted from the SD card");
	} else {
		modalMessage("Delete failed", "Failed to delete file " + filename, true);
	}
    listGFiles();
}).on('click', 'span#filePrint', function() {
    var filename = $(this).parent().parent().text();
	printSDfile(filename);
}).on('click', 'span#fileInfo', function() {
	showFileInfo(gcodeDir + $(this).parent().parent().text());
});
$("button#filereload").on('click', function() {
    $('span#ulTitle').text("File Upload Status");
    setProgress(0, "ul", 0,0);
    listGFiles();
});
$('#uploadPrintGfile').on('click', function(){
    $('input#uploadPrintGselect').click();
});
$('#uploadGfile').on('click', function(){
    $('input#uploadGselect').click();
});
$('#ulConfigG').on('click', function(){
    $('input#ulConfigGselect').click();
});
$('#ulReprapHTM').on('click', function(){
    $('input#ulReprapHTMselect').click();
});
$("input#uploadPrintGselect:file").change(function (e){
    var file = this.files[0];
    readFile(this.files[0], function(e) {
        handleFileDrop(e.target.result, file.name, 'uploadandprint');
    });
    $(this).val('');
});
$("input#uploadGselect:file").change(function (e){
    var file = this.files[0];
    readFile(this.files[0], function(e) {
        handleFileDrop(e.target.result, file.name, 'upload');
    });
    $(this).val('');
});
$("input#ulConfigGselect:file").change(function (e){
    var file = this.files[0];
    readFile(this.files[0], function(e) {
        handleFileDrop(e.target.result, file.name, 'config');
    });
    $(this).val('');
});
$("input#ulReprapHTMselect:file").change(function (e){
    var file = this.files[0];
    readFile(this.files[0], function(e) {
        handleFileDrop(e.target.result, file.name, 'htm');
    });
    $(this).val('');
});

//Settings/cookie buttons
$("div#settings button#saveSettings").on('click', function(){
    saveSettings();
});
$("div#settings button#delSettings").on('click', function(){
    delSettings();
});

$('a[data-toggle="tab"]').on('show.bs.tab', function(e) {
    if (e.target.hash == "#settings") {
        $.askElle("gcode", "M503"); //get config.g on setting view
    }
});

//Messages 
$("div#messages button#clearLog").on('click', function(){
    message('clear', '');
});

//In the following, 'whichTemp' should be 'active', 'standby' or 'bed'
function addTemp(tempVal, whichTemp) {
    if (tempVal != "") {
        var temps = storage.get('temps', whichTemp);
		var newTemp = parseInt(tempVal);
		if (temps.indexOf(newTemp) < 0) {
			temps.push(newTemp);
			temps.sort(function(a, b){return b-a});
			storage.set('temps.' + whichTemp, temps);
			loadSettings();
		}
    }else{
        modalMessage("Error Adding Head Temp!", "You must enter a Temperature to add it to the dropdown list", close);
    }
}

function getCookies() {
    //if none use defaults here, probably move them elsewhere at some point!
    if (!storage.get('settings')) {
        storage.set('settings', { pollDelay : 1000, layerHeight : 0.24, halfz : 0, noOK : 0 });
    }
    if (!storage.get('temps') || !storage.get('temps.active')) {
        storage.set('temps', {'bed' : [120,65,0], 'active' : [240,185,0], 'standby' : [190,150,0] });
    }
}

function loadSettings() {
    $('div#settings input#pollDelay').val(storage.get('settings', 'pollDelay').toString());
    $('div#settings input#layerHeight').val(storage.get('settings', 'layerHeight').toString().toString())
    storage.get('settings', 'halfz')==1?$('div#settings input#halfz').prop('checked', true):$('div#settings input#halfz').prop('checked', false);
    storage.get('settings', 'noOK')==1?$('div#messages input#noOK').prop('checked', true):$('div#messages input#noOK').prop('checked', false);
    
    $('div#bedActiveTemperature ul').html('<li class="divider"></li><li><a href="#" id="addBedActiveTemp">Add Temp</a></li>');
    $('div#head1ActiveTemperature ul').html('<li class="divider"></li><li><a href="#" id="addHead1ActiveTemp">Add Temp</a></li>');
    $('div#head1StandbyTemperature ul').html('<li class="divider"></li><li><a href="#" id="addHead1StandbyTemp">Add Temp</a></li>');
    $('div#head2ActiveTemperature ul').html('<li class="divider"></li><li><a href="#" id="addHead2ActiveTemp">Add Temp</a></li>');
    $('div#head2StandbyTemperature ul').html('<li class="divider"></li><li><a href="#" id="addHead2StandbyTemp">Add Temp</a></li>');
    storage.get('temps', 'bed').forEach(function(item){
        $('div#bedActiveTemperature ul').prepend('<li><a href="#" id="bedActiveTempLink">'+item+'</a></li>');
    });
    storage.get('temps', 'active').forEach(function(item){
        $('div#head1ActiveTemperature ul').prepend('<li><a href="#" id="head1ActiveTempLink">'+item+'</a></li>');
        $('div#head2ActiveTemperature ul').prepend('<li><a href="#" id="head2ActiveTempLink">'+item+'</a></li>');
    });
    storage.get('temps', 'standby').forEach(function(item){
        $('div#head1StandbyTemperature ul').prepend('<li><a href="#" id="head1StandbyTempLink">'+item+'</a></li>');
        $('div#head2StandbyTemperature ul').prepend('<li><a href="#" id="head2StandbyTempLink">'+item+'</a></li>');
    });
}

function delSettings() {
    storage.removeAll();
    getCookies();
    loadSettings();
}

function saveSettings() {
    var zwas = storage.get('settings', 'halfz');
    storage.set('settings.pollDelay', parseInt($('div#settings input#pollDelay').val()));
    storage.set('settings.layerHeight', parseFloat($('div#settings input#layerHeight').val()));
    $('div#settings input#halfz').is(':checked')?storage.set('settings.halfz','1'):storage.set('settings.halfz','0');  
    $('div#settings input#noOK').is(':checked')?storage.set('settings.noOk','1'):storage.set('settings.noOK','0');
    if (zwas !== storage.get('settings', 'halfz')) {
        $('div#Zminus, div#Zplus').text('');
        moveVals(['Z']);
    } 
}

function moveVals(axis) {
    axis.forEach(function(value) {
        storage.get('settings','halfz')==1&&value=='Z'?i=50:i=100;
        var button = 0;
        for (i; i >= 0.05; i=i/10) {
            $('div#'+value+'minus').append('<button type="button" class="btn btn-default disabled">'+chevLeft+value+'-'+i.toString()+'</button>');
            $('div#'+value+'plus').prepend('<button type="button" class="btn btn-default disabled">'+value+'+'+i.toString()+chevRight+'</button>');
            button++;
        }
    });

}

function isNumber(n) {
    return !isNaN(parseFloat(n)) && isFinite(n);
}

function fileDrop() {
    $('#uploadGfile').fileDrop({
        decodeBase64: true,
        removeDataUriScheme: true,
        overClass: 'btn-success',
        onFileRead: function(file) {
                handleFileDrop(file[0].data, file[0].name, "upload");
        }
    });

    $('#uploadPrintGfile').fileDrop({
        decodeBase64: true,
        removeDataUriScheme: true,
        overClass: 'btn-success',
        onFileRead: function(file) {
                handleFileDrop(file[0].data, file[0].name, "uploadandprint");
        }
    });

    $('#ulConfigG').fileDrop({
        decodeBase64: true,
        removeDataUriScheme: true,
        overClass: 'btn-success',
        onFileRead: function(file) {
                handleFileDrop(file[0].data, file[0].name, "config");
        }
    });
    
    $('#ulReprapHTM').fileDrop({
        decodeBase64: true,
        removeDataUriScheme: true,
        overClass: 'btn-success',
        onFileRead: function(file) {
                handleFileDrop(file[0].data, file[0].name, "htm");
        }
    });
}

function getFileLength() {
	var kbytes = gFileData.length/1024;
	return (kbytes < 1000) ? kbytes.toFixed(0) + "Kb" : (kbytes/1024).toFixed(2) + "Mb";
}

function handleFileDrop(data, fName, action) {
    var ext = getFileExt(fName).toLowerCase();
	gFileData = data;
	gFileIndex = 0;
	switch (action) {
		case "config":
			uploadFile(action, fName, sysDir + fName, "");
			break;       
		case "htm":
			switch(ext) {
				case "js":
					uploadFile(action, fName, webDir + "js/" + fName, "");
					break;
				case "css":
					uploadFile(action, fName, webDir + "css/" + fName, "");
					break;
				case "eot":
				case "svg":
				case "ttf":
				case "woff":
					uploadFile(action, fName, webDir + "fonts/" + fName, "");
					break;
				case "png":
					uploadFile(action, fName, webDir + "img/" + fName, "");
					break;
				default:
					uploadFile(action, fName, webDir + fName, "");
					break;
			}					
			break;              
		case "upload":
			uploadFile(action, fName, gcodeDir + fName, "");
			break;
		case "uploadandprint":
			var tempFilename = "tempWebPrint.gcode";
			uploadFile("upload", fName, gcodeDir + tempFilename, tempFilename);
			break;
	}
}

function uploadFile(action, fromFile, toFile, printAfterUpload)
{
	timer();
	var resp = $.askElle('upload_begin', toFile);
	if (resp.err == 0) {
		ubuff = resp.ubuff;
		if (fromFile == toFile)
		{
			message("info", "File Upload of " + fromFile + " started");
		}
		else
		{
			message("info", "File Upload of " + fromFile + " to " + toFile + " started");
		}
		gFilename = fromFile;
		uploadModal();
		expansionFactor = 1.33;		// this is about right for gcode files
		uploadLoop(action, printAfterUpload);
	} else {
		uploadCantStart(toFile);
	}
}

// Update a table with file info and return [layerHeight, height, filament]
function updateFileInfo(fileName, id) {
	var info = $.askElle('fileinfo', fileName);
	return updateFileTableInfo(info, id);
}

// Ditto where we already have the file info
function updateFileTableInfo(info, id)
{
	clearSlic3rSettings(id);
	var height = 0, filament = 0, locLayerHeight = 0;
	if (info.hasOwnProperty('height') && isNumber(info.height))
	{
		height = info.height;
		if (id == 'slic3rModal') {
			addSlic3rSetting(id, "Object height", info.height + 'mm');
		}
	}
	if (info.hasOwnProperty('filament'))
	{
		if ($.isArray(info.filament)) {
			if (info.filament.length != 0) {
				// Duet firmware 0.78d-dc42 and later return an array of filament lengths needed. For now we just total them.
				var fils = "";
				for(var i = 0; i < info.filament.length; ++i) {
					filament += info.filament[i];
					if (i != 0) {
						fils += ' + ';
					}
					fils += info.filament[i];
				}
				addSlic3rSetting(id, "Filament Needed", fils + "mm");
			}
		}
		else if (isNumber(info.filament) && info.filament != 0) {
			filament = info.filament;
			addSlic3rSetting(id, "Filament Needed", filament + "mm");
		}
	}
	if (info.hasOwnProperty('layerHeight') && isNumber(info.layerHeight) && info.layerHeight != 0)
	{
		locLayerHeight = info.layerHeight;
		addSlic3rSetting(id, "Layer height", info.layerHeight + 'mm');
	}
	if (info.hasOwnProperty('generatedBy') && info.generatedBy.length != 0)
	{
		addSlic3rSetting(id, "Generated by", info.generatedBy);
	}
	if (info.hasOwnProperty('size') && isNumber(info.size)) {
		var sizeText = (info.size >= 1048576) ? (info.size/1048576).toFixed(3) + 'Mb' : (info.size >= 1024) ? (info.size/1024).toFixed(3) + 'Kb' : info.size + 'b';
		addSlic3rSetting(id, "File size", sizeText);
	}
	return [locLayerHeight, height, filament];
}

function printSDfile(fName)
{
	var temp = updateFileInfo(gcodeDir + fName, 'slic3rMain');
	layerHeight = temp[0];
	var height = temp[1], filament = temp[2];
	
	$.askElle('gcode', "M23 " + fName + "\nM24");
	message('success', "File [" + fName + "] sent to print");
	$('span#gFileDisplay').html('<strong>Printing ' + fName + ' from Duet SD card</strong>');
	resetLayerData(height, filament, false);
	$('progress#printProgressBar').show();
	$('#tabs a:eq(1)').tab('show');
}

function uploadModal() {
    modalMessage('File Upload Status',
		"<span id='ulTitle'>Uploading " + gFilename + " (" + getFileLength() + ")</span> <span id='ulProgressText' class='pull-right'></span>"+
		"<progress id='ulProgressBar' style='width:100%' max='100' value='0'></progress>",
		false);
}

function uploadCantStart(toFile)
{
	modalMessage("File upload failed", "Upload failed because file " + toFile + " could not be created.", true);
	message("info", "Failed to create file " + toFile);
}

function readFile(file, onLoadCallback){
    //read file from click-choose type printing/upload
    var reader = new FileReader();
    reader.onload = onLoadCallback;
    reader.readAsText(file);
}

function clearSlic3rSettings(id) {
    $('table#' + id + ' tbody').html(''); //slic3r setting <table>
}

function addSlic3rSetting(id, key, data) {
	$('table#' + id + ' tbody').append('<tr><th>'+key+'</th></tr><tr><td>'+data+'</td></tr>');
}

function uploadLoop(action, fileToPrint) { //Web Printing/Uploading
    if (gFileIndex == gFileData.length) {
		//Finished with Dropped file, stop loop, end tasks
		var resp = $.askElle('upload_end', gFileData.length);
		if (resp.err != 0) {
			$('span#ulTitle').text(gFilename + " Upload Failed!");
			$('span#ulProgressText').text("ERROR!");
			$('div#modal button#modalClose').removeClass('hidden');
			message("info", gFilename + " Upload Failed!");     
		}
		else {
			var duration = (timer() - timerStart).toHHMMSS();
			switch (action) {
				case "upload":
					listGFiles();
					if (fileToPrint != "")
					{
						$('div#modal').modal('hide');
						printSDfile(fileToPrint);
					}
					else
					{
						$('span#ulTitle').text(gFilename + " uploaded " + getFileLength() + " in " + duration);
						$('div#modal button#modalClose').removeClass('hidden');
						message("info", gFilename + " Upload Complete in " + duration);     
					}
					break;
				case "config":
					$.askElle("gcode", "M503"); //update config.g on setting view
					// no break
				case "htm":    
					$('span#ulTitle').text(gFilename + " uploaded " + getFileLength() + " in " + duration);
					$('div#modal button#modalClose').removeClass('hidden');
					message("info", gFilename + " Upload Complete in "+ duration);
					break;
			}
		}
		gFileData = "";
	}
	else {
		var wait = 20;
		var progress = Math.floor((100 * gFileIndex) / gFileData.length);
		var lastProgress = progress;
		if (paused == true) {
			wait = 2000;
		} else {
			// Send a number of packets in quick succession while there is plenty of buffer space, to speed up the file upload.
			// If we send too many then the user interface doesn't update often enough.
			do {
				webSend(action);
				progress = Math.floor((100 * gFileIndex) / gFileData.length);
			} while (ubuff >= 600 && gFileIndex < gFileData.length && progress == lastProgress);
			if (ubuff >= 200) {
				wait = 1;
			}
		}	
		setProgress(progress, "ul", 0, 0);
		setTimeout(function() {
			uploadLoop(action, fileToPrint);
		}, wait);
    }
}

function webSend(action) { //Web Printing/Uploading
	if (ubuff > maxUploadBuffer) {
        ubuff = maxUploadBuffer;
	}
    if (gFileIndex < gFileData.length) {
		var line = "";
		var startIndex = gFileIndex;
		while(gFileIndex < gFileData.length && line.length + 50 <= ubuff) {	// keep going until less than 50 bytes free space left
			var chunkSize = Math.min(Math.floor((ubuff - line.length)/(expansionFactor * 1.1)), gFileData.length - gFileIndex);
			var chunk = encodeURIComponent(gFileData.substring(gFileIndex, gFileIndex + chunkSize));
			if (line.length + chunk.length <= ubuff) {
				line += chunk;
				gFileIndex += chunkSize;
			} else {
				expansionFactor = chunk.length/chunkSize;
			}
        }
		
        var resp = $.askElle('upload_data', line); //send chunk of gcodes or html, and get buffer response
		expansionFactor = (0.875 * expansionFactor) + (0.125 * (line.length/(gFileIndex - startIndex)));
        
        if (typeof resp != 'undefined') {
            ubuff = resp.ubuff;
        } else {
            ubuff = 0;
        }
    }
}

function listGFiles() {
    var count = 0;
    var filesPerCol;
    var list = "gFileList1";
    $('div#gFileList1, div#gFileList2, div#gFileList3').html("");
    var result = $.askElle("files", "");
	result.files.sort(function (a, b) {
		return a.toLowerCase().localeCompare(b.toLowerCase());
	});
    result.files.length>18?filesPerCol=Math.ceil(result.files.length/3):filesPerCol = 6;
    result.files.forEach(function(item) {
        count++;
        switch (true) {
            case (count > (filesPerCol * 2)):
                list = "gFileList3";
                break;
            case (count > filesPerCol):
                list = "gFileList2";
                break;
        }
        if(jQuery.inArray(item, macroGs) >= 0) {
            if (!$('table#quicks a[itemprop="M23 '+item+'\nM24"]').text()) {
                $('table#quicks td:eq(0)').append('<a href="#" role="button" class="btn btn-default disabled" itemprop="M23 '+item+'\nM24" id="quickgfile">'+item+'</a>');
            }
        }
        $('div#' + list).append('<div id="gFileLink" class="file-button"><table style="width:100%"><tbody><tr><td style="width:70%;word-break:break-all"><span id="fileName">'
			+ item + '</span></td>'
			+ '<td style="width:20px"><span id="fileInfo" class="glyphicon glyphicon-info-sign pull-right"></span></td>'
			+ '<td style="width:20px"><span id="filePrint" class="glyphicon glyphicon-print pull-right"></span></td>'
			+ '<td style="width:20px"><span id="fileDelete" class="glyphicon glyphicon-trash pull-right"></span></td>'
			+ '</tr></tbody></table></div>');
    });
}

function getFileExt(filename) {
    return filename.split('.').pop();
}

function getFileName(filename) {
    return filename.split('.').shift();
}

function disableButtons(which) {
    switch (which) {
        case "head":
            $('table#moveHead button, table#extruder button, table#extruder label, table#quicks a, button#uploadGfile, button#ulConfigG, button#ulReprapHTM, button#uploadPrintGfile').addClass('disabled');
            break;
		case "temp":
           $('table#temp button').addClass('disabled');
            break;
        case "panic":
            $('div#panicBtn button').addClass('disabled');
            $('button#reset').addClass('hidden');
            break;
        case "gfilelist":
            $('div#gcodefiles button').addClass('disabled');
            break;
        case "sendG":
            $('div#sendG button, div#sendG a, input#gInput').addClass('disabled');
            break;
    }
}

function enableButtons(which) {
    switch (which) {
        case "head":
            $('table#moveHead button, table#extruder button, table#extruder label, table#quicks a, button#uploadGfile, button#ulConfigG, button#ulReprapHTM, button#uploadPrintGfile').removeClass('disabled');
            break;
        case "temp":
            $('table#temp button').removeClass('disabled');
            break;
        case "panic":
            $('div#panicBtn button').removeClass('disabled');
            break;
        case "gfilelist":
            $('div#gcodefiles button').removeClass('disabled');
            break;
        case "sendG":
            $('div#sendG button, div#sendG a, input#gInput').removeClass('disabled');
            break;            
    }
}

function modalMessage(title, text, close) {
    $('div#modal h4.modal-title').text(title);
    $('div#modal div.modal-body').html(text);
    close?$('div#modal button#modalClose').removeClass('hidden'):$('div#modal button#modalClose').addClass('hidden');
    $('div#modal').modal({show:true});
}

function showFileInfo(fileName) {
    $('div#modalFileInfo h4.modal-title').text("File information for " + fileName);
	updateFileInfo(fileName, 'slic3rModal');
    $('div#modalFileInfo').modal({show:true});
}

function message(type, text) {
    var d = new Date();
    var time = zeroPrefix(d.getHours()) + ":" + zeroPrefix(d.getMinutes()) + ":" + zeroPrefix(d.getSeconds());
    if (type == 'clear') {
        $('div#messageText').html(time + " <span class='alert-info'>Log Cleared</span><br />");
    } else {
        $('div#messageText').prepend(time + " <span class='alert-" + type + "'>" + text + "</span><br />");
    }
}

function parseResponse(res) {
    switch (true) {
        case res.indexOf('Debugging enabled') >= 0:
            message('info', '<strong>M111</strong><br />' + res.replace(/\n/g, "<br />"));    
            break;
        case res.indexOf('Firmware') >= 0:
            var strt = res.indexOf("SION:") +5 ;
            var end = res.indexOf(" ELEC");
            $('p#firmVer').text(res.substr(strt, end - strt));
            message('info', '<strong>M115</strong><br />' + res.replace(/\n/g, "<br />"));
            $.askElle("gcode", "M105");
            break;
        case res.indexOf('M550') >= 0:
            message('info', '<strong>M503</strong><br />' + res.replace(/\n/g, "<br />")); 
            $('div#config').html("<span class='col-md-9'><br/><strong>Config.g File Contents:</strong></span>");
            res.split(/\n/g).forEach(function(item) {
                $('div#config').append("<span class='alert-info col-md-9'>" + item + "</span><br />");
            });
            $.askElle("gcode", "M105");
            break;
        case res == "ok":
            if ($('div#messages input#noOK').is(':checked')) {
                message('info', res);
            }
            break;
        default:
            message('info', res);
            break;
    }
}

function homedWarning(x,y,z) {
    if ((x+y+z) < 3) {
        $('span#warning').text('*some axes are not homed');
    } else {    
        $('span#warning').text('');
    }
    x===0?$('button#homeX').removeClass('btn-primary').addClass('btn-warning'):$('button#homeX').removeClass('btn-warning').addClass('btn-primary');
    y===0?$('button#homeY').removeClass('btn-primary').addClass('btn-warning'):$('button#homeY').removeClass('btn-warning').addClass('btn-primary');
    z===0?$('button#homeZ').removeClass('btn-primary').addClass('btn-warning'):$('button#homeZ').removeClass('btn-warning').addClass('btn-primary');
}

function updatePage() {
    var status = $.askElle("status", "");
    if (!status || !polling) {
        $('button#connect').removeClass('btn-success').addClass('btn-danger');
        $('button#printing').removeClass('btn-warning').removeClass('btn-success').addClass('btn-danger').text("Disconnected");
        if (polling) {
            message('danger', "<strong>Warning!</strong> Ormerod webserver is probably broken, power cycle/reset your Duet Board :(");
            $('button#connect').text("Retrying");
        } else {
            message('info', "<strong>Disconnected</strong> Page not being updated");
            $('button#connect').text("Connect");
        }
        $('span[id$="Temp"], span[id$="pos"]').text("0");
		$('span[id$="State"]').text(getState(-1));
		$('td#head1, td#head2').css('background-color', '#FFFFFF');
        disableButtons("head");
		disableButtons("temp");
        disableButtons("panic");
    } else {
        $('button#connect').removeClass('btn-danger').addClass('btn-success').text("Online");
        //Connected Hoorahhh!
        if (messageSeqId !== status.seq) {
            messageSeqId = status.seq;
            parseResponse(status.resp);
        }
		currentFilamentPos = status.extr;
        homedWarning(status.homed[0],status.homed[1],status.homed[2]);
		currentTool = status.tool;
		if (currentTool == 1) {
			$('td#head1').css('background-color', '#E0E0E0');
		} else {
			$('td#head1').css('background-color', '#FFFFFF');
		}
		if (currentTool == 2) {
			$('td#head2').css('background-color', '#E0E0E0');
		} else {
			$('td#head2').css('background-color', '#FFFFFF');
		}
		if (status.hasOwnProperty('fanRPM')) {
			$('#fanRPM').text(status.fanRPM.toString());
		}
		if (status.status == "S") {
			//stopped
			printing = false;
            $('button#printing').removeClass('btn-danger').removeClass('btn-success').addClass('btn-warning').text("Halted");
            disableButtons('panic');
            disableButtons("head");
			disableButtons("temp");
            disableButtons("gfilelist");
		} else if (status.status === "P") {
            //printing
            printing = true;
            objHeight = $('input#objheight').val();
			if (!isNumber(objHeight) && status.status == 'P' && status.hasOwnProperty('height')) {
				objHeight = status.height;
				$('input#objheight').val(objHeight.toString());
			}
            $('button#printing').removeClass('btn-danger').removeClass('btn-warning').addClass('btn-success').text("Active");
            enableButtons('panic');
			enableButtons('temp');
            disableButtons("print");
            disableButtons("gfilelist");
            currentLayer = whichLayer(status.pos[2]);
			if (isNumber(objHeight)) {
                if (!layerHeight) layerHeight = storage.get('settings','layerHeight');
                layerCount = Math.ceil(objHeight / layerHeight);
			}
			if (printStartTime && objTotalFilament > 0) {
				setProgress(Math.floor((objUsedFilament.reduce(function(a, b){ return a + b; }, 0) / objTotalFilament) * 100), 'print', currentLayer, layerCount);
				estEndTime();
            } else if (printStartTime && layerCount > 0) {
                setProgress(Math.floor((currentLayer / layerCount) * 100), 'print', currentLayer, layerCount);
            } else {
                setProgress(0, 'print', 0, 0);
            }
            layers(currentLayer);
        } else if (status.status === "I" && !paused ) {
            //inactive, not printing
            printing = false;
            $('button#printing').removeClass('btn-danger').removeClass('btn-success').addClass('btn-warning').text("Ready :)");
            disableButtons("panic");
            enableButtons('head');
			enableButtons('temp');
            enableButtons("gfilelist");
        } else if (status.status === "I" && paused) {
            //paused
            printing = true;
            $('button#printing').removeClass('btn-danger').removeClass('btn-success').addClass('btn-warning').text("Paused");
            enableButtons('panic');
            enableButtons('head');
			enableButtons('temp');
        } else {
            //unknown state
            printing = paused = false;
            $('button#printing').removeClass('btn-warning').removeClass('btn-success').addClass('btn-danger').text("Error!");
            message('danger', 'Unknown Poll State : ' + status.status);
        }

		// Update the current and selected active/standby temperatures
        $('span#bedCurrentTemp').text(status.heaters[0].toFixed(1) + "°C");
		if (!$('input#bedActiveTempInput').is(":focus")) {
			$('input#bedActiveTempInput').val(status.active[0].toFixed(0));
		}
        $('span#head1CurrentTemp').text(status.heaters[1].toFixed(1) + "°C");
		if (!$('input#head1ActiveTempInput').is(":focus")) {
			$('input#head1ActiveTempInput').val(status.active[1].toFixed(0));
		}
		if (!$('input#head1StandbyTempInput').is(":focus")) {
			$('input#head1StandbyTempInput').val(status.standby[1].toFixed(0));
		}
		$('span#head2CurrentTemp').text((status.heaters.length >= 3) ? status.heaters[2].toFixed(1) + "°C" : "n/a");
		if (!$('input#head2ActiveTempInput').is(":focus")) {
			$('input#head2ActiveTempInput').val((status.active.length >= 3) ? status.active[2].toFixed(0) : "n/a");
		}
		if (!$('input#head2StandbyTempInput').is(":focus")) {
			$('input#head2StandbyTempInput').val((status.standby.length >= 3) ? status.standby[2].toFixed(0) : "n/a");
		}
		
		// Update the heater statuses
		$('span#head1State').text(getState(status.hstat[1]));
		$('span#head2State').text(getState(status.hstat[2]));
		$('span#bedState').text(getState(status.hstat[0]));
		
		// Update the head position and zprobe
        $('span#Xpos').text(status.pos[0].toFixed(2));
        $('span#Ypos').text(status.pos[1].toFixed(2));
        $('span#Zpos').text(status.pos[2].toFixed(2));
		var ePosText = "";
		for(var i=0; i < status.extr.length; ++i) {
			if (i != 0) {
				ePosText += ", ";
			}
			ePosText += status.extr[i].toFixed(1);
		}
        $('span#Epos').text(ePosText);
        $('span#probe').text(status.probe);

        //Temp chart stuff
        chartData[0].push(parseFloat(status.heaters[0]));
        chartData[1].push(parseFloat(status.heaters[1]));
        chartData[2].push((status.heaters.length >= 3) ? parseFloat(status.heaters[2]) : 0);
        chart.setData(parseChartData());
        chart.draw();
		
		// Update the slider positions
		if (!sSliderActive) {
			sFactor = status.sfactor;
			$('#sFactor').slider('setValue', sFactor);
			$("span#sPercent").text(sFactor);
		}
		if (!e1SliderActive) {
			e1Factor = status.efactor[0];
			$('#e1Factor').slider('setValue', e1Factor);
			$("span#e1Percent").text(e1Factor);
		}
		if (status.efactor.length >= 2 && !e2SliderActive) {
			e2Factor = status.efactor[1];
			$('#e2Factor').slider('setValue', e2Factor);
			$("span#e2Percent").text(e2Factor);
		}
    }
}

function getState(state) {
	switch(state) {
	case 0: return 'off';
	case 1: return 'standby';
	case 2: return 'active';
	case 3: return 'fault';
	default: return '';
	}
}

function getFilamentUsed() {
	for(var i = 0; i < currentFilamentPos.length && i < startingFilamentPos.length; ++i) {
		if (currentFilamentPos[i] - startingFilamentPos[i] < objUsedFilament[i] - 12) {
			//gone backwards by more than 12mm so probably just done a G30 E0 to reset the filament origin
			startingFilamentPos[i] = currentFilamentPos[i] - objUsedFilament[i];
		}
		else {
			objUsedFilament[i] = currentFilamentPos[i] - startingFilamentPos[i];
		}
	}
	return objUsedFilament.reduce(function(a, b){ return a + b; }, 0);
}

function estEndTime() {
    var utime = (new Date()).getTime();
    if (layerData.length >= 3 && layerCount > 0) {
		var layerLeft = layerCount - currentLayer;
		// average over the last 5 layers, or all layers if less
        var startAt = (layerData.length > 6) ? layerData.length - 5 : 1;
        var avg5 = (layerData[layerData.length - 1] - layerData[startAt])/(layerData.length - startAt);
        var avg5R = new Date(utime + (avg5 * layerLeft));
        $('span#avg5R').text((avg5 * layerLeft).toHHMMSS()); 
        $('span#avg5').text(avg5R.toLocaleTimeString()); 
	}	
	if (objTotalFilament > 0) {
		var objTotalUsedFilament = getFilamentUsed();
		if (objTotalUsedFilament <= objTotalFilament) {
			var filamentLeft = objTotalFilament - objTotalUsedFilament;
			if (filamentData.length >= 3 && layerCount > 0) {
				var startAt = (layerData.length > 6) ? layerData.length - 5 : 1;
				filamentRate = (filamentData[filamentData.length - 1] - filamentData[startAt])/(layerData[filamentData.length - 1] - layerData[startAt]);
				if (filamentRate != 0) {
					var timeLeft = filamentLeft/filamentRate;
					var estEndTimeFil = new Date(utime + timeLeft);
					$('span#filTimeR').text(timeLeft.toHHMMSS());
					$('span#filTime').text(estEndTimeFil.toLocaleTimeString());
				}
			} else if (objTotalUsedFilament > objTotalFilament * 0.03) {	//if at least 3% filament consumed
				var timeSoFar = utime - printStartTime;
				var timeLeft = timeSoFar * filamentLeft/objTotalUsedFilament;
				var estEndTimeFil = new Date(utime + timeLeft);
				$('span#filTimeR').text(timeLeft.toHHMMSS());
				$('span#filTime').text(estEndTimeFil.toLocaleTimeString());
			}
		}
	}
}

function whichLayer(currZ) {
    if (!layerHeight) {
		layerHeight = storage.get('settings','layerHeight');
	}
    var n = Math.round(currZ / layerHeight);
    if (n === currentLayer + 1 && currentLayer) {
        layerChange();
    }
    return n;
}

function resetLayerData(h, f, reconnecting) {
    //clear layer count,times and chart
	$('input#objheight').val((h == 0) ? "" : h.toString());
	objTotalFilament = f;
	if (reconnecting) {
		startingFilamentPos = [];
		objUsedFilament = currentFilamentPos;
		for(var i = 0; i < currentFilamentPos.length; ++i) {
			startingFilamentPos.push(0);
		}
	} else {
		startingFilamentPos = currentFilamentPos;
		objUsedFilament = [];
		for(var i = 0; i < currentFilamentPos.length; ++i) {
			objUsedFilament.push(0);
		}
	}
    layerData = [];
	filamentData = [];
	if (!reconnecting) {
		printStartTime = null;
	}
    setProgress(0, 'print', 0, 0);
    $('span#elapsed, span#lastlayer, table#finish span').text("00:00:00");
    chart2.setData(parseLayerData());
    chart2.setupGrid();
    chart2.draw();
}

function layerChange() {
    var utime = (new Date()).getTime();
    layerData.push(utime);
	filamentData.push(getFilamentUsed());
    if (layerData.length > maxLayerBars) {
        layerData.shift();
		filamentData.shift();
	}
    if (printStartTime && layerData.length > 1) {
        var lastLayerStart = layerData[layerData.length - 2];
        $('span#lastlayer').text((utime - lastLayerStart).toHHMMSS());
        chart2.setData(parseLayerData());
        chart2.setupGrid();
        chart2.draw();
        if (isNumber(objHeight)) {
            estEndTime();
        }
    }
}

function layers(layer) {
    var utime = (new Date()).getTime();
    if ((layer === 1 || layer === 2) && !printStartTime) {
        printStartTime = utime;
        layerData.push(utime);
		filamentData.push(startingFilamentPos.reduce(function(a, b){ return a + b; }, 0));
    }
    if (printStartTime) {
        $('span#elapsed').text((utime - printStartTime).toHHMMSS());
    }
}

function zeroPrefix(num) {
    var n = num.toString();
    if (n.length === 1) {
        return "0" + n;
    }
    return n;
}

function setProgress(percent, bar, layer, layers) {
	$('progress#'+bar+'ProgressBar').attr('value', percent);
	var ptext = percent + "% Complete";
	if (bar == 'print') {
		if (layers > 0) {
			ptext += ", Layer " + layer + " of " + layers;
		}
		if (objTotalFilament > 0) {
			var currentTotal = currentFilamentPos.reduce(function(a, b) {return a + b; }, 0);
			var startingTotal = startingFilamentPos.reduce(function(a, b) {return a + b; }, 0);
			ptext += ", Filament " + (currentTotal - startingTotal).toFixed(1) + " of " + objTotalFilament + "mm";
		}
	}
	$('span#'+bar+'ProgressText').text(ptext);
}

function parseLayerData() {
    var res = [];
    //res.push([0,0]);
    var elapsed;
    for (var i = 1; i < layerData.length; ++i) {
        elapsed = Math.round((layerData[i] - layerData[i - 1]) / 1000);
        res.push([i, elapsed]);
    }
    return [res];
}

function parseChartData() {
    if (chartData[0].length > maxDataPoints)
        chartData[0].shift();
    if (chartData[1].length > maxDataPoints)
        chartData[1].shift();
    if (chartData[2].length > maxDataPoints)
        chartData[2].shift();
    var res = [[], [], []];
    for (var i = 0; i < chartData[0].length; ++i) {
        res[0].push([i, chartData[0][i]]);
        res[1].push([i, chartData[1][i]]);
        res[2].push([i, chartData[2][i]]);
    }
    return res;
}

function timer() {
    var utime = (new Date()).getTime();
    if (!timerStart) {
        timerStart = utime;
    } else {
        var elapsed = utime - timerStart;
        timerStart = null;
        return elapsed;
    }
}

function poll() {
    if (polling) {
        setTimeout(function() {
            updatePage();
            poll();
        }, storage.get('settings', 'pollDelay'));
    }
}

function getHTMLver() {
    return document.title.substr(document.title.indexOf("v")+1);
}

Number.prototype.toHHMMSS = function() {
    var h,m;
    var sec_num = Math.floor(this / 1000); // don't forget the second param
    var hours = Math.floor(sec_num / 3600);
    var minutes = Math.floor((sec_num - (hours * 3600)) / 60);
    var seconds = sec_num - (hours * 3600) - (minutes * 60);
    hours < 10?hours="0"+hours:false;
    minutes < 10?minutes="0"+minutes:false;
    seconds < 10?seconds="0"+seconds:false;
    hours=='00'?h="":h=hours+"h ";
    minutes=='00'?m="":m=minutes+"m ";
    return h+m+seconds + 's';
};
