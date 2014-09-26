var canvas_height = 500;
var canvas_width = 500;
var surfacePlot;
var surfacePlot2;
var data, options, basicPlotOptions, glOptions, animated, plot1, values;
var numRows = 10;
var numCols = 10;
var tooltipStrings = new Array();

function setUp() {

    values = new Array();

    data = {
        nRows: numRows,
        nCols: numCols,
        formattedValues: values
    };

    surfacePlot = new SurfacePlot(document.getElementById("surfacePlotDiv"));

    // Don't fill polygons in IE < v9. It's too slow.
    var fillPly = true;

    // Define a colour gradient.
    var colour1 = {
        red: 0,
        green: 0,
        blue: 255
    };
    var colour2 = {
        red: 0,
        green: 255,
        blue: 255
    };
    var colour3 = {
        red: 0,
        green: 255,
        blue: 0
    };
    var colour4 = {
        red: 255,
        green: 255,
        blue: 0
    };
    var colour5 = {
        red: 255,
        green: 0,
        blue: 0
    };

    var colours = [colour1, colour2, colour3, colour4, colour5];

    // Axis labels.
    var xAxisHeader = "X-axis";
    var yAxisHeader = "Y-axis";
    var zAxisHeader = "Z-axis";
    var renderDataPoints = false;
    var background = '#ffffff';
    var axisForeColour = '#000000';
    var hideFloorPolygons = true;

    var chartOrigin = {
        x: 200,
        y: 265
    };

    // Options for the basic canvas plot.
    basicPlotOptions = {
        fillPolygons: fillPly,
        tooltips: tooltipStrings,
        renderPoints: renderDataPoints
    }

    // Options for the webGL plot.
    var xLabels = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    var yLabels = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    var zLabels = [0, 1, 2, 3, 4, 5, 6];

    // These labels are used when autoCalcZScale is false;
    glOptions = {
        xLabels: xLabels,
        yLabels: yLabels,
        zLabels: zLabels,
        autoCalcZScale: true
    };

    // Options common to both types of plot.
    options = {
        xPos: 0,
        yPos: 0,
        width: canvas_width,
        height: canvas_height,
        colourGradient: colours,
        xTitle: xAxisHeader,
        yTitle: yAxisHeader,
        zTitle: zAxisHeader,
        backColour: background,
        axisTextColour: axisForeColour,
        hideFlatMinPolygons: hideFloorPolygons,
        origin: chartOrigin
    };

    // Create some data.
    var d = 360 / numRows;
    var idx = 0;

    for (var i = 0; i < numRows; i++) {
        values[i] = new Array();

        for (var j = 0; j < numCols; j++) {
            var value = (Math.cos(i * d * Math.PI / 180.0) * Math.cos(j * d * Math.PI / 180.0));

            values[i][j] = (value / 4.0 + 0.25) * 0.5;

            tooltipStrings[idx] = "x:" + i + ", y:" + j + " = " + value;
            idx++;
        }
    }

    surfacePlot = new SurfacePlot(document.getElementById("surfacePlotDiv"));
    surfacePlot.draw(data, options, basicPlotOptions, glOptions);
}


function scanBed() {
    var step = 10;
    var zdata = new Array();
    var data;
    var X = 60;
    var Y = 20;
    var Yend = 170;
    var movePreCode = "M120\nG91\nG1 ";
    var movePostCode = "\nM121";
    $.askElle('gcode', movePreCode + "X" + X + " Y" + Y + " F200" + movePostCode);
    while(Y < Yend) {
        data = $.askElle('poll', '');
        zdata.push(data.probe);
        Y += step;
        $.askElle('gcode', movePreCode + "X" + X + " Y" + Y + " F200" + movePostCode);
    }
    
}

ormerodIP = "192.168.1.144";

scanBed();
//setUp();