//



// How to create an HTTP request
function create_XMLHttpRequest()
{
if (window.XMLHttpRequest)
    return new XMLHttpRequest();    // IE7+, Firefox, Chrome, Opera, Safari
else
    return new ActiveXObject("Microsoft.XMLHTTP");  // IE6, IE5
}


// everything aroud image reading 
var mjpeg;
var url = window.location.search;
var refreshtime = url.substring(url.lastIndexOf("=")+1);
if (refreshtime == "") { refreshtime = 970; }

function mjpeg_read()
{
    setTimeout("mjpeg.src = 'mjpeg_read.php?time=' + new Date().getTime();", refreshtime);
    showGreenLed(true)
}
function mjpeg_error()
{
    showGreenLed(false);
}
function mjpeg_start()
{
    read_config.send()
    mjpeg = document.getElementById("mjpeg_image");
    mjpeg.onload = mjpeg_read;
    mjpeg.onerror = mjpeg_error;
    mjpeg_read();
}


// command interface is currently based on PUSH to php procedure
var fifo_cmd = create_XMLHttpRequest();
fifo_cmd.timeout = 5000;
fifo_command.ontimeout =  function (e) {
    // XMLHttpRequest timed out. Do something here.
    showGreenLed(false);
}

function fifo_command (cmd, val)
{
    var text = cmd + val + ";"
    fifo_cmd.open("PUT", "fifo_command.php?cmd=" + text, true);
    fifo_cmd.send();
}

//var sys_cmd = create_XMLHttpRequest();

// read config file and update document elements
var read_config = create_XMLHttpRequest();
read_config.onreadystatechange = function()
{
    if(this.readyState == 4 && this.status == 200) {
        var cfg = JSON.parse(this.responseText);
        console.log("CFG: " + JSON.stringify(cfg));
        // set fastmode checkbox status
        if(cfg.fastMode == true) {
            document.getElementById("fm_on").click();
        } else {
            document.getElementById("fm_off").click();
        }
        // set baseB checkbox status
        if(cfg.baseB == 'right') {
            document.getElementById("right").click();
        } else if(cfg.baseB == 'left') {
            document.getElementById("left").click();
        } else {
            document.getElementById("both").click();
        }

        // set slider values
        $('#slider-vMin').val(cfg.vMin).slider('refresh');
        $('#slider-vMax').val(cfg.vMax).slider('refresh');
        $('#slider-MinArea').val(cfg.minArea).slider('refresh');
        $('#slider-MaxArea').val(cfg.maxArea).slider('refresh');
        //$('#slider-NoiseLevel').val(cfg.sadThreshold).slider('refresh');
        $('#slider-MaxMotionDistance').val(cfg.maxDist).slider('refresh');
        $('#slider-DetectionMaturity').val(cfg.trackMaturity).slider('refresh');
        $('#slider-ObjectExtension').val(cfg.extension).slider('refresh');
        $('#slider-ExposureCompensation').val(cfg.exposure).slider('refresh');
        // show green status
        showGreenLed(true);
    } else {
        // show red status
        showGreenLed(false);
    }
}

// append read of config file to document.ready state
$(document).ready(function() { read_config.open("GET","config.json",true); });

function showGreenLed(on)
{
    // blue led off so far
    document.getElementById("ledB1").style.visibility = "hidden";
    document.getElementById("ledB2").style.visibility = "hidden";
    document.getElementById("ledB3").style.visibility = "hidden";

    if(on) {
        // link status (green) on
        document.getElementById("ledG1").style.visibility = "visible";
        document.getElementById("ledG2").style.visibility = "visible";
        document.getElementById("ledG3").style.visibility = "visible";
        // link status (red) of
        document.getElementById("ledR1").style.visibility = "hidden";
        document.getElementById("ledR2").style.visibility = "hidden";
        document.getElementById("ledR3").style.visibility = "hidden";
    } else {
        // link status (green) of
        document.getElementById("ledG1").style.visibility = "hidden";
        document.getElementById("ledG2").style.visibility = "hidden";
        document.getElementById("ledG3").style.visibility = "hidden";
        // link status (red) on
        document.getElementById("ledR1").style.visibility = "visible";
        document.getElementById("ledR2").style.visibility = "visible";
        document.getElementById("ledR3").style.visibility = "visible";
    }
}

