		// Setup the WebSocket connection and start the player
		var client = new WebSocket( 'ws://'+location.host+':8084/' );
		var canvas = document.getElementById('videoCanvas');
 		var VidCanvContext = canvas.getContext('2d');

		var player = new jsmpeg(client, {canvas:canvas});
                
		function clearCanvas() {
			VidCanvContext.clearRect(0, 0, canvas.width, canvas.height);
			VidCanvContext.fillStyle = "white";
			VidCanvContext.font = "bold 22px Arial";
			VidCanvContext.fillText("Video Stream Off", 130, 300);
		};

		var webRep = 0;
		var webLinked = 0;
		var thresholdValue = '2';
		var mpegFileName = '---';
		var datetime = '';
		var minareaValue = $( '#slider-MinArea' ).val();
		var maxareaValue = $( '#slider-MaxArea' ).val();
		var minmotionValue = $( '#slider-MinMotionDistance' ).val();
		var maxmotionValue = $( '#slider-MaxMotionDistance' ).val();
		var maturityValue = $( '#slider-DetectionMaturity' ).val();
		var maxangleValue = $( '#slider-MaxTrackingAngle' ).val();
		var maxtracesValue = $( '#slider-MaxTraces' ).val();
		var lifetimeValue = $( '#slider-TraceLifetime' ).val();
		var averageweightValue = $( '#slider-AverageWeight' ).val();
		
		//setup websocket remote control 
		var clientRC = new WebSocket( 'ws://'+location.host+':8086/' );
		
		clientRC.onerror = function(errstr) {
		  console.log('error: ', errstr);
		}
		clientRC.onmessage = function(e) {
			console.log('Socket Interface Process (si) sent: ' + e.data);
			var varobj = JSON.parse(e.data);
			// Test: btn1
			// ignored

			thresholdValue = varobj.thr;
			minareaValue = varobj.mina;
			maxareaValue = varobj.maxa;
			minmotionValue = varobj.minm;
			maxmotionValue = varobj.maxm;
			maturityValue = varobj.maturity;
			maxangleValue = varobj.mangle;
			maxtracesValue = varobj.maxt;
			lifetimeValue = varobj.trcl;
			averageweightValue = varobj.bgw;

			var activePage = $.mobile.activePage.attr('id');
			
			if (activePage == "page1") {
				// Plane from left
				if (varobj.btn2 == '1') {
					$( "#label3" ).addClass('ui-checkbox-on').removeClass('ui-checkbox-off');
				} else {
					$( "#label3" ).addClass('ui-checkbox-off').removeClass('ui-checkbox-on');
				}

				// Normal Training
				if (varobj.btn3 == '1') {
					$("input[name=rbCmd][value='3']").prop("checked", true);
					$("input[name=rbCmd]").checkboxradio("refresh");
					clearCanvas();
				}
				// Stream
				if (varobj.btn4 == '1') {
					$("input[name=rbCmd][value='4']").prop("checked", true);
					$("input[name=rbCmd]").checkboxradio("refresh");
				}
				// Record
				if (varobj.btn5 == '1') {
					$("input[name=rbCmd][value='5']").prop("checked", true);
					$("input[name=rbCmd]").checkboxradio("refresh");
					clearCanvas();
				}
				// Replay
				if (varobj.btn6 == '1') {
					$("input[name=rbCmd][value='6']").prop("checked", true);
					$("input[name=rbCmd]").checkboxradio("refresh");
				}
				// Record Mpeg, no beep
				if (varobj.btn7 == '1') {
					$("input[name=rbCmd][value='7']").prop("checked", true);
					$("input[name=rbCmd]").checkboxradio("refresh");
					clearCanvas();
				}
				// Replay Mpeg, no beep
				if (varobj.btn8 == '1') {
					$("input[name=rbCmd][value='8']").prop("checked", true);
					$("input[name=rbCmd]").checkboxradio("refresh");
				}
				// Idle
				if (varobj.btn9 == '1') {
					$("input[name=rbCmd][value='9']").prop("checked", true);
					$("input[name=rbCmd]").checkboxradio("refresh");
					clearCanvas();
				}
			
				// MPEG File Name, update on change
				if (datetime != varobj.dt) {
					datetime = varobj.dt;
				}
				if (varobj.btn5 == '1') {
					$("#txt2" ).val('fcd_' + datetime + '.avi');
				}
				if (varobj.btn7 == '1') {
					$("#txt2" ).val('fcd_' + datetime + '.mpeg');
				}
			}

			if (activePage == "page3") {
				if (varobj.flpc == '1') {
					$( "#flipCam" ).addClass('ui-checkbox-on').removeClass('ui-checkbox-off');
				} else {
					$( "#flipCam" ).addClass('ui-checkbox-off').removeClass('ui-checkbox-on');
				}

				// select new option, remove 'selected' from all others, and refresh menue 
				$('#thresh option').eq(thresholdValue-1).prop('selected', true).siblings('option').removeAttr('selected');
				$('#thresh').selectmenu("refresh", true);

				$('#slider-NoiceLevel').val(thresholdValue).slider('refresh');
				$('#slider-MinArea').val(minareaValue).slider('refresh');
				$('#slider-MaxArea').val(maxareaValue).slider('refresh');
				$('#slider-MinMotionDistance').val(minmotionValue).slider('refresh');
				$('#slider-MaxMotionDistance').val(maxmotionValue).slider('refresh');
				$('#slider-DetectionMaturity').val(maturityValue).slider('refresh');
				$('#slider-MaxTrackingAngle').val(maxangleValue).slider('refresh');
				$('#slider-MaxTraces').val(maxtracesValue).slider('refresh');
				$('#slider-TraceLifetime').val(lifetimeValue).slider('refresh');
				$('#slider-AverageWeight').val(averageweightValue).slider('refresh');
			}
		};

		function submitCmd() {
			// Get command code
			var cmd = $('input[name=rbCmd]:checked').val();
			console.log('cmd: ' + cmd);
			if ((cmd == 7) || (cmd == 5)) {
				// provide date and time as part of file name
				var currentdate = new Date(); 
			    var datetime;
			    yy = currentdate.getFullYear();
			    mo = (currentdate.getMonth() + 1);
			    if (mo < 10) { mo = '0' + mo};
			    dd = currentdate.getDate();
			    if (dd < 10) { dd = '0' + dd};
			    hh = currentdate.getHours();
			    if (hh < 10) { hh = '0' + hh};
			    mn = currentdate.getMinutes();
			    if (mn < 10) { mn = '0' + mn};
			    ss = currentdate.getSeconds();
			    if (ss < 10) { ss = '0' + ss};
			    datetime = yy + mo + dd + hh + mn + ss;
			}
			if (cmd == 5) {
				$("#txt2" ).val('fcd_' + datetime + '.avi');
			    clientRC.send('D' + datetime);
			} 
			if (cmd == 7) {
				$("#txt2" ).val('fcd_' + datetime + '.mpeg');
			    clientRC.send('D' + datetime);
			} 
			clientRC.send(cmd);
			playSound1();
		};
		function testCmd() {
			// Tell the server this is button 1
			clientRC.send("1");
			playSound3();
		};
		function storeParamsCmd() {
			clientRC.send("S1");
			playSound1();
		};
		function abaseCmd() {
			// Tell the server this is button 3
			clientRC.send("2");
			playSound1();
		};
		function flipcamCmd() {
			clientRC.send("F1");
			playSound1();
		};
		function selectCmd() {
			playSound1();
			thresholdValue = $( '#thresh' ).val();
			clientRC.send("T" + thresholdValue);
			console.log('New threshold is', thresholdValue);
		};
		function threshCmd() {
			playSound1();
			thresholdValue = $( '#slider-NoiceLevel' ).val();
			clientRC.send("T" + thresholdValue);
			console.log('New threshold is', thresholdValue);
		};
		function minareaCmd() {
			playSound1();
			minareaValue = $( '#slider-MinArea' ).val();
			clientRC.send("m" + minareaValue);
			console.log('New Minimum Area is', minareaValue);
		};
		function maxareaCmd() {
			playSound1();
			maxareaValue = $( '#slider-MaxArea' ).val();
			clientRC.send("M" + maxareaValue);
			console.log('New Maximum Area is', maxareaValue);
		};
		function minmotionCmd() {
			playSound1();
			minmotionValue = $( '#slider-MinMotionDistance' ).val();
			clientRC.send("n" + minmotionValue);
			console.log('New Minimum Motion Distance is', minmotionValue);
		};
		function maxmotionCmd() {
			playSound1();
			maxmotionValue = $( '#slider-MaxMotionDistance' ).val();
			clientRC.send("N" + maxmotionValue);
			console.log('New Maximum Motion Distance is', maxmotionValue);
		};
		function maturityCmd() {
			playSound1();
			maturityValue = $( '#slider-DetectionMaturity' ).val();
			clientRC.send("N" + maturityValue);
			console.log('New Detection Maturity is', maturityValue);
		};
		function maxangleCmd() {
			playSound1();
			maxangleValue = $( '#slider-MaxTrackingAngle' ).val();
			clientRC.send("A" + maxangleValue);
			console.log('New Maximum Tracking Angle is', maxangleValue);
		};
		function maxtracesCmd() {
			playSound1();
			maxtracesValue = $( '#slider-MaxTraces' ).val();
			clientRC.send("t" + maxtracesValue);
			console.log('New Maximum Traces is', maxtracesValue);
		};
		function lifetimeCmd() {
			playSound1();
			lifetimeValue = $( '#slider-TraceLifetime' ).val();
			clientRC.send("L" + lifetimeValue);
			console.log('New Maximum Traces is', lifetimeValue);
		};
		function averageweightCmd() {
			playSound1();
			averageweightValue = $( '#slider-AverageWeight' ).val();
			clientRC.send("W" + averageweightValue);
			console.log('New Average Weight is', averageweightValue);
		};

		//setup websocket websound crossing alert 
               
		var clientSound = new ReconnectingWebSocket( 'ws://'+location.host+':8087/' );


		clientSound.onerror = function(errstr) {
			console.log('error: ', errstr);
		}
		clientSound.onmessage = function(e) {
			//console.log(e.data);
			var sndobj = JSON.parse(e.data);
			if (sndobj.cross == '1') {
				playSound2();
 			};
			$( "#txt1" ).val(sndobj.fps);
			// replay done status; 0: no replay, 1: replay on, but playlist done, 2: replay on
			webRep = sndobj.rep;
		}
                

	var audioCtx = new (window.AudioContext || window.webkitAudioContext || window.audioContext);

	// All arguments are optional:

	// duration of the tone in milliseconds. Default is 500
	// frequency of the tone in hertz. default is 440
	// volume of the tone. Default is 1, off is 0.
	// type of tone. Possible values are sine, square, sawtooth, triangle, and custom. Default is sine.
	// callback to use on end of tone

	function beep(duration, frequency, volume, type, callback) {

	    var oscillator = audioCtx.createOscillator();
	    var gainNode = audioCtx.createGain();

	    oscillator.connect(gainNode);
	    gainNode.connect(audioCtx.destination);

	    if (volume){gainNode.gain.value = volume;};
	    if (frequency){oscillator.frequency.value = frequency;}
	    if (type){oscillator.type = type;}
	    if (callback){oscillator.onended = callback;}

	    oscillator.start();
	    setTimeout(function(){oscillator.stop()}, (duration ? duration : 500));
	};

	function playSound1() {
		beep(100, 1500, 0.1);
	};
	function playSound2() {
		beep(300, 2500, 0.8);
	};
	function playSound3() {
		beep(10, 1000, 0.01);
	};


	$('#txt1').textinput({disabled: true});
	$("#txt1" ).val('---');
	$('#txt2').textinput({disabled: true});
	$("#txt2" ).val('---');


	// show websocket link status; initial value is red, i.e. hide green led
	document.getElementById("ledG1").style.visibility = "hidden";
	document.getElementById("ledG2").style.visibility = "hidden";
	document.getElementById("ledG3").style.visibility = "hidden";
	// replay/record-LED (blue) initially off
	document.getElementById("ledB1").style.visibility = "hidden";
	document.getElementById("ledB2").style.visibility = "hidden";
	document.getElementById("ledB3").style.visibility = "hidden";

	var tim = setInterval(myTimer, 2000);
	function myTimer() {
		// update connection status
		if ((client.readyState == '1') && (clientRC.readyState == 1) && (clientSound.readyState == 1)) {
			document.getElementById("ledR1").style.visibility = "hidden";
			document.getElementById("ledG1").style.visibility = "visible";
			document.getElementById("ledR2").style.visibility = "hidden";
			document.getElementById("ledG2").style.visibility = "visible";
			document.getElementById("ledR3").style.visibility = "hidden";
			document.getElementById("ledG3").style.visibility = "visible";
			webLinked = 1;
			}
		else {
			document.getElementById("ledG1").style.visibility = "hidden";
			document.getElementById("ledR1").style.visibility = "visible";
			document.getElementById("ledG2").style.visibility = "hidden";
			document.getElementById("ledR2").style.visibility = "visible";
			document.getElementById("ledG3").style.visibility = "hidden";
			document.getElementById("ledR3").style.visibility = "visible";
//			$( "#txt1" ).text('FPS: 0');
			$( "#txt1" ).val('0');
			webLinked = 0;
			webRep = 0;
			};

		if (webRep == 1) {
			document.getElementById("ledB1").style.visibility = "visible";
			document.getElementById("ledB2").style.visibility = "visible";
		}
		if (webRep == 0) {
			document.getElementById("ledB1").style.visibility = "hidden";
			document.getElementById("ledB2").style.visibility = "hidden";
		};
	};

	var tim_t = setInterval(myTimer_t, 500);
	var toggle = 0;
	function myTimer_t() {
		if (webRep == 2) {
			if (toggle == 0) {
				document.getElementById("ledB1").style.visibility = "visible";
				document.getElementById("ledB2").style.visibility = "visible";
				toggle = 1;
			}
			else {
				document.getElementById("ledB1").style.visibility = "hidden";
				document.getElementById("ledB2").style.visibility = "hidden";
				toggle = 0;
			};
		};
	};
