<?php
	define("PROGRAM", "piCAMTRacker");
	define("HOST", php_uname("n"));
  	define("TITLE_STRING", PROGRAM . "@" . HOST);

// The 'media' directory in these defines is a link to the media_dir
// which is configured in ~/.piCAMTracker/pikrellcam.conf and the link is
// automatically altered by the startup script if media_dir is changed.
// The videos, stills and timelapse subdirs are fixed to match PiKrellCam.
// Do not change these here.
//
	define("VIDEO_DIR", "media/videos");
	define("STILL_DIR", "media/stills");
	define("TIMELAPSE_DIR", "media/timelapse");

	define("ARCHIVE_DIR", "archive");

// These are set up by the install or piCAMTracker.conf and enforced by
// the startup script.  It is no use to change these here.
//
	define("LOG_FILE", "/tmp/picamtracker.log");
	define("MJPEG_FILE", "/run/picamtracker/mjpeg.jpg");
	//define("PIKRELLCAM", "/home/pi/piCAMTracker/pikrellcam");
	define("FIFO_FILE", "/home/pi/piCAMTracker/www/FIFO");

	define("VERSION", "0.1.0");
?>
