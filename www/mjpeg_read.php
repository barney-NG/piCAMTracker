<?php
    require_once(dirname(__FILE__) . '/config.php');
?>
<?php
  $mjpeg_file = file_get_contents(MJPEG_CTRL_FILE);
	header("Content-Type: image/jpeg");
        #flush();
	readfile($mjpeg_file);
?>
