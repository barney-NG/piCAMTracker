<?php
    require_once(dirname(__FILE__) . '/config.php');
?>
<?php
    $mjpeg_file = file_get_contents(MJPEG_CTRL_FILE);
    header("Content-Type: image/jpeg");
    #flush();
    if($mjpeg_file) {
	readfile($mjpeg_file);
    } else {
	readfile(EMPTY_FILE);
    }
?>
