<?php
    require_once(dirname(__FILE__) . '/config.php');
?>
<?php

	header("Content-Type: image/jpeg");
        #flush();
	readfile(MJPEG_FILE);
?>
