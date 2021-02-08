<?php
    require_once(dirname(__FILE__) . '/config.php');
?>
<?php
    if(array_key_exists('image',$_GET)) {
        $mjpeg_file = $_GET['image'];
        #error_log("image: $mjpeg_file\n", 3, '/tmp/aaa.log');
    } else {
        $mjpeg_file = file_get_contents(MJPEG_CTRL_FILE);
    }


    header("Content-Type: image/jpeg");
    #flush();
    if(file_exists($mjpeg_file)) {
	readfile($mjpeg_file);
    } else {
	readfile(EMPTY_FILE);
    }
?>
