<?php
	require_once(dirname(__FILE__) . '/config.php');
?>

<?php
if (isset($_GET['cmd']))
	{
	$cmd = $_GET['cmd'];

	if ($cmd === "picamtracker_start")
		{
		$SUDO_CMD = "sudo -u pi " . PICAMTRACKER . " > /dev/null 2>&1 &";
		$res = exec($SUDO_CMD);
		}
	else if ($cmd === "picamtracker_stop")
		{
		$fifo = fopen(FIFO_FILE,"w");
		fwrite($fifo, "quit");
		fclose($fifo);
		usleep(500000);
		exec("pgrep python", $output, $return);
		if ($return == 0)
			exec('killall -1 python');
		}
	}
?>
