#!/bin/bash

# this installation script has been copied from pikrellcam

PGM=`basename $0`

if [ `id -u` == 0 ]
then
    echo -e "$PGM should not be run as root.\n"
    exit 1
fi

bad_install()
	{
	echo "Cannot find $1 in $PWD"
	echo "Are you running $PGM in the install directory?"
	exit 1
	}

if [ ! -d $PWD/picamtracker ]
then
	bad_install "directory picamtracker"
fi

if [ ! -d $PWD/www ]
then
	bad_install "directory www"
fi

sudo chgrp www-data $PWD/www
sudo chmod 775 $PWD/www

if [ ! -d media ]
then
	mkdir -p media/archive media/videos media/thumbs media/stills
	sudo chgrp -R www-data media
	sudo chmod -R 775 media
fi

if [ ! -h www/media ]
then
	ln -s $PWD/media www/media
fi

if [ ! -h www/archive ]
then
	ln -s $PWD/media/archive www/archive
fi

echo ""
echo "Set the port for the nginx web server."
echo "If you already have a web server configuration using the default"
echo "port 80, you should enter an alternate port for piCAMTracker."
echo "Otherwise you can use the default port 80 or an alternate as you wish."
echo "The port number will be set in: /etc/nginx.sites-available/picamtracker."
echo -n "Enter web server port: "
read resp
if [ "$resp" == "" ]
then
	PORT=80
else
	PORT=$resp
fi

echo ""
echo "For auto starting at boot, a piCAMTracker start command must be in rc.local."
echo "If you don't start at boot, piCAMTracker can always be started and stopped"
echo "from the web page."
echo -n "Do you want piCAMTracker to be auto started at boot? (yes/no): "
read resp
if [ "$resp" == "y" ] || [ "$resp" == "yes" ]
then
	AUTOSTART=yes
else
	AUTOSTART=no
fi


HTPASSWD=www/.htpasswd
PASSWORD=""

echo ""
if [ -f $HTPASSWD ]
then
	echo "A web password is already set."
	echo -n "Do you want to change the password (yes/no)? "
	read resp
	if [ "$resp" == "y" ] || [ "$resp" == "yes" ]
	then
		SET_PASSWORD=yes
		rm -f $HTPASSWD
	else
		SET_PASSWORD=no
	fi
else
	SET_PASSWORD=yes
fi

if [ "$SET_PASSWORD" == "yes" ]
then
	echo "Enter a password for a web page login for user: $USER"
	echo "Enter a blank entry if you do not want the password login."
	echo -n "Enter password: "
	read PASSWORD
fi




echo ""
echo "Starting piCAMTracker install..."

# =============== apt install needed packages ===============
#
JESSIE=8
STRETCH=9
V=`cat /etc/debian_version`
DEB_VERSION="${V:0:1}"

PACKAGE_LIST=""

if ((DEB_VERSION >= STRETCH))
then
	PHP_PACKAGES="php7.0 php7.0-common php7.0-fpm"
else
	PHP_PACKAGES="php5 php5-common php5-fpm"
fi

for PACKAGE in $PHP_PACKAGES
do
	if ! dpkg -s $PACKAGE 2>/dev/null | grep Status | grep -q installed
	then
		PACKAGE_LIST="$PACKAGE_LIST $PACKAGE"
	fi
done

for PACKAGE in gpac nginx libav-tools bc \
	sshpass mpack imagemagick apache2-utils libasound2 libasound2-dev \
	libmp3lame0 libmp3lame-dev
do
	if ! dpkg -s $PACKAGE 2>/dev/null | grep Status | grep -q installed
	then
		PACKAGE_LIST="$PACKAGE_LIST $PACKAGE"
	fi
done

if [ "$PACKAGE_LIST" != "" ]
then
	echo "Installing packages: $PACKAGE_LIST"
	echo "Running: apt-get update"
	sudo apt-get update
	sudo apt-get install -y --no-install-recommends $PACKAGE_LIST
else
	echo "No packages need to be installed."
fi


if ((DEB_VERSION < JESSIE))
then
	if ! dpkg -s realpath 2>/dev/null | grep Status | grep -q installed
	then
		echo "Installing package: realpath"
		sudo apt-get install -y --no-install-recommends realpath
	fi
fi


set -x
# =============== picamtracker create startup script  ==============
if [ ! -x /usr/local/bin/runtracker ]; then
  sudo cp $PWD/etc/runtracker /usr/local/bin/.
  sudo sed -i "s/USER/$USER/g" /usr/local/bin/runtracker
fi
set +x

# =============== picamtracker autostart to rc.local  ===============
#
#CMD="su $USER -c '(sleep 5; \/home\/pi\/picamtracker\/picamtracker)  \&'"
CMD="su $USER -c '(sleep 5; /usr/local/bin/runtracker ) \&'"

if [ "$AUTOSTART" == "yes" ]
then
    if ! fgrep -q "$CMD" /etc/rc.local
    then
		if grep -q runtracker /etc/rc.local
		then
			sudo sed -i "/runtracker/d" /etc/rc.local
		fi
		echo "Adding a picamtracker autostart command to /etc/rc.local:"
                sudo sed -i "s|^exit.*|$CMD\n&|" /etc/rc.local
		if ! [ -x /etc/rc.local ]
		then
			echo "Added execute permission to /etc/rc.local"
			sudo chmod a+x /etc/rc.local
		fi
		grep runtracker /etc/rc.local
    fi
else
	if grep -q runtracker /etc/rc.local
	then
		echo "Removing picamtracker autostart line from /etc/rc.local."
		sudo sed -i "/runtracker/d" /etc/rc.local
	fi
fi


# ===== sudoers permission for www-data to run picamtracker as pi ======
#
CMD=$PWD/picamtracker
if ! grep -q "$CMD" /etc/sudoers.d/picamtracker 2>/dev/null
then
	echo "Adding to /etc/sudoers.d: www-data permission to run picamtracker as user pi:"
	cp etc/picamtracker.sudoers /tmp/picamtracker.sudoers.tmp
	sed -i "s|picamtracker|$CMD|" /tmp/picamtracker.sudoers.tmp
	sed -i "s/USER/$USER/g" /tmp/picamtracker.sudoers.tmp
	sudo chown root.root /tmp/picamtracker.sudoers.tmp
	sudo chmod 440 /tmp/picamtracker.sudoers.tmp
	sudo mv /tmp/picamtracker.sudoers.tmp /etc/sudoers.d/picamtracker
#	sudo cat /etc/sudoers.d/picamtracker
fi

# =============== Setup Password  ===============
#
OLD_SESSION_PATH=www/session
if [ -d $OLD_SESSION_PATH ]
then
	sudo rm -rf $OLD_SESSION_PATH
fi

OLD_PASSWORD=www/password.php
if [ -f $OLD_PASSWORD ]
then
	rm -f $OLD_PASSWORD
fi

if [ "$PASSWORD" != "" ]
then
	htpasswd -bc $HTPASSWD $USER $PASSWORD
	sudo chown $USER.www-data $HTPASSWD
fi


# =============== nginx install ===============
#
# Logging can eat many tens of megabytes of SD card space per day
# with the mjpeg.jpg streaming
#
if ! grep -q "access_log off" /etc/nginx/nginx.conf
then
	echo "Turning off nginx access_log."
	sudo sed -i  '/access_log/c\	access_log off;' /etc/nginx/nginx.conf
fi

if ((DEB_VERSION < JESSIE))
then
	NGINX_SITE=etc/nginx-wheezy-site-default
else
	NGINX_SITE=etc/nginx-jessie-site-default
fi

echo "Installing /etc/nginx/sites-available/picamtracker"
echo "    nginx web server port: $PORT"
echo "    nginx web server root: $PWD/www"
sudo cp $NGINX_SITE /etc/nginx/sites-available/picamtracker
sudo sed -i "s|PICAMTRACKER_WWW|$PWD/www|; \
			s/PORT/$PORT/" \
			/etc/nginx/sites-available/picamtracker

if ((DEB_VERSION >= STRETCH))
then
	sudo sed -i "s/php5/php\/php7.0/" /etc/nginx/sites-available/picamtracker
fi

NGINX_SITE=/etc/nginx/sites-available/picamtracker

if [ "$PORT" == "80" ]
then
	NGINX_LINK=/etc/nginx/sites-enabled/default
	CURRENT_SITE=`realpath $NGINX_LINK`
	if [ "$CURRENT_SITE" != "$NGINX_SITE" ]
	then
		echo "Changing $NGINX_LINK link to picamtracker"
		sudo rm -f $NGINX_LINK
		sudo ln -s $NGINX_SITE $NGINX_LINK
	fi
else
	NGINX_LINK=/etc/nginx/sites-enabled/picamtracker
fi

if [ ! -h $NGINX_LINK 2>/dev/null ]
then
	echo "Adding $NGINX_LINK link to sites-available/picamtracker."
	sudo ln -s $NGINX_SITE $NGINX_LINK
fi

if [ ! -f $HTPASSWD ]
then
	echo "A password for the web page is not set."
	sudo sed -i 's/auth_basic/\# auth_basic/' $NGINX_SITE
fi

sudo service nginx restart


# =============== Setup FIFO  ===============
#
fifo=$PWD/www/FIFO

if [ ! -p "$fifo" ]
then
	rm -f $fifo
	mkfifo $fifo
fi
sudo chown $USER:www-data $fifo
sudo chmod 664 $fifo



echo ""
echo "Install finished."
echo "This install script does not automatically start picamtracker."
echo "To start picamtracker, open a browser page to:"
if [ "$PORT" == "80" ]
then
	echo "    http://your_pi"
else
	echo "    http://your_pi:$PORT"
fi
echo "and click on the \"System\" panel and then the \"Start piCAMTracker\" button."
echo "piCAMTracker can also be run from a Pi terminal for testing purposes."
if [ "$AUTOSTART" == "yes" ]
then
	echo "Automatic picamtracker starting at boot is enabled."
fi
echo ""
