#!/bin/sh
#
# S70vacrouter.sh - INIT script for use on the Openmiko firmware
# Could/should it happen earlier in init?  Need network working
#
# Built in to the image but perhaps should be on SD card instead?
# Files on SD card (VFAT) in /config/overlay/* are copied at boot
# So /config/overlay/etc/init.d/thisfile.sh would get copied to /etc/init.d
#
# 
# Starts ardith and the vacrouter operating scripts
#

start() {
        printf "Starting vacrouter: "
        start-stop-daemon -S  -b --exec /sdcard/ardith.sh
        start-stop-daemon -S  -b --exec /sdcard/vacrouter.sh
        echo "OK"
}
stop() {
        printf "Stopping vacrouter: "
        #start-stop-daemon -K -q -p $( pidof vacrouter.sh )
        killall vacrouter.sh
        killall ardith.sh
        echo "OK"
}
restart() {
        stop
        start
}

case "$1" in
  start)
        start
        ;;
  stop)
        stop
        ;;
  restart|reload)
        restart
        ;;
  *)
        echo "Usage: $0 {start|stop|restart}"
        exit 1
esac

exit $?
