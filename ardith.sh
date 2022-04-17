#!/bin/bash
#
# ardith.sh     a simple bash script to provide a command line interface to an arduino Mega 2560
#               arduino running code based on this commmand line structure: https://create.arduino.cc/projecthub/mikefarr/simple-command-line-interface-4f0a3f
#               This script initializes and holds open the serial port and does initial homing.
#               Other than homing, all other commands are handled by vacrouter.sh
#               JAC
#
# Version       .1  3/9/2022 - First version
#               .2 3/15/2022 - Simplified script as we will do all sending from vacrouter.sh now
#
# TODO:         -Store last received line in /tmp
#set -x

# CONFIGURATION
CONSOLE="/dev/ttyACM0"

# Last line temp file
TMP_LINE=/tmp/lastline.txt

# Configure the serial port
stty -F $CONSOLE cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts

# Times & Flags
START_DELAY=0   # Give the Arduino time to become ready after serial connect
LINE_DELAY=1
START_FLAG=1    # Indicate first run state
LOCAL_ECHO=0

### Functions
Serial.println() {
    echo "$1" > $CONSOLE
    if [ $LOCAL_ECHO = 1 ]; then
    echo "$1"
    fi
}

# On startup, ensure there are is no output left in /tmp
echo > $TMP_LINE

while read -r LINE; do
    # Strip tabs & EOL character
    CLEAN_LINE=${LINE//[$'\t\r\n']}
    LINE="$CLEAN_LINE"

#  echo "Line: $LINE"
    # Match the first 4 characters of the reset banner line
    if [[ ${LINE:0:4} == "Vacr" ]]; then
        START_FLAG=1
    fi


    if [[ ${LINE:0:7} == "OK PPOS" ]]; then
        #0   1     2  3    4
        #OK PPOS: -1 CPOS: 2
        read -ra LINEARRAY <<< $LINE
        STATUS=$(echo ${LINEARRAY[0]})
        PPOS=$(echo ${LINEARRAY[2]})
        CPOS=$(echo ${LINEARRAY[4]})
        echo "ARDITH: STATUS=$STATUS PPOS=$PPOS CPOS=$CPOS"
    fi

   # Add delay for the port to become ready, if necessary
   if [ $START_FLAG == 1 ]; then
     Serial.println "HOME"
     START_FLAG=2

   fi
    # If no other task, print the line
     echo $LINE
     echo $LINE > $TMP_LINE

done < $CONSOLE
