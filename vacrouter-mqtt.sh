#!/bin/bash
#
# vacrouter.sh - MQTT interface script for SmartShop Vacuum Router events
#
# JAC - March 8, 2022
#
# Rev .1        -First version
# Rev .2        -Added more error handling around the temp file creation
# Rev .3        -Added new MOVE GOCNC, MOVE GOCHOPSAW and MOVE GOWORKBENCH commands
#               -Powering off the vacuum returns to CHOPSAW position when complete
#
# TODO:         -Monitor to amke sure ardith.sh is running

# Bash debug, can be moved anywhere, set +x to turn off if you want to isolate an area
#set -x

DEBUG=1                 # Enable (1) / Disable (0) Console log messages


#  VARIABLES & PATHS
VAC_DELAY_DEF=2         # Number of seconds to leave vacuum on to clear the line, before shutting down
CONSOLE=/dev/ttyACM0    # Port Arduino is connected to (stty setup handled by ardith which holds the port open and reads lines)
TMP_LASTLINE=/tmp/lastline.txt  # Stores the last line of text sent on the serial port
DEVICE=""               # MQTT device (e.g. cnc, chopsaw etc) - Nulled to allow test to skip case stmt in main loop
JQ=/usr/bin/jq          # JSON slicer for mosquitto_sub output
VACR_CPOS=""            # Store the current position of the arm
VACR_STATE=""           # Store the last reported state of the arm
OLDLASTLINE=""          # Store the previous serial line received, for comparison
NOHUP=/usr/bin/nohup    # No allow ardith to keep running in the event we stop and start this script
PIDOF=/bin/pidof        # To get the pid of ardith.sh on startup
SLEEP=/bin/sleep
ARDITH=/sdcard/ardith.sh        # The script that opens the serial port, homes the machine, writes Rxd serial line
ARDITH_SHORT=ardith.sh          # For pidof

### MQTT variables
# Make sure MQTT topics have no leading slash and single quotes
# TOPICS TO SUBSCRIBE TO
TOPIC_POWER='stat/+/POWER'      # Match all devices that report POWER state

# COMMANDS 
VAC_POWER_CMD='cmnd/vacuum/POWER'

# TOPICS TO PUBLISH TO
VACR_ST_TOPIC='stat/vacrouter/STATE'            # Last state read from serial
VACR_CPOS_TOPIC='stat/vacrouter/POSITION'       # Last position of arm read from serial

# MQTT Running on the router via Entware
BROKER=192.168.2.1
M_PUB_PORT=1883
M_SUB_PORT=1883

# MQTT PUBLISH command
M_PUB=/"usr/bin/mosquitto_pub"

# MQTT PUBlISH options
# -h MQTT Broker IP
# -t topic to publish to
# -m message to publish

MSG_PUBLISH() {
        PUB_TOPIC=$1
        PUB_MSG=$2
        M_PUB_OPTS="-h $BROKER -p $M_PUB_PORT -t $PUB_TOPIC -m $PUB_MSG"
        $M_PUB $M_PUB_OPTS
}

# MQTT SUBSCRIBE command
M_SUB="/usr/bin/mosquitto_sub"

# MQTT SUBSCRIBE options
# -C 1, read one message and exit
# -h MQTT Broker IP
# -v Verbose shows the full topic in output then space then msg
# -N don't append a \n newline to the end of the string
# -W x Where x is seconds to wait before timeout and exit
# --quiet Suppress extraneous output
## NOTE Topic must be at the end of the options for scripting here
# -t topic to subscribe to

M_SUB_OPTS="-h $BROKER -p $M_SUB_PORT -C 1 -v -N --quiet -t"

MSG_SUBSCRIBE() {
#       IFS=" "
        M_SUB_TOPIC=$1
        $M_SUB $M_SUB_OPTS $M_SUB_TOPIC
}

# Leaving this here in case I change my mind and want to write states to /tmp
oldUPDATE_STATE_FILE(){
        # STATE FILES
        if [ ! -d $TMP_DIR/${TOPIC[0]} ]; then
                mkdir $TMP_DIR/${TOPIC[0]}
                LOG ${FUNCNAME[0]} "mkdir $TMP_DIR/${TOPIC[0]}"
        fi
        if [ ! -d $TMP_DIR/${TOPIC[0]}/${TOPIC[1]} ]; then
                mkdir $TMP_DIR/${TOPIC[0]}/${TOPIC[1]}
                LOG ${FUNCNAME[0]} "mkdir $TMP_DIR/${TOPIC[0]}/${TOPIC[1]}"
        fi
        if [ ! -d $TMP_DIR/${TOPIC[0]}/${TOPIC[1]}/${TOPIC[2]} ]; then
                mkdir $TMP_DIR/${TOPIC[0]}/${TOPIC[1]}/${TOPIC[2]}
                LOG ${FUNCNAME[0]} "mkdir $TMP_DIR/${TOPIC[0]}/${TOPIC[1]}/${TOPIC[2]}"
        fi

        LOG  ${FUNCNAME[0]} "Writing $TOPIC_MSG > $TMP_DIR/${TOPIC[0]}/${TOPIC[1]}/${TOPIC[2]}/value"
        echo "$TOPIC_MSG" > "$TMP_DIR/${TOPIC[0]}/${TOPIC[1]}/${TOPIC[2]}/value"
        LOG ${FUNCNAME[0]} ""$DEVICE" "$DEVICE_VAR" status is: "$TOPIC_MSG""
}

# Parse the MQTT messages for topic, takes one argument, topic e.g. stat/cnc/POWER
# Note - this is a delaying function (1 message or 1s timeout) to allow serial read in main loop
# Use wildcards as appropriate e.g. stat/+/POWER
MON_TOPIC() {
        # LOG ${FUNCNAME[0]} "Subscribing to $1"
        RESPONSE_RAW=$(MSG_SUBSCRIBE $1)        # When a topic and message arrive, load them in a variable e.g. "/state/cnc/POWER on"
        read -ra RESPONSE <<< $RESPONSE_RAW     # Read the variable in to an array using the default IFS of space
        TOPIC_RAW=$(echo ${RESPONSE[0]})        # Reference array variable for full topic
        TOPIC_MSG=$(echo ${RESPONSE[1]})        # Reference array variable for message
        TOPIC_SPACED=$( echo ${RESPONSE[0]//\// } )  # Convert topic slashes to spaces to make it easier to work with
        read -ra TOPIC <<< $TOPIC_SPACED
        DEVICE="$(echo ${TOPIC[1]})"            # Reference the device from the topic tree
        # Slice from the second item (third position) to the end of the array in case the topic is longer
        # Should really count elements here, todo
        #DEVICE_VAR="$(echo ${TOPIC[2]:-1})"
        DEVICE_VAR="$(echo ${TOPIC[2]})"        # Reference the sub-topic for the device
}

### Miscellaneous Functions
# arg1 = function name from ${FUNCNAME[0]}
# arg2 = message to log
LOG() {
        if [ $DEBUG = 1 ]; then
                echo "$(date) $1: $2"
        fi
}

INIT() {
        LOG ${FUNCNAME[0]} "*** Vacrouter v1.0 ***"
        # Start Ardith if not already running
        ARDITH_PID=$($PIDOF $ARDITH_SHORT)
        if [[ -z $ARDITH_PID ]]; then
                LOG ${FUNCNAME[0]} "Initializing ardith.sh for background arduino communication."
                echo > $TMP_LASTLINE
                $NOHUP $ARDITH >/dev/null 2>&1 &
                INIT_WAIT_HOMING=1
        else
                LOG ${FUNCNAME[0]} "ardith.sh appears to be running alreading, continuing..."        
        fi
        # Publish init state at startup
        LOG ${FUNCNAME[0]} "Publishing $VACR_ST_TOPIC INIT state"
        MSG_PUBLISH $VACR_ST_TOPIC INIT
        # NOTE: Moved discovery in to SERIAL during homing to parallelize it
        SERIAL
        LOG ${FUNCNAME[0]} "INIT Complete"
}


SERIAL() {
        LASTLINE=$( cat $TMP_LASTLINE )
        if [ ${FUNCNAME[1]} == "INIT" ] && [ "$INIT_WAIT_HOMING" == "1" ]; then
                LOG ${FUNCNAME[0]} "Waiting for HOME signal from ardith."
                LASTLINE=$( cat $TMP_LASTLINE )
                # Loop until HOME is reported on serial port
                until [[ ${LASTLINE:0:4} == "HOME" ]]
                do
                        LASTLINE=$( cat $TMP_LASTLINE )
                done

                # Do a discovery while we wait
                if [[ ${LASTLINE:0:4} == "HOME" ]]; then
                        # Not using this info for anything yet
                        LOG ${FUNCNAME[0]} "Vacrouter is HOMING. Discovering devices while homing completes."
                        # Using jq This loads all non-null entries of key ".t" in to a bash array
                        devs=$( ($M_SUB -h $BROKER -t 'tasmota/discovery/+/+' -W 1 | $JQ -r '.t | strings | @sh') 2>/dev/null ) 
                        read devarray <<< $devs
                        #echo "${devs[@]}"
                fi

                # Loop until we are no longer reporting "HOME" on the serial port
                until [[ ${LASTLINE:0:4} != "HOME" ]]
                do
                        LASTLINE=$( cat $TMP_LASTLINE )
                done
                INIT_WAIT_HOME=0
        fi

        if [[ ${LASTLINE:0:7} == "OK PPOS" ]]; then
                #0   1     2  3    4
                #OK PPOS: -1 CPOS: 2
                read -ra LASTLINEARRAY <<< $LASTLINE
                VACR_STATE=$(echo ${LASTLINEARRAY[0]})
                PPOS=$(echo ${LASTLINEARRAY[2]})
                VACR_CPOS=$(echo ${LASTLINEARRAY[4]})
                # LOG  ${FUNCNAME[0]} "VACR_STATE: $VACR_STATE / VACR_CPOS: $VACR_CPOS"
                MSG_PUBLISH $VACR_ST_TOPIC $VACR_STATE
                MSG_PUBLISH $VACR_CPOS_TOPIC $VACR_CPOS
        fi

        
        if [ "$OLDLASTLINE" != "$LASTLINE" ]; then
                LOG  ${FUNCNAME[0]} "$LASTLINE"
        fi      
        OLDLASTLINE="$LASTLINE"
}


### EVENT CONTROLS

vacuum_CONTROL() {
                VAC_SWITCH=$1
                if [ ! -z $2 ]; then
                        LOG ${FUNCNAME[0]} "Adjusting vacuum delay to $2 seconds"
                        VAC_DELAY=$2
                else
                        VAC_DELAY=$VAC_DELAY_DEF
                fi

                if ! [[ "$VAC_SWITCH" =~ ^(ON|OFF)$ ]]; then
                        LOG ${FUNCNAME[0]} "$VAC_SWITCH is not a valid value for vacuum_CONTROL"

                else
                        LOG ${FUNCNAME[0]} "$DEVICE is turning $VAC_SWITCH the vacuum"
                        if [ "$VAC_SWITCH" == "OFF" ]; then
                                vacuum_POWER
                        fi

                        if [ "$VAC_SWITCH" == "ON" ]; then
                                vacuum_POWER
                        fi
                fi
}

cnc_POWER() {
        if [ "$TOPIC_MSG" == "ON" ]; then
                echo "MOVE GOCNC" > $CONSOLE
                LOG ${FUNCNAME[0]} "Sent MOVE GOCNC command to Arduino. TOPIC_MSG = $TOPIC_MSG"
                SERIAL
        fi

        if [[ "$TOPIC_MSG" =~ ^(ON|OFF)$ ]]; then
                vacuum_CONTROL $TOPIC_MSG
        else
                LOG ${FUNCNAME[0]} "Invalid message (not ON or OFF), TOPIC_MSG = $TOPIC_MSG"
        fi
}

chopsaw_POWER() {
        if [ "$TOPIC_MSG" == "ON" ]; then
                echo "MOVE GOCHOPSAW" > $CONSOLE
                LOG ${FUNCNAME[0]} "Sent MOVE GOCNC command to Arduino. TOPIC_MSG = $TOPIC_MSG"
                SERIAL
        fi

        if [[ "$TOPIC_MSG" =~ ^(ON|OFF)$ ]]; then
                vacuum_CONTROL $TOPIC_MSG
        else
                LOG ${FUNCNAME[0]} "Invalid message (not ON or OFF), TOPIC_MSG = $TOPIC_MSG"
        fi
}

workbench_POWER() {
        if [ "$TOPIC_MSG" == "ON" ]; then
                echo "MOVE GOWORKBENCH" > $CONSOLE
                LOG ${FUNCNAME[0]} "Sent MOVE GOCNC command to Arduino. TOPIC_MSG = $TOPIC_MSG"
                SERIAL
        fi

        if [[ "$TOPIC_MSG" =~ ^(ON|OFF)$ ]]; then
                vacuum_CONTROL $TOPIC_MSG
        else
                LOG ${FUNCNAME[0]} "Invalid message (not ON or OFF), TOPIC_MSG = $TOPIC_MSG"
        fi
}

vacuum_POWER() {
        if [ "$TOPIC_MSG" = "ON" ]; then
                LOG ${FUNCNAME[0]} "$DEVICE turned vacuum $VAC_SWITCH"
                MSG_PUBLISH $VAC_POWER_CMD $VAC_SWITCH

                if [ "$VAC_STATE" = "OFF" ]; then
                        TOPIC[1]=vacuum
                fi

        elif [ "$TOPIC_MSG" = "OFF" ]; then
                LOG ${FUNCNAME[0]} "Clearing vacuum line for $VAC_DELAY seconds"
                # Push the sleep and vacuum off to the background so they don't block
                sleep $VAC_DELAY && MSG_PUBLISH $VAC_POWER_CMD $VAC_SWITCH 
                echo "MOVE GOCHOPSAW" > $CONSOLE
                SERIAL
        fi
}

### BEGIN MAIN ###

INIT

while :
do
        MON_TOPIC $TOPIC_POWER

        if [ "$DEVICE" != "" ]; then
                case $DEVICE in
                        cnc )
                                if [ "$DEVICE_VAR" == "POWER" ]; then
                                        cnc_POWER
                                        SERIAL
                                fi
                        ;;

                        chopsaw )
                                if [ "$DEVICE_VAR" == "POWER" ]; then
                                        chopsaw_POWER
                                        SERIAL
                                fi
                        ;;

                        workbench )
                                if [ "$DEVICE_VAR" == "POWER" ]; then
                                        workbench_POWER
                                        SERIAL
                                fi
                        ;;

                vacuum )
                                SERIAL
                ;;

                        * )
                                LOG ${FUNCNAME[0]} "CASE: ${RESPONSE[0]} ${RESPONSE[1]} has no rules, fell through to wildcard."
                        ;;
                esac
        fi
        SERIAL
        $SLEEP 3        # To allow the arm to finish moving and report so we receive it before the blocking MQTT subscription holds for a new msg
        SERIAL
done
