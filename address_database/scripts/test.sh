#!/bin/bash

LOG_FILE=logs.txt
DATABASE_FILE=database.csv

if test -f "$LOG_FILE" ; then
echo "$LOG_FILE exists"
else
echo "$LOG_FILE does not exist, creating one!"
touch $LOG_FILE
fi

timestamp() {
    # time=$(date +%Y-%m-%d_%H-%M-%S)
    date -u
}
timestamp
echo $time

sleep 5


log()
{
#	TODO Write activities to log files along with timestamp, pass argument as a string
	
	echo "`date -u`" > $LOG_FILE
}

log