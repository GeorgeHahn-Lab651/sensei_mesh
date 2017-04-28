#!/bin/sh

CHANNEL=39

make || exit -1

function program {
  if [ -c $1 ]; then
    if [ "$3" == "clock_master" ]; then
      ARGS="--no-sleeping"
    fi
    make install SERIAL_PORT=$1 && ./pyaci/configure_sensor.py --channel $CHANNEL $ARGS -d $1 $2
    if [ "$3" == "clock_master" ]; then
      ./pyaci/set_time.py -d $1
    fi
  else
    echo "Skipping $1 (not plugged in?)"
  fi
}

program /dev/cu.usbserial-DN00D34P 1 &
program /dev/cu.usbserial-DN00CSZ7 2 &
program /dev/cu.usbserial-DO00C2G2 3  &
program /dev/cu.usbserial-FTZ86FTC 12 &
program /dev/cu.usbserial-AI04QL7P 30 clock_master &
program /dev/cu.usbserial-A105RB12 31 &


wait
