#!/bin/sh

CHANNEL=39

make || exit -1

(make install SERIAL_PORT=/dev/cu.usbserial-DN00D34P && ./pyaci/configure_sensor.py --channel $CHANNEL -d /dev/cu.usbserial-DN00D34P 1) &

(make install SERIAL_PORT=/dev/cu.usbserial-DN00CSZ7 && ./pyaci/configure_sensor.py --channel $CHANNEL -d /dev/cu.usbserial-DN00CSZ7 2) &

(make install SERIAL_PORT=/dev/cu.usbserial-DO00C2G2 && ./pyaci/configure_sensor.py --channel $CHANNEL --no-sleeping -d /dev/cu.usbserial-DO00C2G2 3) &

wait

./pyaci/set_time.py -d /dev/cu.usbserial-DO00C2G2
