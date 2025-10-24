#!/usr/bin/env bash
idf.py -p /dev/ttyACM0 erase-flash && sleep 0.5
idf.py -p /dev/ttyACM0 build && sleep 0.5
idf.py -p /dev/ttyACM0 flash && sleep 0.5
idf.py -p /dev/ttyACM0 monitor

