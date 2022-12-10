#! /bin/bash

# Script to remove gz and ros processes
killall -9 gzserver
killall -9 gzclient
killall -9 roscore
killall -9 rosmaster