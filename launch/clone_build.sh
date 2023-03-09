#!/bin/bash

/usr/bin/expect <<EOD
spawn ssh ubuntu@192.168.0.165
expect "Password"
send "turtlebot\n"
expect eof
EOD
