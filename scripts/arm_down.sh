#!/usr/bin/expect

set timeout 30
spawn ssh -Y pi@$env(ARM_IP)
expect "password:"
send "raspberry\r"
expect "$*"
send "cd Arm_Pi\r"
expect "$*"
send "python down.py\r"
expect "$*"
send "exit\r"
interact
