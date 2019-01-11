#!/bin/bash
# Script for connecting QGC to a remote server running the PX4 SITL simulation. 
# 
# Sources used: 
# https://gist.github.com/scy/6781836
# https://www.quora.com/How-do-you-forward-UDP-packets-through-an-SSH-tunnel
# 
# Script created by ThunderFly s.r.o 

usage="
$(basename "$0") [-h] [SERVER_NAME]
 
A script to setup communication tunnel from remote simulation server to local QGC instance.

SERVER_NAME the address of simulation server IP or its hostname
"

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters"
    echo "$usage" >&2
    exit 1
fi

server_name=$1

## create named pipe on remote machine (delete a named pipe if it already exists)
ssh $server_name rm /tmp/udp2tcp
ssh $server_name mkfifo /tmp/udp2tcp

## create named pipe on local machine (delete a named file that already exists)
rm /tmp/tcp2udp
mkfifo /tmp/tcp2udp

## opens tunnel from remote machine to local machine
echo "opening connection from remote server."
ssh -C -fR 14551:localhost:14551 $server_name sleep 10

## start translation of UDP packets to TCP packets on local machine side.
echo "starting local connection to QGC."
netcat -lvp 14551 < /tmp/tcp2udp | netcat -uv localhost 14550 > /tmp/tcp2udp &

# start translation of remote UDP packets to TCP packets transferred trought tunnel
echo "Connecting the simulator to the tunnel"
ssh $server_name ' 
netcat -lvup 14550 < /tmp/udp2tcp | netcat -v localhost 14551 > /tmp/udp2tcp
'


