#!/bin/bash
sudo chrt -f 99 ptp4l -f /etc/linuxptp/ptp4l.conf -i eth0 -m 
