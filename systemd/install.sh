#!/bin/sh
# A shell script to start the GoPiGo robot and web server under Systemd
sudo cp *.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable robot
sudo systemctl enable webcontrol
sudo systemctl start robot
sudo systemctl start webcontrol
