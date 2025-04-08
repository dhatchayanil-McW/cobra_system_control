#!/bin/sh

sudo cp ./systemd_services/remote.service /lib/systemd/system/remote.service
sudo cp ./systemd_services/monitor.service /lib/systemd/system/monitor.service
sudo cp ./run_remote /usr/bin
sudo cp ./run_monitor /usr/bin
sudo systemctl daemon-reload
sudo systemctl start remote.service
sudo systemctl enable remote.service
sudo systemctl start monitor.service
sudo systemctl enable monitor.service
