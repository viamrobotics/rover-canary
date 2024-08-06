#!/bin/bash

# ensure neither viam-canary or viam-server are running
sudo systemctl stop viam-server

# install the RDK server
sudo curl "https://storage.googleapis.com/packages.viam.com/apps/viam-server/viam-server-latest-$(uname -m).AppImage" -o viam-server
sudo chmod 755 viam-server
sudo ./viam-server --aix-install

# start viam-canary service
sudo systemctl start viam-server
sleep 120 # allow the server enough time to start

# run the rover canary tests
cd /home/rover-canary/rover-canary
sudo go run main.go

