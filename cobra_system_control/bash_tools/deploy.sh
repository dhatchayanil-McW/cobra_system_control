# This script, in order:
# 1. shuts down existing services
# 2.

# stop any existing services
systemctl stop remote

# Turn off the other rails just in case
gpioset 4 17=0
gpioset 4 18=0

# free the I2C bus just in case
gpioset 4 19=0
gpioset 3 19=0
gpioset 3 19=1
gpioset 4 19=1

# create the /run/lumotive folder
sudo mkdir -p -m 777 /run/lumotive
sudo chmod -R 777 /run/lumotive

# start the cobra hosting for development
python3.10 ../boot_scripts/host_cobra.py
