## Onboard controller

The controller was tested on a RaspberryPi 4 4GB, Raspbian, 32GB microSD-Card

# Raspbian installation
Install the rasbian image on a microSD-Card
```
umount /dev/sdX1
dd bs=4M if=2019-09-26-raspbian-buster.img of=/dev/sdX conv=fsync
```

# Raspberrypi config

## Add a new user
```
add user msp
```

## Set a password
```
passwd
```

## Enable I2C BUS
```
sudo raspi-config   #enable i2c
chgrp i2c /dev/i2c-1
chmod 666 /dev/i2c-1
```

## Install tools (cmake, gdbserver, doxygen)
```
sudp apt-get update
sudo apt-get install build-essential
sudo apt-get install gcc-7 g++-7        # min version for std=c++17 <filesystem>      
sudo apt-get install cmake
sudo apt-get install gdbserver
dsudo apt-get install doxygen
```

## DJI OSDK needs 
- Create a udev file called DJIDevice.rules inside /etc/udev/rules.d/
- Add SUBSYSTEM=="usb", ATTRS{idVendor}=="2ca3", MODE="0666" to this file
- Reboot your computer

### Permissions
```
sudo usermod -a -G dialout $USER
```
### DJI App-key
- create a DJI developer account
- create a new OSDK app
- copy the key and id to a `UserConfig.txt` file
- copy UserConfig.txt to the home directory


# Installation
```
# download git repository
git clone https://gitlab.ti.bfh.ch/msp/msp-onboard.git
cd msp-onboard

# executing cmake will download necessary dependencies
mkdir _build
cd _build
cmake -DCMAKE_BUILD_TYPE=Release ..     # cmake -DCMAKE_BUILD_TYPE=Debug .. 
cd ..

# install DJI Onboard-SDK
cd Onboard-SDK
mkdir _build
cd _build
cmake ..
make install
cd ..

# install MSP application
mkdir _build
cd _build
cmake ..
make install

# copy msp systemd service
cp msp.service /lib/systemd/system/msp.service
```


# Documentation
The documentation is done by doxygen
```
gunzip doxygen-$VERSION.src.tar.gz    # uncompress the archive
tar xf doxygen-$VERSION.src.tar       # unpack it

cd doxygen-$VERSION
mkdir build
cd build

```

# App Service
```
sudo systemctl enable msp.service
sudo systemctl start msp.service
sudo systemctl daemon-reload
```

# Debug 
example with gdbserver and VS code

## On RaspberryPi
- stop the msp.service service
- build the software 
    - cmake -DCMAKE_BUILD_TYPE=Debug ..
    - make install

```
gdbserver :9999 bin/msp-onboard --debug
```

## On host
### mount rasperryPi's filesystem with sshfs

```
git clone https://gitlab.ti.bfh.ch/msp/msp-onboard.git
cd msp-onboard/remote
sshfs msp@RASPI_IP/home/msp/msp-onboard ./msp-onboard-remote
```
- open new VS Code window directory `remote`
- add the file `.vscode/launch.json`
- add the following configuration

```
{
"configurations": [    
        {
            "name": "C++ Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceRoot}/msp-onboard-remote/bin/msp-onboard",
            "miDebuggerServerAddress": "RASPI_IP:9999",
            "sourceFileMap": { "/home/msp/msp-onboard/src/":"PATH_TO_REPO/msp-onboard/remote/msp-onboard-remote/src/" },
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceRoot}",
            "environment": [],
            "externalConsole": true,
            "linux": {
              "MIMode": "gdb"
            },
            "osx": {
              "MIMode": "gdb"
            },
            "windows": {
              "MIMode": "gdb"
            }
          }
    ]
}
```





