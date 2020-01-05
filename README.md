## Onboard controller


# Install
```
# download git repository
git clone https://gitlab.ti.bfh.ch/msp/msp-onboard.git
cd msp-onboard

# executing cmake will download necessary dependencies
mkdir _build
cd _build
cmake ..
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

# copy and start msp systemd service
cp msp.service /lib/systemd/system/msp.service
sudo systemctl enable msp.service
sudo systemctl start msp.service
sudo systemctl daemon-reload
```