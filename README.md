# MarioKar

## Development Setup

### Beginning Docker Container
#### **For Windows**
Before starting the docker container, the .devcontainer.json file requires that the USB device is connected the Linux distribution running on WSL 2. This will enable us to flash the ESP32 in the container. 

#### Connect USB Device
Prerequisites (can also refer to https://learn.microsoft.com/en-us/windows/wsl/connect-usb): 
* WSL is installed 
* Install USBIPD-WIN

Instructions to be done every session: 
1. Connect USB device to computer and list device in Windows Powershell in *administrator* mode: \
`usbipd list`

2. Run the command below with the bus ID of the ESP32 device: \
`usbipd bind --busid 4-4`

3. Attach the USB device: \
`usbipd attach --wsl --busid <busid>`

4. Open the **MarioKar** repo and `ctrl+shift+P` and type `Dev Containers: Rebuild and Reopen in Container`

### Build and Flash ESP32S3
If the ESP-IDF extension doesn't work when building or flashing, open a terminal within VSCode and follow the instructions below. 

1. `source /opt/esp/idf/export.sh`
2. Navigate to the project directory, and configure the target by running \
`idf.py set-target esp32s3`
3. Build the project with \
`idf.py build`
4. Flash the project to the board with \
`idf.py flash`

* After flashing, you can open the serial monitor \
`idf.py monitor`\
and close serial monitor with \
`ctrl+]`

### Alternative Setup (macOS/Linux)
```bash
# Install prerequisites
brew install cmake ninja dfu-util

# Install ESP-IDF tools
brew install espressif/idf/esp-idf-tools

# Setup ESP-IDF environment
/opt/esp/idf/install.sh
source /opt/esp/idf/export.sh

# Create and build project
cd /workspaces/MarioKar
idf.py create-project hello_world
cd hello_world
idf.py set-target esp32s3      # or esp32s3/esp32c3/etc if that's your chip
idf.py build
idf.py flash
```
