# MAVLink GoPro Remote Control

![License](https://img.shields.io/badge/license-GPL--3.0-blue.svg)

The MAVLink GoPro Remote Control (MavGoPro_RC) allows you to remotely control a GoPro HERO camera using MAVLink commands, such as those sent from a ground control station (GCS) or autopilot system like ArduPilot or PX4. This system is designed for integration with drones or robotic platforms, enabling automation and remote control of GoPro cameras for aerial photography, inspections, and more.

You can purchase the hardware at the [SidePilot Store](https://store.sidepilot.net), or build your own!

## Features

- Supports GoPro HERO 5/6/7/8/9/10 Black cameras via BLE (10+ have not been tested but should work)
- Connects to MAVLink telemetry via UART
- Automatically parses MAVLink messages
- Allows camera control via MAV_CMD_DO_DIGICAM_CONTROL and MAV_CMD_IMAGE_START_CAPTURE

## Hardware Requirements

- ESP32-based MavGoPro_RC board (custom PCB with ESP32 WROOM module)
- Compatible GoPro HERO 5/6/7/8/9/10 Black  (10+ have not been tested but should work)
- MAVLink-compatible flight controller or GCS
- 5V DC power source (e.g., drone power rail or USB)

## Wiring Overview

- Connect MAVLink TX/RX from flight controller to ESP32
- Power the board with regulated 5V

## Software Installation

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/)
- ESP32 board definitions installed
- USB-to-Serial adapter (for initial flashing if needed)

### Manually Generate and Flash the Firmware

1. Clone this repository.

2. Open the project in Arduino IDE.

3. Modify the Global_Variables.h file to work with your ESP32 board.

4. Flash the firmware to the board.

5. After flashing, the ESP32 will reboot and start broadcasting an access point (for setup), or attempt to connect to your saved GoPro WiFi network.

## Configuration

After flashing:

1. Ensure your MAVLink parameters are set to:
   ```
   SERIALX_PROTOCOL	2
   SERIALX_BAUD		57
   ```
2. Turn off your flight controller and plug the Mav GoPro Remote Controller into the desired serial port using the included cable.

3. Turn on your flight controller and GoPro

4. On your GoPro go to Preferences -> Connections -> WiFi Band and ensure it is set to 2.4GHz. Then go to Preferences -> Connections -> Connect Device and select GoPro App'.

5. Pairing will begin and it will pick up the MGRC automatically. When pairing, the red LED will flash fast (If not flashing fast/paired within 20 seconds, press the MGRC reset button).

6. The red blinking light will turn solid when it has successfully connected to the camera. You can now control the shutter (take video, or start/stop recording) via a Trigger Camera action in Mission Planner.
   
7. When rebooting, the GoPro should automatically reconnect to the device. If it doesnt automatically connect, you may need to give the RST button a tap.

## Web Interface

The MGRC provides a web interface where you can check the MAVLink status, as well as GoPro information such as its current mode and battery status.

1. Press the 'OPT" button to start the WiFi network and connect to it using your computer or smartphone.
   ```
   SSID: MAV-GOPRO-RC
   Password: MavlinkGoProRC
   ```

2. Open a browser and navigate to `https://10.71.79.1/info` to access the information page.

## Firmware Update

The MGRC firmware is updatable via a web upload.

1. Generate a firmware update .bin file or download from here.

2. Ensure the GoPro is turned and remove the battery (to avoid it turning back on automatically).

3. Press the 'OPT" button to start the WiFi network and connect to it using your computer or smartphone.
   ```
   SSID: MAV-GOPRO-RC
   Password: MavlinkGoProRC
   ```

4. Open a browser and navigate to `https://10.71.79.1/update` to access the OTA update page.

5. Tap on ‘Browse’ and select the .bin firmware file.

6. Tap ‘Update’ and wait. The Green and Red LED’s will flash while updating. The update process usually takes 20-30 seconds and the MGRC will automatically reboot when completed. The Web interface may not update upon reboot.

7. Verify the installation was successful by navigating to http://10.71.79.1/info and check the revision number.

## MAVLink Commands

The following MAVLink commands are supported:

- `MAV_CMD_DO_DIGICAM_CONTROL`
  - Trigger single photos or change capture mode
- `MAV_CMD_IMAGE_START_CAPTURE`
  - Start/stop video recording

You can send these via MAVProxy, Mission Planner, QGroundControl, or programmatically from onboard companion computers.

## Contributing

Pull requests are welcome! Feel free to fork the project and submit improvements.

## License

This project is licensed under the GNU General Public License v3.0 — see the [LICENSE](./LICENSE) file for details.

---

Need help? Open an issue or start a discussion!
