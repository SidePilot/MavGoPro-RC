/*
  =================================================================================================

      MavGoProRC  (MAVLink GoPro Remote Controller)
	  GOPRO CLIENT

      Created by Tom Brereton
      http://sidepilot.net

  =================================================================================================
*/

void sendGoProKeepAlive() {
  if (pSettingsCharacteristic != nullptr) {
    pSettingsCharacteristic->writeValue({0x03, 0x5B, 0x01, 0x42}, 4);
  }
}

//GoPro BLE Command query response callback
void goproCommandNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if (pData[0] == 0x20) { //Detect if this is a hardware request response first packet. AKA Contains GoPro model name
    int startCharIndex = 0;
    boolean startChar = false;
    for (int x = 0; x < length; x++) {
      if (startChar) {
        goProModel[x - startCharIndex] = pData[x];
      }
      if (pData[x] == 0x0B) {
        startChar = true;
        startCharIndex = x + 1;
      }
    }
    DEBUG_PRINTN("GoPro Model: ", goProModel);
  } else {
    if (pData[0] == 0x80 || pData[0] == 0x81 || pData[0] == 0x82 || pData[0] == 0x83) { //If the received packet is the continuation of the GoPro Model packet, disregard.
      return;
    }
    if (pData[length - 1] == 0x02) {
        GOPRO_COMMAND command = GOPRO_COMMAND_ENUM_END;
        switch (pData[1]) {
        case 0x02:
          command = GOPRO_COMMAND_CAPTURE_MODE; break;
        case 0x01:
          command = GOPRO_COMMAND_SHUTTER; break;
        }
        DEBUG_PRINTS("Setting change rejected.\r\n");
        sendMavlinkGoProSetResponse(command, GOPRO_REQUEST_FAILED);
      }
    //Other response
    DEBUG_PRINTS("Command Response: ");
    serialPrintHex(pData, length);
    DEBUG_PRINTS("\r\n");
  }
  //Restart the GoPro heartbeat timer.
  restartGoProHeartbeat();
}

//BLE Settings query response callback
void goproSettingsNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if (length > 2) {
    if (pData[0] = 2 && pData[1] == 0x5B) {
      //Keep alive received. Do nothing
    } else {
      if (pData[length - 1] == 0x02) {
        GOPRO_COMMAND command = GOPRO_COMMAND_ENUM_END;
        switch (pData[1]) {
        case 0x02:
        case 0x03:
        case 0x04:
        case 0x86:
        case 0x39:
        case 0x7A:
        case 0x79:
        case 0x7B:
          command = GOPRO_COMMAND_VIDEO_SETTINGS; break;
        }
        DEBUG_PRINTS("Setting change rejected.\r\n");
        sendMavlinkGoProSetResponse(command, GOPRO_REQUEST_FAILED);
      }
      //Other response
      DEBUG_PRINTS("Settings Response: ");
      serialPrintHex(pData, length);
      DEBUG_PRINTS("\r\n");
    }
  }
  //Restart the GoPro heartbeat timer.
  restartGoProHeartbeat();
}

void restartGoProHeartbeat() {
  //GoPro data received, mark as connected and set last received time to now.
  isCameraConnected = true;
  cameraHeartbeatReceivedTimestamp = millis();
  cameraStatus = GOPRO_HEARTBEAT_STATUS_CONNECTED;
}

//Check the timelapse flag status of the GoPro. Needed to tell the difference between video and timelapse modes.
void getTimelapseStatus() {
  if (pQueryCharacteristic != nullptr) {
    goproNeedsTimelapseFlag = false;
    pQueryCharacteristic->writeValue({0x02, 0x13, 0x2C}, 3);
  }
}

//Register to notifications on the applicable GoPro BLE status's.
void registerGoProStatusQuery() {
  if (pQueryCharacteristic != nullptr) {
    if (!goproHasRegisteredQueryStatus) {
      DEBUG_PRINTS("Registering for Status Query Notifications\r\n");
      // 2B = 43 - Mode, 46 = 70 - Battery Level, 8 = 8 - Recording Status, 2C = 44 - Timelapse flag, 60 = 96 - Active Preset Group ID
      pQueryCharacteristic->writeValue({0x06, 0x53, 0x2B, 0x46, 0x08, 0x2C, 0x60}, 7);
    }
  }
}

void registerGoProSettingQuery() {
  if (pQueryCharacteristic != nullptr) {
      DEBUG_PRINTS("Registering for Setting Notifications\r\n");
      // 2 = 2 - Resolution, 3 = 3 - Framerate
      pQueryCharacteristic->writeValue({0x02, 0x52, 0x02, 0x03, 0x08, 0x2C, 0x60}, 7);
  }
}

//Setup variables for the GoPro status response class data class.
int gopro_accumulate_remaining = 0;
int gopro_accumulate_index = 0;
uint8_t gopro_accumulate_array[1000] = {};

bool gopro_accumulate_complete() {
  return (gopro_accumulate_index > 0 && gopro_accumulate_remaining == 0);
}

//If continuous packets are received, append them to the last received data until a full packet is received.
const void goproAccumulateData(uint8_t* pData, size_t length) {
  uint8_t CONT_MASK = 0b10000000; //Continuing data from last packet
  uint8_t HDR_MASK = 0b01100000;
  uint8_t GEN_LEN_MASK = 0b00011111;
  uint8_t EXT_13_BYTE0_MASK = 0b00011111;

  uint8_t GENERAL = 0b00;
  uint8_t EXT_13 = 0b01;
  uint8_t EXT_16 = 0b10;
  uint8_t RESERVED = 0b11;

  int x = 0;
  if (pData[0] & CONT_MASK) {
    //Is continuing to append data...
    x = 1;
  } else {
    //If first data packet received.
    gopro_accumulate_index = 0;
    memset(gopro_accumulate_array, 0, sizeof(gopro_accumulate_array));
    uint8_t hdr = ((pData[0] & HDR_MASK) >> 5);
    if (hdr == GENERAL) {
      gopro_accumulate_remaining = pData[0] & GEN_LEN_MASK;
      x = 1;
    } else if (hdr == EXT_13) {
      gopro_accumulate_remaining = ((pData[0] & EXT_13_BYTE0_MASK) << 8) + pData[1];
      x = 2;
    } else if (hdr == EXT_16) {
      gopro_accumulate_remaining = (pData[1] << 8) + pData[2];
      x = 3;
    }
  }
  for (int i = x; i < length; i++) {
    gopro_accumulate_array[gopro_accumulate_index + i - x] = pData[i];
  }

  gopro_accumulate_index += (length - x);
  gopro_accumulate_remaining -= (length - x);
}

//BLE Query response callback
const void goproQueryNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  uint8_t query_len = pData[0];
  if (!(query_len & 0b10000000)) {
    uint8_t query_id = pData[1];
    uint8_t query_val = pData[2];
    if (query_id == 83) { //Registered for Status successfully
      goproHasRegisteredQueryStatus = true;
      DEBUG_PRINTS("Registered for Status Query Notifications\r\n");
    }
  }

  restartGoProHeartbeat();

  goproAccumulateData(pData, length); //Add the data to the existing data packets if applicable

  if (gopro_accumulate_complete()) { //If all data has been received, parse.
    int x = 2;
    while (x < gopro_accumulate_index) {
      uint8_t status_id = gopro_accumulate_array[x];
      uint8_t status_len = gopro_accumulate_array[x + 1];
      uint8_t status_val[status_len] = {};
      for (int i = 0; i < status_len; i++) {
        status_val[i] = gopro_accumulate_array[x + i + 2];
      }
      goproReceivedStatusResponse(status_id, status_val, status_len); //Received valid query response.
      x += status_len + 2;
    }
  }
}

void goproReceivedStatusResponse(uint8_t status_id, uint8_t* status_value, uint8_t status_length) {  
  if (goproQueryPacket != 0 && goproQueryPacket == status_id) { // If user is using /query endpoint, return result.
    String response = String(goproQueryPacket) + ": ";
    for (int i = 0; i < status_length; i++) {
      response += String(status_value[i]);
      if (i != status_length - 1) {
        response += ", ";
      }
    }
    goproQueryPacket = 0;
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", response);
  }

  if (status_id == 43) { //Current Mode
    if (status_value[0] == 0) { //Video mode, now get timelapse flag before setting mode.
      cameraCurrentMode = GOPRO_CAPTURE_MODE_UNKNOWN;
      goproNeedsTimelapseFlag = true;
    } else if (status_value[0] == 1 && cameraCurrentMode != GOPRO_CAPTURE_MODE_PHOTO) { //Photo
      DEBUG_PRINTS("Got Mode = Photo\r\n");
      cameraCurrentMode = GOPRO_CAPTURE_MODE_PHOTO;
    }
  } else if (status_id == 8) { //Current recording status
    if (status_value[0] == 0 && isCameraRecording) { //Not Recording
      DEBUG_PRINTS("Got Recording = false\r\n");
      isCameraRecording = false;
    } else if (status_value[0] == 1 && !isCameraRecording && cameraCurrentMode != GOPRO_CAPTURE_MODE_PHOTO) { //Recording (don't trigger if in photo mode)
      DEBUG_PRINTS("Got Recording = true\r\n");
      isCameraRecording = true;
    }
  } else if (status_id == 70) { //Battery level
    cameraBatteryLevel = status_value[0];
    DEBUG_PRINTN("Got Battery Level = ", cameraBatteryLevel);
  } else if (status_id == 44) { //Timelapse
    if (cameraCurrentMode != GOPRO_CAPTURE_MODE_PHOTO) {
      if (status_value[0] == 1 && cameraCurrentMode != GOPRO_CAPTURE_MODE_TIME_LAPSE) {
        DEBUG_PRINTS("Got Mode = Timelapse\r\n");
        cameraCurrentMode = GOPRO_CAPTURE_MODE_TIME_LAPSE;
      } else if (cameraCurrentMode != GOPRO_CAPTURE_MODE_VIDEO) {
        DEBUG_PRINTS("Got Mode = Video\r\n");
        cameraCurrentMode = GOPRO_CAPTURE_MODE_VIDEO;
      }
    }
  } else if (status_id == 19) { //Pairing status
    if ((status_value[0] == 0 || status_value[0] == 4) && !paired) {
      DEBUG_PRINTS("GoPro Paired\r\n");
      paired = true;
    } else if (!paired) {
      DEBUG_PRINTS("GoPro Set to pairing\r\n");
      pairing = true;
    }
  } else if (status_id == 96) { //Active Preset Group
    if (!isOpenGoPro) {
      DEBUG_PRINTS("GoPro Hero 9+ detected, using Preset Groups for mode detection.\r\n");
      isOpenGoPro = true;
    }
    for (int x = 0; x < status_length; x++) {
      if (status_value[x] == 233) { //233 = Photo
        DEBUG_PRINTS("Got Mode = Photo\r\n");
        cameraCurrentMode = GOPRO_CAPTURE_MODE_PHOTO;
        break;
      } else if (status_value[x] == 232) { //232 = Video
        DEBUG_PRINTS("Got Mode = Video\r\n");
        cameraCurrentMode = GOPRO_CAPTURE_MODE_VIDEO;
        break;
      } else if (status_value[x] == 234) { //234 = Timelapse
        DEBUG_PRINTS("Got Mode = TimeLapse\r\n");
        cameraCurrentMode = GOPRO_CAPTURE_MODE_TIME_LAPSE;
        break;
      }
    }
  }
  sendMavlinkGoProHeartbeat(); //Send a MAVLink GoPro heartbeat containing updated data.
}

//If pairing to GoPro, connect to it's WiFi network and send the pair URI request to complete pairing.
void confirmGoProPairing(const char* wifiSSID, const char* wifiPassword) {
  DEBUG_PRINT("SSID: ", wifiSSID);
  DEBUG_PRINTN(", Password: ", wifiPassword);
  WiFi.begin(wifiSSID, wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    updateLEDs();
    DEBUG_PRINTS(".");
    if (!isCameraConnected) {
      return;
    }
  }

  WiFiClient client;
  const int httpPort = 80;
  const char* hostIP = "10.5.5.9";
  if (!client.connect(hostIP, httpPort)) {
    DEBUG_PRINTS("Wifi connection failed\r\n");
    return;
  }

  // We now create a URI for the request
  String url = "/gp/gpControl/command/wireless/pair/complete?success=1&deviceName=MAV-GPRC";

  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + hostIP + "\r\n" +
               "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      DEBUG_PRINTS(">>> Client Timeout !");
      client.stop();
      return;
    }
  }
  DEBUG_PRINTS("\r\n");
  pairing = false;
  delay(200);
  return;
}
