/*
  =====================================================================================================================
      
	  MavGoProRC  (MAVLink GoPro Remote Controller)
	  MAIN INITIALISER

      Created by Tom Brereton
      http://sidepilot.net
	  
  =====================================================================================================================
*/

#include "BLEDevice.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include "mavlink/ardupilotmega/mavlink.h"
#include <Preferences.h>
#include "Global_Variables.h"

//--------------------- BLE Client Classes ------------------------------------------------------------

static BLERemoteCharacteristic* pWifiSSIDCharacteristic;
static BLERemoteCharacteristic* pWifiPasswordCharacteristic;
static BLERemoteCharacteristic* pWifiPowerCharacteristic;
static BLERemoteCharacteristic* pWifiStateCharacteristic;
static BLERemoteCharacteristic* pSettingsCharacteristic;
static BLERemoteCharacteristic* pSettingsResponseCharacteristic;
static BLERemoteCharacteristic* pCommandCharacteristic;
static BLERemoteCharacteristic* pCommandResponseCharacteristic;
static BLERemoteCharacteristic* pQueryResponseCharacteristic;
static BLERemoteCharacteristic* pQueryCharacteristic;

class CameraBLEClientCallback : public BLEClientCallbacks  {
    void onDisconnect(BLEClient* pclient) {
      isCameraConnected = false;
      DEBUG_PRINTS("BLE Disconnected\r\n");
      pairing = false;
      doScan = true;
      paired = false;
      if (cameraType == GoPro) {
        goProModel[0] = 0;
        isOpenGoPro = false;
        goproQueryPacket = 0;
      }
      isCameraRecording = false;
      cameraCurrentMode = GOPRO_CAPTURE_MODE_UNKNOWN;
      cameraStatus = GOPRO_HEARTBEAT_STATUS_DISCONNECTED;
    }

    void onConnect(BLEClient* pClient) {
      DEBUG_PRINTS("BLE Connected\r\n");
      doScan = false;
    }
};

static CameraBLEClientCallback cameraBLEClientCallback;
static BLEClient* cameraBLEClient;

bool connectToBLE() {
  DEBUG_PRINT("Forming a connection to ", cameraBLEAdvertisedDevice->getAddress().toString().c_str());
  DEBUG_PRINTN("\r\nMy address: ", BLEDevice::getAddress().toString().c_str());
  if (cameraBLEClient == nullptr) {
    cameraBLEClient = BLEDevice::createClient();
    DEBUG_PRINTS("BLE Client client created\r\n");
    cameraBLEClient->setClientCallbacks(&cameraBLEClientCallback);
    cameraBLEClient->connect(cameraBLEAdvertisedDevice);
  }

  int retryCount = 5;
  while (!cameraBLEClient->isConnected()) {
    if (retryCount <= 0) {
      return false;
    } else {
      DEBUG_PRINTS("Retrying BLE connection...\r\n");
      delay(1000);
    }
    BLEDevice::getScan()->stop();
    cameraBLEClient->disconnect();
    delay(500);
    cameraBLEClient->connect(cameraBLEAdvertisedDevice);
    --retryCount;
  }

  BLERemoteService* pWifiService = cameraBLEClient->getService(wifiUUID);
  if (pWifiService == nullptr) {
    DEBUG_PRINTS("Failed to get wifi service\r\n");
    cameraBLEClient->disconnect();
    return false;
  }

  pWifiSSIDCharacteristic = pWifiService->getCharacteristic(wifiSSIDUUID);
  pWifiPasswordCharacteristic = pWifiService->getCharacteristic(wifiPasswordUUID);
  pWifiPowerCharacteristic = pWifiService->getCharacteristic(wifiPowerUUID);
  pWifiStateCharacteristic = pWifiService->getCharacteristic(wifiStateUUID);

  if (pWifiSSIDCharacteristic == nullptr || pWifiPasswordCharacteristic == nullptr || pWifiPowerCharacteristic == nullptr || pWifiStateCharacteristic == nullptr) {
    DEBUG_PRINTS("Failed to get GoPro WiFi characteristics, disconnecting\r\n");
    cameraBLEClient->disconnect();
    return false;
  }

  BLERemoteService* pControlService = cameraBLEClient->getService(serviceUUID);
  if (pControlService == nullptr) {
    DEBUG_PRINTN("Failed to find service UUID: ", serviceUUID.toString().c_str());
    cameraBLEClient->disconnect();
    return false;
  }
  DEBUG_PRINTS("Connected to Control & Query Service\r\n");

  pSettingsCharacteristic = pControlService->getCharacteristic(settingsUUID);
  pSettingsResponseCharacteristic = pControlService->getCharacteristic(settingsResponseUUID);
  pCommandCharacteristic = pControlService->getCharacteristic(commandUUID);
  pCommandResponseCharacteristic = pControlService->getCharacteristic(commandResponseUUID);
  pQueryCharacteristic = pControlService->getCharacteristic(queryUUID);
  pQueryResponseCharacteristic = pControlService->getCharacteristic(queryResponseUUID);

  if (pSettingsCharacteristic == nullptr || pSettingsResponseCharacteristic == nullptr || pCommandCharacteristic == nullptr || pCommandResponseCharacteristic == nullptr || pQueryCharacteristic == nullptr || pQueryResponseCharacteristic == nullptr) {
    cameraBLEClient->disconnect();
    return false;
  }

  if (pCommandResponseCharacteristic->canNotify()) {
    pCommandResponseCharacteristic->registerForNotify(goproCommandNotifyCallback);
    DEBUG_PRINTS("Registered for Command Responses\r\n");
  }

  if (pSettingsResponseCharacteristic->canNotify()) {
    pSettingsResponseCharacteristic->registerForNotify(goproSettingsNotifyCallback);
    DEBUG_PRINTS("Registered for Settings Responses\r\n");
  }

  if (pQueryResponseCharacteristic->canNotify()) {
    pQueryResponseCharacteristic->registerForNotify(goproQueryNotifyCallback);
    DEBUG_PRINTS("Registered for Query Responses\r\n");
  }
  return true;
}

class CameraBLEAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      vTaskDelay(10);
      DEBUG_PRINTS(".");
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
        if (cameraType == GoPro) {
          if (savedGoProName != "") {
            if (savedGoProName.c_str() != advertisedDevice.getName().c_str()) {
              DEBUG_PRINT("Advertised device ", advertisedDevice.getName().c_str());
              DEBUG_PRINTN(" does not match saved value ", savedGoProName.c_str());
              return;
            }
          }
        }
        DEBUG_PRINTN("\r\nConnecting to ", advertisedDevice.getName().c_str());
        BLEDevice::getScan()->stop();
        cameraBLEAdvertisedDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
      }
    }
};

//--------------------- Main ------------------------------------------------------------

void setup() {
  // Give a new BLE MAC address. Needs to be done for GoPro to save the correct MAC for pairing.
  uint8_t newMac[6] = {0xa4, 0xe7, 0xe4, 0x22, 0x2e, 0xb3};
  esp_base_mac_addr_set(newMac);

  //Set LED/Button pins
  pinMode(BLE_LED, OUTPUT);
  pinMode(MAV_LED, OUTPUT);
  pinMode(OPT_BTN, INPUT_PULLUP);
  delay(100);
  digitalWrite(BLE_LED, HIGH);
  //Start Serials
  Serial.begin(57600);
  delay(100);
  Serial.flush();
  Serial2.begin(9600, SERIAL_8N1, DBG_RX, DBG_TX);
  Serial2.setDebugOutput(true);
  Serial2.flush();
  Serial2.print("MAVLink GoPro RC Debugger");

  //Create second core LED flash task.
  xTaskCreatePinnedToCore(
    LEDFlashTaskLoop, /* Function to implement the task */
    "LED_FLASH_TASK", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    1,  /* Priority of the task */
    &LED_FLASH_TASK,  /* Task handle. */
    0); /* Core where the task should run */

#if SOLO
  setupWebServer();
#else
  //Set WiFi to Station to connect to GoPro network
  WiFi.mode(WIFI_STA);
  delay(100);
  //Set Minimum WiFi power.
  WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);
#if DEBUG
  //If DEBUG enabled, start the web UI.
  DEBUG_PRINTS(" - ENABLED\r\n");
  setupWebServer();
#else
  Serial2.println(" - DISABLED");
#endif
#endif

  //Open persistant memory data and read
  prefs.begin("mavcam");
  if (cameraType == GoPro) {
    savedGoProName = prefs.getString("camera_name", ""); //Default = ""
  }
  MAVLinkSystemID = prefs.getInt("sys_id", 1); //Default = 1
#if SOLO
  MAVLinkComponentID = prefs.getInt("comp_id", MAV_COMP_ID_GIMBAL); //Default = 154
  MAVLinkProtocol = prefs.getInt("mav_protocol", 1); //Default = 1
#else
  MAVLinkComponentID = prefs.getInt("comp_id", MAV_COMP_ID_CAMERA); //Default = 100
  MAVLinkProtocol = prefs.getInt("mav_protocol", 2); //Default = 2
#endif
  updateMAVLinkProtocol();
  //Initialise BLE device and start scanning
#if SOLO
  BLEDevice::init("SOLO-GPRC");
#else
  BLEDevice::init("MAV-GPRC");
#endif
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new CameraBLEAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(45);
  pBLEScan->setWindow(15);
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT); //GoPro needs encrypt to connect
  BLEDevice::setPower(ESP_PWR_LVL_N11); //Set minimum power
  pBLEScan->setActiveScan(true);
  DEBUG_PRINTS("Scanning BLE...\r\n");
  pBLEScan->start(5);
}

void updateMAVLinkProtocol() {
  mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);
  if (MAVLinkProtocol == 1) {
    chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  } else {
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
  }
  DEBUG_PRINTN("Set MAVLink Protocol: ", MAVLinkProtocol);
}

void loop() {
#if DEBUG
  debug_receive(); //Check debug serial for commands.
#endif

  //Check serial for mavlink data
  mavlink_receive();

  if (doConnect == true) { //Connect to GoPro
    if (connectToBLE()) {
      DEBUG_PRINTS("Connected to GoPro BLE\r\n");
      isCameraConnected = true;
      cameraHeartbeatReceivedTimestamp = millis();
    } else {
      DEBUG_PRINTS("Failed to connect to the GoPro BLE\r\n");
      isCameraConnected = false;
    }
    doConnect = false;
  }

  //Record the current boot time.
  unsigned long currentMillis = millis();

  if (currentMillis - cameraHeartbeatReceivedTimestamp >= (cameraKeepAliveInterval + 10000) && isCameraConnected) { //If no message received in 60 + 10 seconds
    DEBUG_PRINTS("GoPro Heartbeat timed out\r\n");
    sendGoProKeepAlive(); //Manually send the keep alive as last ditch effort to maintain connection
    isCameraConnected = false;
  }

  if (currentMillis - cameraKeepAliveSentTimestamp >= cameraKeepAliveInterval && isCameraConnected) { //Send keep alive every 60s
    sendGoProKeepAlive();
    cameraKeepAliveSentTimestamp = currentMillis;
  }

  if (goproQueryPacket != 0 && currentMillis - goproQuerySentTimestamp >= 3000) { //Wait 3 seconds for query response otherwise cancel.
    goproQueryPacket = 0;
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "No Response, try again.");
  }

  //If no MAVLink message received for 10s, set connected flag to false.
  if (currentMillis - MAVLinkHeartbeatReceivedTimestamp >= 10000 && isMavlinkConnected) {
    DEBUG_PRINTS("MAVLink Heartbeat timed out\r\n");
    isMavlinkConnected = false;
  }

  //Every interval, send a MAVLink heartbeat and MAVLink GoPro heartbeat.
  if (currentMillis - MAVLinkLastSentTimestamp >= MAVLink_interval) {
    sendMavlinkHeartbeat();
    sendMavlinkGoProHeartbeat();
    //Reset mavlink timeout
    MAVLinkLastSentTimestamp = currentMillis;
  }

  if (isIntervalCaptureEnabled) {
    if (currentMillis - intervalLastTimestamp >= captureInterval) {
      setShutter(true);
      intervalLastTimestamp = currentMillis;
      captureIndex += 1;
      if (captureTotal != 0 && captureIndex >= captureTotal) {
        isIntervalCaptureEnabled = false;
      }
    }
  }

  if (isCameraConnected) { //Camera is connected
    if (cameraType == GoPro) {
      if (!paired) { //GoPro is not paired
        if (pairing) { //GoPro is in pairing mode
          if (pWifiSSIDCharacteristic->canRead() && pWifiPasswordCharacteristic->canRead()) {
            std::string ssidStr = pWifiSSIDCharacteristic->readValue(); //Get GoPro SSID
            const char* wifiSSID = ssidStr.c_str();
            std::string passStr = pWifiPasswordCharacteristic->readValue(); //Get GoPro Password
            const char* wifiPassword = passStr.c_str();
            confirmGoProPairing(wifiSSID, wifiPassword); //Connect to GoPro WiFi.
            delay(50);
          }
        } else { //GoPro is NOT in pairing mode, set to pairing mode.
          static uint8_t pairAttempt = 0;
          pairAttempt += 1;
          if (pairAttempt > 5) {
            if (cameraBLEClient) { //If more than 5 attempts at pairing mode, disconnect BLE.
              cameraBLEClient->disconnect();
            }
            pairAttempt = 0;
            delay(50);
          } else {
            DEBUG_PRINTS("Getting paired status...\r\n");
            pQueryCharacteristic->writeValue({0x02, 0x13, 0x13}, 3); //Set to pairing mode
            delay(50);
          }
        }
      } else { //Is paired to GoPro.
        //Is paired
        if (!goproHasRegisteredQueryStatus) {
          registerGoProStatusQuery(); // Register for GoPro Status updates.
        }
        if (goProModel[0] == 0) {
          getGoProModel(); //Query the GoPro Model
        }
        if (goproNeedsTimelapseFlag) {
          getTimelapseStatus(); //Check if the timelapse flag is set (For differenciating video/timelapse modes).
        }
      }
    }
  } else if (doScan && !disableBLE && !isCameraPoweredOff) { //No Camera is connected, start scanning.
    if (cameraType == GoPro) {
      BLEDevice::getScan()->start(1);
    }
  }

#if SOLO == 0
  int optBtnPressed = digitalRead(OPT_BTN); //Read button press state.
  if (OPT_BTN_PREV_STATE != optBtnPressed) {
    OPT_BTN_PREV_STATE = optBtnPressed;
    if (optBtnPressed == LOW && !isWebServerRunning) { //If Button pressed and web server is not running, start it.
      setupWebServer();
    } else if (optBtnPressed == LOW && isWebServerRunning) { //If button pressed and web server is running, disable it.
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      isWebServerRunning = false;
      disableBLE = false;
      DEBUG_PRINTS("Stopping WiFi AP...\r\n");
    }
  }
#endif
  if (isWebServerRunning) { //If web server is running, do actions.
    updateWifi();
  }
}

void setupWebServer() {
  if (isWebServerRunning) { //If already running, skip
    return;
  }
  isWebServerRunning = true;

  //Setup WiFi network with SSID & Password
  DEBUG_PRINTN("Starting WiFi AP: ", ssid);

  WiFi.mode(WIFI_AP_STA);
  delay(100);
  WiFi.softAPConfig(ip, gateway, subnet);
  WiFi.softAP(ssid, wifi_password);
  delay(100);
  MDNS.begin(mdsn_hostname);

  //Setup Wifi endpoints
  server.on("/info", HTTP_GET, []() { //Info page
    server.sendHeader("Connection", "close");
    server.sendHeader("Refresh", "5"); //Auto refresh page every 5 seconds.
    handleInfo();
  });

#if DEBUG
  server.on("/test", HTTP_GET, []() { //Info page
    server.sendHeader("Connection", "close");
    testWebCommand();
  });
#endif
  server.on("/control", HTTP_GET, []() { //GoPro WiFi control page
    if (server.hasArg("shutter")) { //If shutter button pressed, trigger.
      DEBUG_PRINTS("Control: Shutter triggered\r\n");
      if (isCameraRecording) {
        setShutter(false);
      } else {
        setShutter(true);
      }
    } else if (server.hasArg("photo")) { //Photo button pressed
      DEBUG_PRINTS("Control: Photo mode triggered\r\n");
      setMode(GOPRO_CAPTURE_MODE_PHOTO);
    } else if (server.hasArg("video")) { //Video button pressed
      DEBUG_PRINTS("Control: Video mode triggered\r\n");
      setMode(GOPRO_CAPTURE_MODE_VIDEO);
    } else if (server.hasArg("timelapse")) { //Timelapse button pressed
      DEBUG_PRINTS("Control: Timelapse mode triggered\r\n");
      setMode(GOPRO_CAPTURE_MODE_TIME_LAPSE);
    }
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", controlIndex);
  });

  server.on("/config", HTTP_GET, []() { //MavGoPro RC device configure page
    if (server.hasArg("sys_id")) { //User has set Mavlink system id, save in prefs.
      int sys_id = server.arg("sys_id").toInt();
      if (sys_id > 254) {
        server.send(200, "text/plain", "FAILED: System ID must be a value between 0-254!");
        return;
      }
      DEBUG_PRINTN("Config: System ID set to ", sys_id);
      prefs.putInt("sys_id", sys_id);
      MAVLinkSystemID = sys_id;
      server.send(200, "text/plain", "Updated System ID to " + String(sys_id));
    } else if (server.hasArg("comp_id")) {  //User has set Mavlink component id, save in prefs.
      int comp_id = server.arg("comp_id").toInt();
      if (comp_id > 254) {
        server.send(200, "text/plain", "FAILED: Component ID must be a value between 0-254!");
        return;
      }
      DEBUG_PRINTN("Config: Component ID set to ", comp_id);
      prefs.putInt("comp_id", comp_id);
      MAVLinkComponentID = comp_id;
      server.send(200, "text/plain", "Updated Component ID to " + String(comp_id));
    } else if (server.hasArg("mav_protocol")) {  //User has set Mavlink component id, save in prefs.
      int mav_protocol = server.arg("mav_protocol").toInt();
      if (mav_protocol > 2) {
        server.send(200, "text/plain", "FAILED: Component ID must be a value between 1-2!");
        return;
      }
      DEBUG_PRINTN("Config: MAVLink Protocol set to ", mav_protocol);
      prefs.putInt("mav_protocol", mav_protocol);
      MAVLinkProtocol = mav_protocol;
      updateMAVLinkProtocol();
      server.send(200, "text/plain", "Updated MAVLink Protocol to " + String(mav_protocol));
    } else { //Load config page. Needs to be in here to have the variables update dynamically.
      String configIndex =
        "<!DOCTYPE html>"
        "<html lang='en'>"
        "<head></head>"
        "<body>"
        "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
        "<p>System Id: <input type='number' min='0' max='254' name='system_id' id='system_id' value='" + String(MAVLinkSystemID) + "'>"
        " <button id='sys_id_button'>Set System ID</button>&nbsp Default value = 1</p>"
        "<br>"
        "<p>Component Id: <input type='number' min='0' max='254' name='component_id' id='component_id' value='" + String(MAVLinkComponentID) + "'>"
#if SOLO
        " <button id='comp_id_button'>Set Component ID</button>&nbsp Default value = 154 (Gimbal)</p>"
#else
        " <button id='comp_id_button'>Set Component ID</button>&nbsp Default value = 100 (Camera)</p>"
#endif
        "<br>"
        "<p>MAVLink Protocol: <input type='number' min='1' max='2' name='mav_protocol' id='mav_protocol' value='" + String(MAVLinkProtocol) + "'>"
#if SOLO
        " <button id='protocol_button'>Set Protocol</button>&nbsp Default value = 1</p>"
#else
        " <button id='protocol_button'>Set Protocol</button>&nbsp Default value = 2</p>"
#endif
        "<script>"
        "$(document).ready(function(){"
        "var input;"
        "$('#sys_id_button').click(function(e){"
        "e.preventDefault();"
        "input = $('#system_id').val();"
        "$.get('/config?sys_id=' + input, function(data){"
        "alert(data);"
        "if (data.indexOf('FAILED') >= 0){"
        "$('#system_id').val('" + String(MAVLinkSystemID) + "');"
        "}"
        "});"
        "});"
        "$('#comp_id_button').click(function(e){"
        "e.preventDefault();"
        "input = $('#component_id').val();"
        "$.get('/config?comp_id=' + input, function(data){"
        "alert(data);"
        "if (data.indexOf('FAILED') >= 0){"
        "$('#component_id').val('" + String(MAVLinkComponentID) + "');"
        "}"
        "});"
        "});"
        "$('#protocol_button').click(function(e){"
        "e.preventDefault();"
        "input = $('#mav_protocol').val();"
        "$.get('/config?mav_protocol=' + input, function(data){"
        "alert(data);"
        "if (data.indexOf('FAILED') >= 0){"
        "$('#mav_protocol').val('" + String(MAVLinkProtocol) + "');"
        "}"
        "});"
        "});"
        "});"
        "</script>"
        "</body>"
        "</html>";
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", configIndex);
    }
  });

  server.on("/pair", HTTP_GET, []() { //GoPro save pairing page
    server.sendHeader("Connection", "close");
    server.send(200, "text/html",
                "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
                "<form method='POST' action='#' enctype='multipart/form-data' id='pair_form'>"
                "<input type='text' name='camera_name' value='" + savedGoProName + "'>"
                "<input type='submit' value='Submit'>"
                "</form>"
                "<script>"
                "$('form').submit(function(e){"
                "e.preventDefault();"
                "var form = $('#pair_form')[0];"
                "var data = new FormData(form);"
                " $.ajax({"
                "url: '/pair_post',"
                "type: 'POST',"
                "data: data,"
                "contentType: false,"
                "processData:false,"
                "success:function(d, s) {"
                "console.log('success!')"
                "},"
                "error: function (a, b, c) {"
                "}"
                "});"
                "});"
                "</script>"
               );
  });

  server.on("/pair_post", HTTP_POST, []() { //GoPro pairing post request result
    if (!server.hasArg("camera_name")) {
      server.send(500, "text/plain", "BAD ARGS");
      return;
    }
    String path = server.arg("camera_name");
    savedGoProName = path;
    prefs.putString("camera_name", path);
    server.send(200, "text/plain", "SAVED");;
    DEBUG_PRINTN("Saved name: ", savedGoProName.c_str());
  });

#if DEBUG
  server.on("/status_query", HTTP_GET, []() {
    if (!server.hasArg("id")) {
      server.send(500, "text/plain", "BAD ARGS");
      return;
    }
    String idStr = server.arg("id");
    unsigned int id = idStr.toInt();
    if (cameraType == GoPro) {
      pQueryCharacteristic->writeValue({0x02, 0x13, id}, 3);
      goproQueryPacket = id;
      goproQuerySentTimestamp = millis();
    }
  });

    server.on("/setting_query", HTTP_GET, []() {
    if (!server.hasArg("id")) {
      server.send(500, "text/plain", "BAD ARGS");
      return;
    }
    String idStr = server.arg("id");
    unsigned int id = idStr.toInt();
    if (cameraType == GoPro) {
      pQueryCharacteristic->writeValue({0x02, 0x12, id}, 3);
      goproQueryPacket = id;
      goproQuerySentTimestamp = millis();
    }
  });
#endif
  // Handling uploading firmware file
  server.on("/update", HTTP_GET, []() { //Update OTA page
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", updateIndex);
    disableBLE = true;
    if (isCameraConnected) {
      cameraBLEClient->disconnect();
    }
  });

  server.on("/update_post", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      isUpdatingOTA = true;
      delay(10);
      DEBUG_PRINTN("Update: ", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
#if DEBUG
        Update.printError(Serial2);
#endif
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      isUpdatingOTA = true;
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
#if DEBUG
        Update.printError(Serial2);
#endif
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      isUpdatingOTA = true;
      if (Update.end(true)) { //true to set the size to the current progress
        DEBUG_PRINTN("Update Success: ", upload.totalSize);
        DEBUG_PRINTS("Rebooting...\r\n");
      } else {
#if DEBUG
        Update.printError(Serial2);
#endif
      }
    }
  });
  server.begin();
}

//Serve dynamic info text page.
void handleInfo() {
  unsigned long currentMillis = millis();
  String message = "";
#if DEBUG
  message += "DEBUG ENABLED\r\n";
#endif
#if SOLO
  message += "Solo GoPro Remote Controller BLE - fw v" + firmware_version + "\r\nHelp & Support - https://sidepilot.net\r\n\r\n";
  message += "\r\Solo connected: ";
#else
  message += "Mav GoPro Remote Controller - fw v" + firmware_version + "\r\nHelp & Support - https://sidepilot.net\r\n\r\n";
  message += "\r\nMAVLink connected: ";
#endif
  message += isMavlinkConnected ? "Yes" : "No";
  message += "\r\n  Last Mavlink Heartbeat: ";
  if (MAVLinkHeartbeatReceivedTimestamp == 0) {
    message += "Never";
  } else {
    message += (currentMillis - MAVLinkHeartbeatReceivedTimestamp) / 1000;
    message += "s ago";
  }
  message += "\r\n";
  if (cameraType == GoPro) {
    message += "\r\nGoPro Connected: ";
    message += isCameraConnected ? "Yes" : "No";
    if (isCameraConnected || savedGoProName != "") {
      if (cameraBLEAdvertisedDevice != nullptr) {
        message += "\r\n  BLE Name: ";
        message += cameraBLEAdvertisedDevice->getName().c_str();
      } else {
        message += "\r\n  Saved BLE Name: ";
        message += savedGoProName;
      }
    }
    if (goProModel[0] != 0) {
      message += "\r\n  Model: ";
      message += goProModel;
      message += "k";
    }
    if (isCameraConnected) {
      message += "\r\n  Current Mode: ";
      if (cameraCurrentMode == GOPRO_CAPTURE_MODE_PHOTO) {
        message += "Photo";
      } else if (cameraCurrentMode == GOPRO_CAPTURE_MODE_VIDEO) {
        message += "Video";
        message += "\r\n  Recording: ";
        message += isCameraRecording ? "Yes" : "No";
      } else if (cameraCurrentMode == GOPRO_CAPTURE_MODE_TIME_LAPSE) {
        message += "Timelapse";
      } else if (cameraCurrentMode == GOPRO_CAPTURE_MODE_BURST) {
        message += "Burst";
      } else {
        message += "Unknown";
      }
      message += "\r\n  Battery Level: ";
      message += cameraBatteryLevel;
      message += "%";
      if (isOpenGoPro) {
        message += "\r\n  GoPro Hero9+ Detected: ";
      }
    }
  }
  message += "\r\n  Last Camera Heartbeat: ";
  if (cameraHeartbeatReceivedTimestamp == 0) {
    message += "Never";
  } else {
    message += (currentMillis - cameraHeartbeatReceivedTimestamp) / 1000;
    message += "s ago";
  }
  server.send(200, "text/plain", message);
}

void updateWifi() {
  server.handleClient();
}

void LEDFlashTaskLoop(void * parameter) {
  //Call Update LEDs function every 20ms.
  for (;;) {
    updateLEDs();
    delay(20);
  }
}

void updateLEDs() {
  if (!isCameraConnected || isUpdatingOTA) { //If updating firmware or gopro is not connected, flash leds as below.
    if (pairing || paired) { //If pairing or paired, flash at 5hz.
      LED_FLASH_RATE = 200;
    } else if (doConnect || isUpdatingOTA) { //If connecting to GoPro BLE or updating firmware, flash at 10hz
      LED_FLASH_RATE = 100;
    } else {
      LED_FLASH_RATE = 1000; //Otherwise set the flash rate to 1hz
    }
    unsigned long currentMillis = millis();
    if (currentMillis - LED_INTERVAL >= LED_FLASH_RATE) {
      if (BLE_LED_STATE == 1) {
        digitalWrite(BLE_LED, LOW);
        if (isUpdatingOTA) {
          digitalWrite(MAV_LED, HIGH);
        }
        BLE_LED_STATE = 0;
      } else {
        digitalWrite(BLE_LED, HIGH);
        if (isUpdatingOTA) {
          digitalWrite(MAV_LED, LOW);
        }
        BLE_LED_STATE = 1;
      }
      LED_INTERVAL = currentMillis;
    }
  } else if (BLE_LED_STATE != 1) {
    digitalWrite(BLE_LED, HIGH);
    BLE_LED_STATE = 1;
  }
  if (!isUpdatingOTA) { //If not updating firmware, always turn off MAV_LED. Will flash as it receives data.
    digitalWrite(MAV_LED, LOW);
  }
}

//Querys the GoPro Model command.
void getGoProModel() {
  if (pCommandCharacteristic != nullptr) {
    pCommandCharacteristic->writeValue({0x01, 0x3C}, 2);
  }
}

#if DEBUG
//Checks the debug serial port for data.
void debug_receive() {
  if (Serial2.available() <= 0) {
    return;
  }
  while (Serial2.available() > 0) {
    char input = Serial2.read();
    switch (input) {
      case 'v':
        setMode(GOPRO_CAPTURE_MODE_VIDEO);
        break;
      case 'p':
        setMode(GOPRO_CAPTURE_MODE_PHOTO);
        break;
      case 't':
        setMode(GOPRO_CAPTURE_MODE_TIME_LAPSE);
        break;
      case 's':
        setShutter(true);
        break;
      case 'q':
        setShutter(false);
        break;
      case 'n':
        setVideoSettings(GOPRO_RESOLUTION_4k_16_9, GOPRO_FRAME_RATE_60, GOPRO_FIELD_OF_VIEW_SUPERVIEW, 0); break;
      case 'o':
        setVideoSettings(GOPRO_RESOLUTION_5k_16_9, GOPRO_FRAME_RATE_240, GOPRO_FIELD_OF_VIEW_WIDE, 0); break;
      case 'z':
        setVideoSettings(GOPRO_RESOLUTION_2_7k_4_3, GOPRO_FRAME_RATE_60, GOPRO_FIELD_OF_VIEW_SUPERVIEW, 0); break;
      case 'l':
        setVideoSettings(GOPRO_RESOLUTION_4k_16_9, GOPRO_FRAME_RATE_60, GOPRO_FIELD_OF_VIEW_LINEAR, 0); break;
    }
  }
}
#endif

//Checks the serial port for MAVLink data
void mavlink_receive() {
  if (Serial.available() <= 0) {
    return;
  }
  mavlink_message_t msg;
  mavlink_status_t status;
  while (Serial.available() > 0) {
    uint8_t c = Serial.read();
    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { //Parse serial data and check if MAVLink.
      unsigned long currentMillis = millis();
      MAVLinkHeartbeatReceivedTimestamp = currentMillis;
      if (!isMavlinkConnected) {
        DEBUG_PRINTS("MAVLink Connected!\r\n");
        sendMavlinkStatusText();
      }
      isMavlinkConnected = true;
      digitalWrite(MAV_LED, HIGH); //Set the MAV_LED output HIGH.
      // Handle message
      switch (msg.msgid) { //Check the received MAVLink message id.
        case MAVLINK_MSG_ID_COMMAND_LONG:
          {
            mavlink_command_long_t command_long;
            mavlink_msg_command_long_decode(&msg, &command_long);
            uint16_t command = command_long.command;
            if (command == MAV_CMD_DO_DIGICAM_CONTROL) { //MAV_CMD_DO_DIGICAM_CONTROL
              DEBUG_PRINTS("Received MAV_CMD_DO_DIGICAM_CONTROL\r\n");
              if (cameraCurrentMode == GOPRO_CAPTURE_MODE_PHOTO) { //If photo mode, trigger shutter
                setShutter(true);
              } else { //If not photo mode, either enable or disable recording.
                setShutter(!isCameraRecording);
              }
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED); //Send Accepted command back to MAV.
            } else if (command == MAV_CMD_DO_DIGICAM_CONFIGURE) { //MAV_CMD_DO_DIGICAM_CONFIGURE
              DEBUG_PRINTN("Received MAV_CMD_DO_DIGICAM_CONFIGURE: ", command_long.param1);
              if (command_long.param1 == 0) {
                setMode(GOPRO_CAPTURE_MODE_PHOTO);
                sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              } else if (command_long.param1 == 1) {
                setMode(GOPRO_CAPTURE_MODE_VIDEO);
                sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              } else if (command_long.param1 == 2) {
                setMode(GOPRO_CAPTURE_MODE_TIME_LAPSE);
                sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              } else {
                sendMavlinkAck(command, MAV_RESULT_DENIED);
                break;
              }
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
            } else if (command == MAV_CMD_SET_CAMERA_MODE) { //MAV_CMD_SET_CAMERA_MODE
              DEBUG_PRINTN("Received MAV_CMD_SET_CAMERA_MODE: ", command_long.param2);
              if (command_long.param2 == 0) {
                setMode(GOPRO_CAPTURE_MODE_PHOTO);
                sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              } else if (command_long.param2 == 1) {
                setMode(GOPRO_CAPTURE_MODE_VIDEO);
                sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              } else {
                sendMavlinkAck(command, MAV_RESULT_DENIED);
                break;
              }
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
            } else if ((command == MAV_CMD_REQUEST_MESSAGE && command_long.param1 == 259) || command == MAV_CMD_REQUEST_CAMERA_INFORMATION) {
              DEBUG_PRINTS("Received MAV_CMD_REQUEST_CAMERA_INFORMATION\r\n");
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              sendMavlinkCameraInformation();
            } else if ((command == MAV_CMD_REQUEST_MESSAGE && command_long.param1 == 260) || command == MAV_CMD_REQUEST_CAMERA_SETTINGS) {
              DEBUG_PRINTS("Received MAV_CMD_REQUEST_CAMERA_SETTINGS\r\n");
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              sendMavlinkCameraSettings();
            } else if ((command == MAV_CMD_REQUEST_MESSAGE && command_long.param1 == 262) || command == MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS) {
                DEBUG_PRINTS("Received MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS\r\n");
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              sendMavlinkCameraCaptureStatus();
            } else if (command == MAV_CMD_SET_CAMERA_MODE) {
              DEBUG_PRINTS("Received MAV_CMD_SET_CAMERA_MODE\r\n");
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              if (command_long.param2 == CAMERA_MODE_IMAGE) {
                setMode(GOPRO_CAPTURE_MODE_PHOTO);
              } else if (command_long.param2 == CAMERA_MODE_VIDEO) {
                setMode(GOPRO_CAPTURE_MODE_VIDEO);
              } else if (command_long.param2 == CAMERA_MODE_IMAGE_SURVEY) {
                setMode(GOPRO_CAPTURE_MODE_TIME_LAPSE);
              }
            } else if (command == MAV_CMD_IMAGE_START_CAPTURE) {
              DEBUG_PRINTS("Received MAV_CMD_IMAGE_START_CAPTURE\r\n");
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              captureInterval = command_long.param2 / 1000;
              captureTotal = command_long.param3;
              if (captureTotal == 1) {
                setShutter(true);
              } else {
                startIntervalCapture();
              }
            } else if (command == MAV_CMD_IMAGE_STOP_CAPTURE) {
              DEBUG_PRINTS("Received MAV_CMD_IMAGE_STOP_CAPTURE\r\n");
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
            } else if (command == MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE ) {
              DEBUG_PRINTS("Received MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE\r\n");
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              setShutter(true);
            } else if (command == MAV_CMD_VIDEO_START_CAPTURE) {
              DEBUG_PRINTS("Received MAV_CMD_VIDEO_START_CAPTURE\r\n");
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              setShutter(true);
            } else if (command == MAV_CMD_VIDEO_STOP_CAPTURE) {
              DEBUG_PRINTS("Received MAV_CMD_VIDEO_STOP_CAPTURE\r\n");
              sendMavlinkAck(command, MAV_RESULT_ACCEPTED);
              setShutter(false);
            } else {
              DEBUG_PRINTN("Received Unknown CMD: ", command);
              sendMavlinkAck(command, MAV_RESULT_UNSUPPORTED);
            }
          }
          break;
        case MAVLINK_MSG_ID_GOPRO_SET_REQUEST:
          {
            mavlink_gopro_set_request_t gopro_set_request;
            mavlink_msg_gopro_set_request_decode(&msg, &gopro_set_request);
            if (gopro_set_request.cmd_id == GOPRO_COMMAND_CAPTURE_MODE) {
              GOPRO_CAPTURE_MODE newMode = GOPRO_CAPTURE_MODE(gopro_set_request.value[0]);
              if (newMode == GOPRO_CAPTURE_MODE_VIDEO || newMode == GOPRO_CAPTURE_MODE_PHOTO || newMode == GOPRO_CAPTURE_MODE_TIME_LAPSE) {
                setMode(newMode);
                sendMavlinkGoProSetResponse(gopro_set_request.cmd_id, GOPRO_REQUEST_SUCCESS);
              } else {
                sendMavlinkGoProSetResponse(gopro_set_request.cmd_id, GOPRO_REQUEST_FAILED);
                break;
              }
            } else if (gopro_set_request.cmd_id == GOPRO_COMMAND_SHUTTER) {
              setShutter(gopro_set_request.value[0]);
              sendMavlinkGoProSetResponse(gopro_set_request.cmd_id, GOPRO_REQUEST_SUCCESS);
              if (gopro_set_request.cmd_id == GOPRO_COMMAND_VIDEO_SETTINGS) {
                //0 = Resolution, 1 = frame rate, 2 = field of view, 3 = tv mode (0 = NTSC 60Hz, 1 = PAL 50Hz).
                setVideoSettings(GOPRO_RESOLUTION(gopro_set_request.value[0]), GOPRO_FRAME_RATE(gopro_set_request.value[1]), GOPRO_FIELD_OF_VIEW(gopro_set_request.value[2]), gopro_set_request.value[3]);
              } else {
                //If the command cannot be answered, reply failed.
              }
              sendMavlinkGoProSetResponse(gopro_set_request.cmd_id, GOPRO_REQUEST_FAILED);
            }
          }
          break;
        case MAVLINK_MSG_ID_GOPRO_GET_REQUEST:
          {
            mavlink_gopro_get_request_t gopro_get_request;
            mavlink_msg_gopro_get_request_decode(&msg, &gopro_get_request);
            uint8_t value[4] = {};
            if (gopro_get_request.cmd_id == GOPRO_COMMAND_CAPTURE_MODE) {
              value[0] = cameraCurrentMode;
              sendMavlinkGoProGetResponse(gopro_get_request.cmd_id, GOPRO_REQUEST_SUCCESS, value);
            } else if (gopro_get_request.cmd_id == GOPRO_COMMAND_BATTERY) {
              value[0] = cameraBatteryLevel;
              sendMavlinkGoProGetResponse(gopro_get_request.cmd_id, GOPRO_REQUEST_SUCCESS, value);
            } else {
              sendMavlinkGoProGetResponse(gopro_get_request.cmd_id, GOPRO_REQUEST_FAILED, value);
            }
          }
          break;
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
          { //CRC ERROR WHEN USED, JUST DONT RESPOND?
            mavlink_autopilot_version_request_t autopilot_version_request;
            mavlink_msg_autopilot_version_request_decode(&msg, &autopilot_version_request);
            if (autopilot_version_request.target_system == MAVLinkSystemID && autopilot_version_request.target_component == MAVLinkComponentID) {
              //MGRC version has been requested, respond
              sendMavlinkAutopilotVersion();
            }
          }
          break;
        case MAVLINK_MSG_ID_HEARTBEAT:
          break;
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
          break;
        default:
#if DEBUG
            DEBUG_PRINTN("Unknown MAVLink message received: ", msg.msgid);
#endif
          break;
      }
    }
  }
}

void sendMavlinkHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(MAVLinkSystemID, MAVLinkComponentID, &msg, MAVLinkType, MAVLinkAutopilotType, 0, 0, MAV_STATE_ACTIVE);
  packAndSendMav(msg);
}

void sendMavlinkGoProHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_gopro_heartbeat_pack(MAVLinkSystemID, MAVLinkComponentID, &msg, cameraStatus, cameraCurrentMode, isCameraRecording);
  packAndSendMav(msg);
}

void sendMavlinkAck(uint16_t command, uint8_t result) {
  mavlink_message_t msg;
  mavlink_msg_command_ack_pack(MAVLinkSystemID, MAVLinkComponentID, &msg, command, result, UINT8_MAX, 0, 0, 0);
  packAndSendMav(msg);
}

void sendMavlinkGoProSetResponse(uint8_t command, uint8_t result) {
  mavlink_message_t msg;
  mavlink_msg_gopro_set_response_pack(MAVLinkSystemID, MAVLinkComponentID, &msg, command, result);
  packAndSendMav(msg);
}

void sendMavlinkGoProGetResponse(uint8_t command, uint8_t result, uint8_t value[4]) {
  mavlink_message_t msg;
  mavlink_msg_gopro_get_response_pack(MAVLinkSystemID, MAVLinkComponentID, &msg, command, result, value);
  packAndSendMav(msg);
}

void sendMavlinkAutopilotVersion() {
  mavlink_message_t msg;
  mavlink_msg_autopilot_version_pack(MAVLinkSystemID, MAVLinkComponentID, &msg, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0);
  packAndSendMav(msg);
}

void sendMavlinkStatusText() {
  mavlink_message_t msg;
  mavlink_msg_statustext_pack(MAVLinkSystemID, MAVLinkComponentID, &msg, MAV_SEVERITY_INFO, "Mav GoPro Remote Control Connected", 0, 0);
  packAndSendMav(msg);
}

void sendMavlinkCameraInformation() {
  mavlink_message_t msg;
  uint32_t time_boot_ms = uint32_t(millis());
  uint32_t flags = CAMERA_CAP_FLAGS_CAPTURE_VIDEO | CAMERA_CAP_FLAGS_CAPTURE_IMAGE | CAMERA_CAP_FLAGS_HAS_MODES | CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
  uint16_t definition_version = 1;
  uint8_t vendor_name[32] = "SidePilot";
  uint8_t model_name[32] = "MavGoProRC";
  char definition_uri[140] = "https://sidepilot.net/downloads/test_camera_definition_v1.xml";
  mavlink_msg_camera_information_pack(MAVLinkSystemID, MAVLinkComponentID, &msg, time_boot_ms, vendor_name, model_name, firmware_version_int, 0, 0, 0, 0, 0, 0 , flags, definition_version, definition_uri);
  packAndSendMav(msg);
}

void sendMavlinkCameraSettings() {
  mavlink_message_t msg;
  uint32_t time_boot_ms = uint32_t(millis());
  uint8_t cameraMode = CAMERA_MODE_IMAGE;
  if (cameraCurrentMode == GOPRO_CAPTURE_MODE_VIDEO) {
    cameraMode = CAMERA_MODE_VIDEO;
  } else if (cameraCurrentMode == GOPRO_CAPTURE_MODE_TIME_LAPSE) {
    cameraMode = CAMERA_MODE_IMAGE_SURVEY;
  }
  mavlink_msg_camera_settings_pack(MAVLinkSystemID, MAVLinkComponentID, &msg, time_boot_ms, cameraMode, NAN, NAN);
  packAndSendMav(msg);
}

void sendMavlinkCameraCaptured() {
  mavlink_message_t msg;
  uint32_t time_boot_ms = uint32_t(millis());
  int32_t capturedIndex = 0;
  if (isIntervalCaptureEnabled) {
    capturedIndex = captureIndex;
  }
  mavlink_msg_camera_image_captured_pack(MAVLinkSystemID, MAVLinkComponentID, &msg, time_boot_ms, 0, 0, 0, 0, 0, 0, 0, capturedIndex, 1, "");
  packAndSendMav(msg);
}

void sendMavlinkCameraCaptureStatus() {
  mavlink_message_t msg;
  uint32_t time_boot_ms = uint32_t(millis());
  uint8_t status = isCameraRecording;
  mavlink_msg_camera_capture_status_pack(MAVLinkSystemID, MAVLinkComponentID, &msg, time_boot_ms, status, status, captureInterval, 0, 0, 0);
  packAndSendMav(msg);
}

//Helper function to pack and write MAVLink message to serial port.
void packAndSendMav(mavlink_message_t msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  delay(10);
}

//Helper function to convert serial data to HEX and print to debug.
void serialPrintHex(uint8_t msg[], int numBytes) {
  for (int i = 0; i < numBytes; i++) {
    DEBUG_PRINTX("", msg[i]);
    if (i != numBytes - 1) {
      DEBUG_PRINTS(":");
    }
  }
}
