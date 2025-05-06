/*
  =================================================================================================

      MavGoProRC  (MAVLink GoPro Remote Controller)
      GLOBAL VARIABLES
    
      Created by Tom Brereton
      http://sidepilot.net

  ================================================================================================= 
*/

//------------- Setup Variables --------------------
#define DEBUG 0 // Switch debug output on and off by 1 or 0
const String firmware_version = "0.2";
const uint32_t firmware_version_int = 825110784;

//------------- Definitions --------------------
#define BLE_LED 19
#define MAV_LED 18
#define OPT_BTN 4 //0 for dev module, 4 for MavGoPro RC
#define DBG_RX 17
#define DBG_TX 16

// Debugging Setup
#if DEBUG
  #define DEBUG_PRINTS(s)   { Serial2.print(s); }
  #define DEBUG_PRINT(s,v)  { Serial2.print(s); Serial2.print(v); }
  #define DEBUG_PRINTN(s,v)  { Serial2.print(s); Serial2.print(v); Serial2.println(""); }
  #define DEBUG_PRINTX(s,v) { Serial2.print(s); Serial2.print("0x"); Serial2.print(v, HEX); }
#else
  #define DEBUG_PRINTS(s)
  #define DEBUG_PRINT(s,v)
  #define DEBUG_PRINTN(s,v)
  #define DEBUG_PRINTX(s,v)
#endif

Preferences prefs;
//--------------------- LED ------------------------------------------------------------
uint8_t LED_HIGH_COUNT = 0;
unsigned long LED_INTERVAL = 0;
unsigned long BLE_LED_STATE = 0;
unsigned long LED_FLASH_RATE = 1000;
TaskHandle_t LED_FLASH_TASK;
int OPT_BTN_PREV_STATE = HIGH;

enum CameraType {
  GoPro
};

CameraType cameraType = GoPro;

//--------------------- MAVLINK --------------------------------------------------------
int MAVLinkSystemID = 1;
int MAVLinkComponentID = MAV_COMP_ID_CAMERA;
int MAVLinkType = MAV_TYPE_CAMERA;
int MAVLinkAutopilotType = MAV_AUTOPILOT_INVALID;
unsigned long MAVLinkLastSentTimestamp = 0;     // will store last time MAVLink was transmitted and listened
unsigned long MAVLink_interval = 1000;  // next interval to count
boolean isMavlinkConnected = false;
unsigned long MAVLinkHeartbeatReceivedTimestamp = 0;
int MAVLinkProtocol = 2;
uint8_t MAVLINK_MAX_PACKET_LEN_SHORT = 32;
float captureIndex = 0;
float captureTotal = 0;
boolean isIntervalCaptureEnabled = false;
unsigned long captureInterval = 0;
unsigned long intervalLastTimestamp = 0;

//--------------------- GoPro ------------------------------------------------------------
boolean isCameraRecording = 0;
uint8_t cameraCurrentMode = GOPRO_CAPTURE_MODE_UNKNOWN;
boolean isCameraConnected = false;
uint8_t cameraStatus = GOPRO_HEARTBEAT_STATUS_DISCONNECTED;
uint8_t cameraBatteryLevel = 0;
unsigned long cameraKeepAliveInterval = 60000;  // next interval to count
unsigned long cameraHeartbeatReceivedTimestamp = 0;
unsigned long cameraKeepAliveSentTimestamp = 0;
String savedGoProName = "";
boolean goproHasRegisteredQueryStatus = false;
boolean goproNeedsTimelapseFlag = true;
boolean isOpenGoPro = false;
char goProModel[10] = {};
unsigned int goproQueryPacket = 0;
unsigned long goproQuerySentTimestamp = 0;
boolean isCameraPoweredOff = false;

//--------------------- BLE ------------------------------------------------------------
boolean doConnect = false;
boolean doScan = true;
boolean paired = false;
boolean pairing = false;

static BLEAdvertisedDevice* cameraBLEAdvertisedDevice;
static BLEDevice* cameraBLEDevice;

boolean disableBLE = false;

BLEUUID wifiUUID("b5f90001-aa8d-11e3-9046-0002a5d5c51b");
BLEUUID wifiSSIDUUID("b5f90002-aa8d-11e3-9046-0002a5d5c51b");
BLEUUID wifiPasswordUUID("b5f90003-aa8d-11e3-9046-0002a5d5c51b");
BLEUUID wifiPowerUUID("b5f90004-aa8d-11e3-9046-0002a5d5c51b");
BLEUUID wifiStateUUID("b5f90005-aa8d-11e3-9046-0002a5d5c51b");
BLEUUID serviceUUID("FEA6");
BLEUUID commandUUID("b5f90072-aa8d-11e3-9046-0002a5d5c51b");
BLEUUID commandResponseUUID("b5f90073-aa8d-11e3-9046-0002a5d5c51b");
BLEUUID settingsUUID("b5f90074-aa8d-11e3-9046-0002a5d5c51b");
BLEUUID settingsResponseUUID("b5f90075-aa8d-11e3-9046-0002a5d5c51b");
BLEUUID queryUUID("b5f90076-aa8d-11e3-9046-0002a5d5c51b");
BLEUUID queryResponseUUID("b5f90077-aa8d-11e3-9046-0002a5d5c51b");

//--------------------- WiFI ------------------------------------------------------------
const unsigned int wifiChannel = 1;             // Channel
const char* ssid = "MAV_GOPRO_RC";    // SSID
const char* wifi_password = "MavlinkGoProRC"; // PASSKEY
IPAddress ip(10, 71, 79, 1);                    // IP
IPAddress gateway(10, 71, 79, 1);               // GW
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);
const char* mdsn_hostname = "sologopro-rc";
boolean isWebServerRunning = false;
//--------------------- OTA Updates ------------------------------------------------------------

const char* update_path = "/update";
boolean isUpdatingOTA = false;

const char* updateIndex =
  "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
  "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
  "<input type='file' name='update'>"
  "<input type='submit' value='Update'>"
  "</form>"
  "<div id='prg'>progress: 0%</div>"
  "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update_post',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')"
  "},"
  "error: function (a, b, c) {"
  "}"
  "});"
  "});"
  "</script>";

//--------------------- WiFi Client Control UI ------------------------------------------------------------

const char* controlIndex =
  "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
  "<button id='shutter_button'>Trigger Shutter</button>"
  "<br>"
  "<br>"
  "<button id='photo_button'>Mode: Photo</button>"
  "<br>"
  "<br>"
  "<button id='video_button'>Mode: Video</button>"
  "<br>"
  "<br>"
  "<button id='timelapse_button'>Mode: Timelapse</button>"
  "<script>"
  "$(document).ready(function(){"
  "$('#shutter_button').click(function(e){"
  "e.preventDefault();"
  "$.get('/control?shutter=1');"
  "});"
  "$('#photo_button').click(function(e){"
  "e.preventDefault();"
  "$.get('/control?photo=1');"
  "});"
  "$('#video_button').click(function(e){"
  "e.preventDefault();"
  "$.get('/control?video=1');"
  "});"
  "$('#timelapse_button').click(function(e){"
  "e.preventDefault();"
  "$.get('/control?timelapse=1');"
  "});"
  "});"
  "</script>";
