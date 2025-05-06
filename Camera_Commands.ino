/*
  =====================================================================================================================
      
	  MavGoProRC  (MAVLink GoPro Remote Controller)
	  CAMERA COMMANDS
	  
      Created by Tom Brereton
      http://sidepilot.net
	  
  =====================================================================================================================
*/

void startIntervalCapture() {
  isIntervalCaptureEnabled = true;
  intervalLastTimestamp = millis();
  captureIndex = 0;
}

void stopIntervalCapture() {
  isIntervalCaptureEnabled = false;
  intervalLastTimestamp = 0;
}

void setShutter(boolean isEnabled) {
  DEBUG_PRINTN("Set Shutter: ", isEnabled);
  if (cameraType == GoPro) {
    if (isEnabled) {
      pCommandCharacteristic->writeValue({0x03, 0x01, 0x01, 0x01}, 4);
    } else {
      pCommandCharacteristic->writeValue({0x03, 0x01, 0x01, 0x00}, 4);
    }
  }
  sendMavlinkCameraCaptured();
}

void setGoProPower(boolean setOn) {
  if (setOn) {
    isCameraPoweredOff = false;
  } else {
    isCameraPoweredOff = true;
    pCommandCharacteristic->writeValue({0x01, 0x05}, 2);
  }
}

void setMode(GOPRO_CAPTURE_MODE mode) {
  switch (mode) {
    case GOPRO_CAPTURE_MODE_VIDEO:
      DEBUG_PRINTS("Set mode: Video\r\n");
      if (cameraType == GoPro) {
        if (isOpenGoPro) {
          pCommandCharacteristic->writeValue({0x04, 0x3E, 0x02, 0x03, 0xE8}, 5);
        } else {
          pCommandCharacteristic->writeValue({0x03, 0x02, 0x01, 0x00}, 4);
        }
      }
      break;
    case GOPRO_CAPTURE_MODE_PHOTO:
      DEBUG_PRINTS("Set mode: Photo\r\n");
      if (cameraType == GoPro) {
        if (isOpenGoPro) {
          pCommandCharacteristic->writeValue({0x04, 0x3E, 0x02, 0x03, 0xE9}, 5);
        } else {
          pCommandCharacteristic->writeValue({0x03, 0x02, 0x01, 0x01}, 4);
        }
      }
      break;
    case GOPRO_CAPTURE_MODE_TIME_LAPSE:
      DEBUG_PRINTS("Set mode: Timelapse\r\n");
      if (cameraType == GoPro) {
        if (isOpenGoPro) {
          pCommandCharacteristic->writeValue({0x04, 0x3E, 0x02, 0x03, 0xEA}, 5);
        } else {
          pCommandCharacteristic->writeValue({0x05, 0x03, 0x01, 0x00, 0x01, 0x01}, 6);
        }
      }
      break;
  }
}

void setVideoSettings(GOPRO_RESOLUTION resolution, GOPRO_FRAME_RATE frameRate, GOPRO_FIELD_OF_VIEW fov, uint8_t tvMode) {
  if (pSettingsCharacteristic != nullptr) {
    switch (resolution) {
    case GOPRO_RESOLUTION_1080p:
      pSettingsCharacteristic->writeValue({0x03, 0x02, 0x01, 0x09}, 4); break;
    case GOPRO_RESOLUTION_2_7k_16_9:
      pSettingsCharacteristic->writeValue({0x03, 0x02, 0x01, 0x04}, 4); break;
    case GOPRO_RESOLUTION_2_7k_4_3:
      pSettingsCharacteristic->writeValue({0x03, 0x02, 0x01, 0x06}, 4); break;
    case GOPRO_RESOLUTION_1440p:
      pSettingsCharacteristic->writeValue({0x03, 0x02, 0x01, 0x07}, 4); break;
    case GOPRO_RESOLUTION_4k_16_9:
      pSettingsCharacteristic->writeValue({0x03, 0x02, 0x01, 0x01}, 4); break;
    case GOPRO_RESOLUTION_4k_4_3:
      pSettingsCharacteristic->writeValue({0x03, 0x02, 0x01, 0x12}, 4); break;
    case GOPRO_RESOLUTION_5k_16_9:
      pSettingsCharacteristic->writeValue({0x03, 0x02, 0x01, 0x18}, 4); break;
    case GOPRO_RESOLUTION_5k_4_3:
      pSettingsCharacteristic->writeValue({0x03, 0x02, 0x01, 0x19}, 4); break;
    case GOPRO_RESOLUTION_5_3k:
      pSettingsCharacteristic->writeValue({0x03, 0x02, 0x01, 0x64}, 4); break;
    }

    switch (frameRate) {
      case GOPRO_FRAME_RATE_240:
        pSettingsCharacteristic->writeValue({0x03, 0x03, 0x01, 0x00}, 4); break;
      case GOPRO_FRAME_RATE_120:
        pSettingsCharacteristic->writeValue({0x03, 0x03, 0x01, 0x01}, 4); break;
      case GOPRO_FRAME_RATE_100:
        pSettingsCharacteristic->writeValue({0x03, 0x03, 0x01, 0x02}, 4); break;
      case GOPRO_FRAME_RATE_60:
        pSettingsCharacteristic->writeValue({0x03, 0x03, 0x01, 0x05}, 4); break;
      case GOPRO_FRAME_RATE_50:
        pSettingsCharacteristic->writeValue({0x03, 0x03, 0x01, 0x06}, 4); break;
      case GOPRO_FRAME_RATE_30:
        pSettingsCharacteristic->writeValue({0x03, 0x03, 0x01, 0x08}, 4); break;
      case GOPRO_FRAME_RATE_25:
        pSettingsCharacteristic->writeValue({0x03, 0x03, 0x01, 0x09}, 4); break;
      case GOPRO_FRAME_RATE_24:
        pSettingsCharacteristic->writeValue({0x03, 0x03, 0x01, 0x0A}, 4); break;
      case GOPRO_FRAME_RATE_200:
        pSettingsCharacteristic->writeValue({0x03, 0x03, 0x01, 0x0D}, 4); break;
      }
    if (isOpenGoPro) {
      if (cameraCurrentMode == GOPRO_CAPTURE_MODE_PHOTO) {
        switch (fov) {
        case GOPRO_FIELD_OF_VIEW_NARROW:
          pSettingsCharacteristic->writeValue({0x03, 0x7A, 0x01, 0x13}, 4); break;
        case GOPRO_FIELD_OF_VIEW_MAX_SUPERVIEW:
          pSettingsCharacteristic->writeValue({0x03, 0x7A, 0x01, 0x64}, 4); break;
        case GOPRO_FIELD_OF_VIEW_WIDE:
          pSettingsCharacteristic->writeValue({0x03, 0x7A, 0x01, 0x65}, 4); break;
        case GOPRO_FIELD_OF_VIEW_LINEAR:
          pSettingsCharacteristic->writeValue({0x03, 0x7A, 0x01, 0x66}, 4); break;
        }
      } else if (cameraCurrentMode == GOPRO_CAPTURE_MODE_VIDEO) {
        switch (fov) {
        case GOPRO_FIELD_OF_VIEW_WIDE:
          pSettingsCharacteristic->writeValue({0x03, 0x79, 0x01, 0x00}, 4); break;
        case GOPRO_FIELD_OF_VIEW_NARROW:
          pSettingsCharacteristic->writeValue({0x03, 0x79, 0x01, 0x02}, 4); break;
        case GOPRO_FIELD_OF_VIEW_SUPERVIEW:
          pSettingsCharacteristic->writeValue({0x03, 0x79, 0x01, 0x03}, 4); break;
        case GOPRO_FIELD_OF_VIEW_LINEAR:
          pSettingsCharacteristic->writeValue({0x03, 0x79, 0x01, 0x04}, 4); break;
        case GOPRO_FIELD_OF_VIEW_MAX_SUPERVIEW:
          pSettingsCharacteristic->writeValue({0x03, 0x79, 0x01, 0x07}, 4); break;
        case GOPRO_FIELD_OF_VIEW_LINEAR_LEVELING:
          pSettingsCharacteristic->writeValue({0x03, 0x79, 0x01, 0x08}, 4); break;
        }
      } else if (cameraCurrentMode == GOPRO_CAPTURE_MODE_TIME_LAPSE) {
        switch (fov) {
        case GOPRO_FIELD_OF_VIEW_NARROW:
          pSettingsCharacteristic->writeValue({0x03, 0x7B, 0x01, 0x13}, 4); break;
        case GOPRO_FIELD_OF_VIEW_MAX_SUPERVIEW:
          pSettingsCharacteristic->writeValue({0x03, 0x7B, 0x01, 0x64}, 4); break;
        case GOPRO_FIELD_OF_VIEW_WIDE:
          pSettingsCharacteristic->writeValue({0x03, 0x7B, 0x01, 0x65}, 4); break;
        case GOPRO_FIELD_OF_VIEW_LINEAR:
          pSettingsCharacteristic->writeValue({0x03, 0x7B, 0x01, 0x66}, 4); break;
        }
      }
    } else {
      if (cameraCurrentMode == GOPRO_CAPTURE_MODE_VIDEO) {
        switch (fov) {
        case GOPRO_FIELD_OF_VIEW_WIDE:
          pSettingsCharacteristic->writeValue({0x03, 0x04, 0x01, 0x00}, 4); break;
        case GOPRO_FIELD_OF_VIEW_MEDIUM:
          pSettingsCharacteristic->writeValue({0x03, 0x04, 0x01, 0x01}, 4); break;
        case GOPRO_FIELD_OF_VIEW_NARROW:
          pSettingsCharacteristic->writeValue({0x03, 0x04, 0x01, 0x02}, 4); break;
        case GOPRO_FIELD_OF_VIEW_LINEAR:
          pSettingsCharacteristic->writeValue({0x03, 0x04, 0x01, 0x04}, 4); break;
        case GOPRO_FIELD_OF_VIEW_SUPERVIEW:
          pSettingsCharacteristic->writeValue({0x03, 0x04, 0x01, 0x03}, 4); break;
        }
      }
    }
    
    if (isOpenGoPro) {
      if (tvMode == 1) {
        pSettingsCharacteristic->writeValue({0x03, 0x86, 0x01, 0x03}, 4);
      } else {
        pSettingsCharacteristic->writeValue({0x03, 0x86, 0x01, 0x02}, 4);
      }
    } else {
      if (tvMode == 1) {
        pSettingsCharacteristic->writeValue({0x03, 0x39, 0x01, 0x01}, 4);
      } else {
        pSettingsCharacteristic->writeValue({0x03, 0x39, 0x01, 0x00}, 4);
      }
    }
  }
}

//TODO: Future Implementation for advances settings

void getVideoSettings() {
  
}

void setPhotoResolution(GOPRO_RESOLUTION resolution) {

}

void getPhotoResolution() {

}

void setPhotoBurstRate(GOPRO_BURST_RATE) {

}

void getPhotoBurstRate() {
  
}

void setLowLight(boolean isEnabled) {
  
}

void getLowLight() {
  
}

void setProTune(boolean isEnabled) {

}

void getProTune() {
  
}

void setProTuneWhiteBalance(GOPRO_PROTUNE_WHITE_BALANCE whiteBalance) {
  
}

void getProTuneWhiteBalance() {
  
}

void setProTuneColour(GOPRO_PROTUNE_COLOUR colour) {
  
}

void getProTuneColour() {
  
}

void setProTuneGain(GOPRO_PROTUNE_GAIN gain) {
  
}

void getProTuneGain() {
  
}

void setProTuneSharpness(GOPRO_PROTUNE_SHARPNESS sharpness) {
  
}

void getProTuneSharpness() {
  
}

void setProTuneExposure(GOPRO_PROTUNE_EXPOSURE) {
  
}

void getProTuneExposure() {
  
}
