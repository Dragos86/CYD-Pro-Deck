#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <LovyanGFX.hpp>
#include <XPT2046_Touchscreen.h>

// Standard JPEG Huffman Tables for Video Decoder
const uint8_t std_dht[] = {
  0xFF, 0xC4, 0x01, 0xA2, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x10,
  0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7D, 0x01, 0x02, 0x03, 0x00, 0x04,
  0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08, 0x23, 0x42,
  0xB1, 0xC1, 0x15, 0x52, 0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0A, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x25, 0x26, 0x27,
  0x28, 0x29, 0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55,
  0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A,
  0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5,
  0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8,
  0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA,
  0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05,
  0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61,
  0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0, 0x15, 0x62, 0x72, 0xD1,
  0x0A, 0x16, 0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38, 0x39,
  0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66,
  0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A,
  0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4,
  0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
  0xD8, 0xD9, 0xDA, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA
};

class CYDDisplay : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 panel;
  lgfx::Bus_SPI bus;
  lgfx::Light_PWM backlight;

public:
  CYDDisplay() {
    auto b = bus.config();
    b.spi_host = VSPI_HOST;
    b.spi_mode = 0;
    b.freq_write = 80000000; 
    b.freq_read = 16000000;
    b.spi_3wire = true;
    b.use_lock = true;
    b.dma_channel = 1;
    b.pin_sclk = 14;
    b.pin_mosi = 13;
    b.pin_miso = -1;
    b.pin_dc = 2;
    bus.config(b);
    panel.setBus(&bus);

    auto p = panel.config();
    p.pin_cs = 15;
    p.pin_rst = -1;
    p.pin_busy = -1;
    p.memory_width = 320;
    p.memory_height = 240;
    p.panel_width = 320;
    p.panel_height = 240;
    p.offset_x = 0;
    p.offset_y = 0;
    p.offset_rotation = 45;
    p.readable = true;
    p.invert = false;
    p.rgb_order = false;
    panel.config(p);
    
    auto l = backlight.config();
    l.pin_bl = 21;
    l.invert = false;
    l.freq = 44100;
    l.pwm_channel = 7;
    backlight.config(l);
    panel.setLight(&backlight);
    setPanel(&panel);
  }
};

CYDDisplay lcd;
SPIClass touchSPI(HSPI);
XPT2046_Touchscreen touch(33, 36);

const char* DRONE_SSID = "FLOW-UFO-8c5405";
const char* RTSP_HOST = "192.168.1.1";

enum AppState { DASHBOARD, FLY_MODE, FLIP_DIR_MODE, MONITOR_MODE, CAMERA_MODE };
AppState appState = DASHBOARD;
AppState previousState = DASHBOARD; 

// Network Globals
WiFiUDP udp7099;
unsigned long lastHeartbeat = 0;
unsigned long lastCtrlTime = 0;

// Flight Control Flags & States
uint8_t cmdRoll = 128, cmdPitch = 128, cmdThrottle = 128, cmdYaw = 128;
uint8_t packetActionFlag = 0x00; 
bool isLanding = false, isEmergency = false;
int calibTimer = 0, flipTimer = 0;
bool isFlipping = false; // Locks the joystick until flip is done

// UI Joysticks
int leftCursorX = 80, leftCursorY = 90;
int rightCursorX = 240, rightCursorY = 90;
int lastLeftX = 80, lastLeftY = 90;
int lastRightX = 240, lastRightY = 90;

// Video Globals
WiFiClient rtspClient;
WiFiUDP rtpUdp;
String rtspSessionID = "";
int rtpSeqNumber = 3; 
unsigned long lastRtspKeepalive = 0;
bool cameraUINeedsRedraw = true; 
int videoRot = 0; // Toggles between 0 and 2 (the two 90-degree sideways modes)

uint8_t frameBuffer[45000]; 
int framePtr = 0, jpegHeaderLen = 0;
bool hasValidHeader = false;

// UI DRAWING HELPERS
void button(int x, int y, int w, int h, const char* text, uint16_t color) {
  lcd.fillRoundRect(x + 2, y + 2, w, h, 6, TFT_BLACK); 
  lcd.fillRoundRect(x, y, w, h, 6, color);
  lcd.drawRoundRect(x, y, w, h, 6, TFT_LIGHTGREY);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_WHITE);
  lcd.drawString(text, x + w / 2, y + h / 2);
}

void drawDashboard() {
  lcd.setRotation(1); 
  lcd.fillScreen(lcd.color565(15, 15, 20)); 
  
  lcd.fillRect(0, 0, 320, 32, lcd.color565(0, 100, 150));
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextDatum(middle_center);
  lcd.drawString("ULTIMATE PRO DECK", 160, 16);
  
  button(60, 50, 200, 45, "FLY PAD", lcd.color565(0, 120, 255));
  button(60, 110, 200, 45, "CAMERA FPV", lcd.color565(200, 50, 50));
  button(60, 170, 200, 45, "SPECTATOR", lcd.color565(0, 180, 50));

  lcd.setTextColor(TFT_CYAN);
  lcd.setTextDatum(top_center);
  lcd.drawString("WIFI: " + String(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED"), 160, 220);
}

void drawFlyInterface() {
  lcd.setRotation(1);
  lcd.fillScreen(TFT_BLACK);
  
  lcd.drawCircle(80, 90, 60, TFT_DARKGREY);
  lcd.drawCircle(80, 90, 20, TFT_DARKGREY);
  lcd.drawLine(80, 30, 80, 150, TFT_DARKGREY); 
  lcd.drawLine(20, 90, 140, 90, TFT_DARKGREY); 
  
  lcd.drawCircle(240, 90, 60, TFT_DARKGREY);
  lcd.drawCircle(240, 90, 20, TFT_DARKGREY);
  lcd.drawLine(240, 30, 240, 150, TFT_DARKGREY);
  lcd.drawLine(180, 90, 300, 90, TFT_DARKGREY);

  button(5, 195, 56, 35, "<", TFT_RED);
  button(69, 195, 56, 35, "CALIB", lcd.color565(200, 100, 0));
  button(133, 195, 56, 35, "FLIP", lcd.color565(150, 50, 200));
  button(197, 195, 56, 35, "LAND", isLanding ? TFT_ORANGE : lcd.color565(0, 120, 255));
  button(261, 195, 56, 35, "STOP!", TFT_RED);
  
  lastLeftX = -1; lastRightX = -1;
}

void drawFlipInterface() {
  lcd.setRotation(1);
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextDatum(middle_center);
  lcd.drawString("SELECT FLIP DIRECTION", 160, 20);
  
  button(110, 40, 100, 50, "FORWARD", lcd.color565(150, 50, 200));
  button(110, 140, 100, 50, "BACKWARD", lcd.color565(150, 50, 200));
  button(10, 90, 90, 50, "LEFT", lcd.color565(150, 50, 200));
  button(220, 90, 90, 50, "RIGHT", lcd.color565(150, 50, 200));
  button(5, 5, 70, 30, "CANCEL", TFT_RED);
}

void updateCursors() {
  if (appState != FLY_MODE) return;

  if (isLanding || isEmergency || calibTimer > 0) {
    lcd.fillRect(100, 160, 120, 20, TFT_BLACK); 
    lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
    lcd.setTextDatum(middle_center);
    if (isEmergency) lcd.drawString("E-STOP ACTIVE", 160, 170);
    else if (calibTimer > 0) lcd.drawString("CALIBRATING...", 160, 170);
    else lcd.drawString("LANDING...", 160, 170);
  } else {
    lcd.fillRect(100, 160, 120, 20, TFT_BLACK); 
  }

  if (leftCursorX != lastLeftX || leftCursorY != lastLeftY) {
    if (lastLeftX != -1) {
      lcd.fillCircle(lastLeftX, lastLeftY, 12, TFT_BLACK); 
      lcd.drawCircle(80, 90, 60, TFT_DARKGREY);
      lcd.drawLine(80, 30, 80, 150, TFT_DARKGREY);       
      lcd.drawLine(20, 90, 140, 90, TFT_DARKGREY);
    }
    lcd.fillCircle(leftCursorX, leftCursorY, 12, TFT_CYAN); 
    lastLeftX = leftCursorX;
    lastLeftY = leftCursorY;
  }
  if (rightCursorX != lastRightX || rightCursorY != lastRightY) {
    if (lastRightX != -1) {
      lcd.fillCircle(lastRightX, lastRightY, 12, TFT_BLACK);
      lcd.drawCircle(240, 90, 60, TFT_DARKGREY);
      lcd.drawLine(240, 30, 240, 150, TFT_DARKGREY);
      lcd.drawLine(180, 90, 300, 90, TFT_DARKGREY);
    }
    lcd.fillCircle(rightCursorX, rightCursorY, 12, TFT_ORANGE);
    lastRightX = rightCursorX;
    lastRightY = rightCursorY;
  }
}

void connectWiFi() {
  lcd.fillRect(0, 216, 320, 24, lcd.color565(30, 30, 30));
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextDatum(middle_center);
  lcd.drawString("Connecting Wi-Fi...", 160, 228);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(250);
  WiFi.begin(DRONE_SSID);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) delay(200);

  if (WiFi.status() == WL_CONNECTED) {
    udp7099.begin(7099);
  }
  drawDashboard();
}

String readRtspResponse(int timeoutMs) {
  String response = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (rtspClient.available()) response += char(rtspClient.read());
    if (response.indexOf("\r\n\r\n") >= 0) break;
    delay(5);
  }
  return response;
}

void startCameraMode(AppState mode) {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!rtspClient.connect(IPAddress(192, 168, 1, 1), 7070)) return;

  rtspClient.print(
    "SETUP rtsp://192.168.1.1:7070/webcam/track0 RTSP/1.0\r\n"
    "CSeq: " + String(rtpSeqNumber++) + "\r\n"
    "Transport: RTP/AVP;unicast;client_port=5000-5001\r\n\r\n"
  );
  
  String setupResp = readRtspResponse(3000);
  int sessionIdx = setupResp.indexOf("Session: ");
  if (sessionIdx > 0) {
    int endIdx = setupResp.indexOf("\r\n", sessionIdx);
    rtspSessionID = setupResp.substring(sessionIdx + 9, endIdx);
    int semiIdx = rtspSessionID.indexOf(";");
    if (semiIdx > 0) rtspSessionID = rtspSessionID.substring(0, semiIdx);
    rtspSessionID.trim();
  } else {
    rtspClient.stop();
    return;
  }

  rtspClient.print(
    "PLAY rtsp://192.168.1.1:7070/webcam/ RTSP/1.0\r\n"
    "CSeq: " + String(rtpSeqNumber++) + "\r\n"
    "Session: " + rtspSessionID + "\r\n\r\n"
  );
  
  String playResp = readRtspResponse(3000);
  if (playResp.startsWith("RTSP/1.0 200")) {
    rtpUdp.begin(5000);
    appState = mode; 
    lastRtspKeepalive = millis();
    cameraUINeedsRedraw = true; 
  } else {
    rtspClient.stop();
  }
}

void stopCameraMode() {
  rtspClient.stop();
  rtpUdp.stop();
  appState = DASHBOARD;
  lcd.setRotation(1);
  drawDashboard();
}

// Blast UDP/HTTP switch hacks quickly
void fireCameraSwitchHacks() {
  uint8_t pkt1[] = {0x06, 0x01, 0x01, 0x01, 0x00}; 
  udp7099.beginPacket(RTSP_HOST, 7099); udp7099.write(pkt1, sizeof(pkt1)); udp7099.endPacket();
  uint8_t pkt2[] = {0x0A, 0x01, 0x01, 0x00}; 
  udp7099.beginPacket(RTSP_HOST, 8888); udp7099.write(pkt2, sizeof(pkt2)); udp7099.endPacket();

  WiFiClient c;
  c.setTimeout(100);
  if(c.connect(IPAddress(192, 168, 1, 1), 80)) { c.print("GET /cgi-bin/setting?cam=next HTTP/1.0\r\n\r\n"); }
  c.stop();
}

void drawCameraUI() {
  lcd.setRotation(1); // Buttons are ALWAYS perfectly drawn in Landscape
  lcd.fillScreen(TFT_BLACK);
  
  if (appState == MONITOR_MODE) {
    int cx = 160, cy = 100;
    lcd.drawCircle(cx, cy, 30, TFT_GREEN);
    lcd.drawFastVLine(cx, cy - 40, 20, TFT_GREEN);
    lcd.drawFastVLine(cx, cy + 20, 20, TFT_GREEN);
    lcd.drawFastHLine(cx - 40, cy, 20, TFT_GREEN);
    lcd.drawFastHLine(cx + 20, cy, 20, TFT_GREEN);
    lcd.setTextColor(TFT_GREEN);
    lcd.setTextDatum(top_center);
    lcd.drawString("STEALTH SPECTATOR ACTIVE", cx, 10);
    lcd.drawString("USE OG REMOTE", cx, 25);
  }

  // Simplified Camera Buttons (Flip removed from Camera mode)
  button(10, 200, 90, 35, "ROTATE", lcd.color565(0, 120, 255));
  button(115, 200, 90, 35, "SW CAM", lcd.color565(200, 100, 0));
  button(220, 200, 90, 35, "BACK", TFT_RED);
}

void handleCameraRTSP() {
  if (appState != CAMERA_MODE && appState != MONITOR_MODE) return;

  if (millis() - lastRtspKeepalive > 4000) {
    if (rtspClient.connected()) {
      rtspClient.print(
        "OPTIONS rtsp://192.168.1.1:7070/webcam RTSP/1.0\r\n"
        "CSeq: " + String(rtpSeqNumber++) + "\r\n"
        "Session: " + rtspSessionID + "\r\n\r\n"
      );
      while(rtspClient.available()) rtspClient.read();
      lastRtspKeepalive = millis();
    } else {
      stopCameraMode();
      return;
    }
  }

  if (cameraUINeedsRedraw) {
    drawCameraUI();
    cameraUINeedsRedraw = false;
  }

  bool frameReadyToDraw = false;
  while (int packetSize = rtpUdp.parsePacket()) {
    uint8_t buf[1460];
    int len = rtpUdp.read(buf, sizeof(buf));
    if (len < 12) continue;

    uint8_t pt = buf[1] & 0x7F;
    bool marker = (buf[1] & 0x80) != 0;
    if (pt != 26) continue; 

    int rfcOffset = 12;
    uint32_t fragOffset = (buf[rfcOffset+1] << 16) | (buf[rfcOffset+2] << 8) | buf[rfcOffset+3];
    uint8_t type = buf[rfcOffset+4];
    uint8_t q = buf[rfcOffset+5];
    uint8_t w = buf[rfcOffset+6];
    uint8_t h = buf[rfcOffset+7];
    int headerLen = 8;
    uint16_t restartInterval = 0;
    
    if (type >= 64 && type <= 127) {
        restartInterval = (buf[rfcOffset + 8] << 8) | buf[rfcOffset + 9];
        headerLen += 4;
    }

    uint8_t* qTable = nullptr;
    if (fragOffset == 0 && q >= 128) {
        int qHeaderOffset = rfcOffset + headerLen;
        int qLen = (buf[qHeaderOffset+2] << 8) | buf[qHeaderOffset+3];
        qTable = &buf[qHeaderOffset+4];
        headerLen += 4 + qLen;
    }

    int payloadOffset = rfcOffset + headerLen;
    int payloadLen = len - payloadOffset;

    if (fragOffset == 0) {
        hasValidHeader = true;
        framePtr = 0;
        frameBuffer[framePtr++] = 0xFF; frameBuffer[framePtr++] = 0xD8; 
        if (qTable != nullptr) {
            frameBuffer[framePtr++] = 0xFF; frameBuffer[framePtr++] = 0xDB;
            frameBuffer[framePtr++] = 0x00; frameBuffer[framePtr++] = 0x84; 
            frameBuffer[framePtr++] = 0x00; 
            memcpy(&frameBuffer[framePtr], qTable, 64);
            framePtr += 64;
            frameBuffer[framePtr++] = 0x01; 
            memcpy(&frameBuffer[framePtr], qTable + 64, 64);
            framePtr += 64;
        }
        if (restartInterval > 0) {
            frameBuffer[framePtr++] = 0xFF; frameBuffer[framePtr++] = 0xDD; 
            frameBuffer[framePtr++] = 0x00; frameBuffer[framePtr++] = 0x04;
            frameBuffer[framePtr++] = (restartInterval >> 8) & 0xFF;
            frameBuffer[framePtr++] = restartInterval & 0xFF;
        }
        memcpy(&frameBuffer[framePtr], std_dht, sizeof(std_dht)); 
        framePtr += sizeof(std_dht);
        frameBuffer[framePtr++] = 0xFF; frameBuffer[framePtr++] = 0xC0; 
        frameBuffer[framePtr++] = 0x00; frameBuffer[framePtr++] = 0x11; 
        frameBuffer[framePtr++] = 0x08; 
        int height = h * 8; int width = w * 8;
        frameBuffer[framePtr++] = (height >> 8) & 0xFF; frameBuffer[framePtr++] = height & 0xFF;
        frameBuffer[framePtr++] = (width >> 8) & 0xFF; frameBuffer[framePtr++] = width & 0xFF;
        frameBuffer[framePtr++] = 0x03; 
        uint8_t y_samp = (type == 0 || type == 64) ? 0x21 : 0x22;
        frameBuffer[framePtr++] = 0x01; frameBuffer[framePtr++] = y_samp; frameBuffer[framePtr++] = 0x00;
        frameBuffer[framePtr++] = 0x02; frameBuffer[framePtr++] = 0x11; frameBuffer[framePtr++] = 0x01;
        frameBuffer[framePtr++] = 0x03; frameBuffer[framePtr++] = 0x11; frameBuffer[framePtr++] = 0x01;
        frameBuffer[framePtr++] = 0xFF; frameBuffer[framePtr++] = 0xDA; 
        frameBuffer[framePtr++] = 0x00; frameBuffer[framePtr++] = 0x0C;
        frameBuffer[framePtr++] = 0x03; 
        frameBuffer[framePtr++] = 0x01; frameBuffer[framePtr++] = 0x00;
        frameBuffer[framePtr++] = 0x02; frameBuffer[framePtr++] = 0x11;
        frameBuffer[framePtr++] = 0x03; frameBuffer[framePtr++] = 0x11;
        frameBuffer[framePtr++] = 0x00; frameBuffer[framePtr++] = 0x3F; frameBuffer[framePtr++] = 0x00;
        jpegHeaderLen = framePtr;
    }

    if (hasValidHeader && fragOffset + jpegHeaderLen + payloadLen < sizeof(frameBuffer)) {
        memcpy(&frameBuffer[jpegHeaderLen + fragOffset], &buf[payloadOffset], payloadLen);
        if (jpegHeaderLen + fragOffset + payloadLen > framePtr) {
            framePtr = jpegHeaderLen + fragOffset + payloadLen;
        }
    }
    if (marker && hasValidHeader) {
        frameBuffer[framePtr++] = 0xFF; frameBuffer[framePtr++] = 0xD9; 
        frameReadyToDraw = true;
        hasValidHeader = false; 
    }
  }

  if (frameReadyToDraw) {
    // ---------------------------------------------------------------------------------
    // THE 90-DEGREE FULLSCREEN MAGIC
    // By physically rotating the screen to Portrait (0 or 2) while drawing the sideways 240x320 feed,
    // the image perfectly fits the screen horizontally in Landscape!
    // ---------------------------------------------------------------------------------
    lcd.setRotation(videoRot); 

    // ANTI-FLICKER MATH: Protects the bottom 45 pixels of the Landscape (Rotation 1) view.
    // If we are in rot 0, the landscape bottom is on the left (X=0 to 45).
    // If we are in rot 2, the landscape bottom is on the right (X=195 to 240).
    if (videoRot == 0) lcd.setClipRect(45, 0, 195, 320); 
    else if (videoRot == 2) lcd.setClipRect(0, 0, 195, 320); 

    lcd.drawJpg(frameBuffer, framePtr, 0, 0); 
    
    lcd.clearClipRect();
    lcd.setRotation(1); // Always reset to standard Landscape for touch logic!
  }
}

void handleUDP() {
  if (WiFi.status() != WL_CONNECTED) return;

  // -------------------------------------------------------------
  // TRUE STEALTH SPECTATOR MODE
  // Absolute radio silence. Physical remote takes 100% control.
  // -------------------------------------------------------------
  if (appState == MONITOR_MODE) {
    while (udp7099.parsePacket()) {
      uint8_t buf[256]; udp7099.read(buf, sizeof(buf));
    }
    return; 
  }

  if (millis() - lastHeartbeat > 2000) {
    uint8_t heartbeat[] = {0x01, 0x01};
    udp7099.beginPacket(RTSP_HOST, 7099);
    udp7099.write(heartbeat, sizeof(heartbeat));
    udp7099.endPacket();
    lastHeartbeat = millis();
  }

  if ((appState == FLY_MODE || appState == FLIP_DIR_MODE || appState == CAMERA_MODE) && millis() - lastCtrlTime > 50) {
    uint8_t packet[21] = {0};
    packet[0] = 0x03; packet[1] = 0x66; packet[2] = 0x14; 
    
    if (isEmergency) {
      packet[3] = 128; packet[4] = 128; packet[5] = 0; packet[6] = 128; 
      packet[7] = 0x04; 
    } 
    else if (calibTimer > 0) {
      packet[3] = 255; packet[4] = 0; packet[5] = 0; packet[6] = 255; 
      packet[7] = 0x00;
      calibTimer--;
    } 
    else {
      packet[3] = cmdRoll; packet[4] = cmdPitch; packet[5] = cmdThrottle; packet[6] = cmdYaw; 
      packet[7] = packetActionFlag; 
      
      if (isLanding) packet[7] = 0x02; 
      
      // FLIP BUG FIX: Only release the flag once the timer is zero
      if (flipTimer > 0) {
        packet[7] |= 0x08; 
        flipTimer--;
        if (flipTimer == 0) isFlipping = false; // Finally unlock the joystick
      }
    }
    
    packet[8] = 0x02; 
    uint8_t checksum = 0;
    for (int i = 3; i < 19; i++) checksum ^= packet[i];
    packet[19] = checksum; packet[20] = 0x99; 

    udp7099.beginPacket(RTSP_HOST, 7099);
    udp7099.write(packet, sizeof(packet));
    udp7099.endPacket();
    lastCtrlTime = millis();
  }

  while (udp7099.parsePacket()) {
    uint8_t buf[256];
    udp7099.read(buf, sizeof(buf));
  }
}

bool readTouch(int &x, int &y) {
  if (!touch.touched()) return false;
  TS_Point p = touch.getPoint();
  x = map(p.x, 3735, 345, 0, 319);
  y = map(p.y, 3537, 408, 0, 239);
  x = constrain(x, 0, 319);
  y = constrain(y, 0, 239);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // MAXIMIZE CPU AND WI-FI RANGE
  setCpuFrequencyMhz(240);

  lcd.init();
  lcd.setColorDepth(16);
  
  touchSPI.begin(25, 39, 32, 33);
  touch.begin(touchSPI);
  touch.setRotation(1);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); 
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  drawDashboard();
}

bool wasTouched = false; 

void loop() {
  handleUDP();
  handleCameraRTSP();

  int x, y;
  bool isTouched = readTouch(x, y);

  if (isTouched) {
    if (appState == DASHBOARD && !wasTouched) {
      if (y >= 210 && x >= 110 && x <= 210 && WiFi.status() != WL_CONNECTED) {
        connectWiFi();
      }
      else if (WiFi.status() == WL_CONNECTED) {
        if (y >= 45 && y <= 95) {
          if (x >= 60 && x <= 260) { appState = FLY_MODE; drawFlyInterface(); }
        } else if (y >= 105 && y <= 155) {
          if (x >= 60 && x <= 260) startCameraMode(CAMERA_MODE);
        } else if (y >= 165 && y <= 215) {
          if (x >= 60 && x <= 260) startCameraMode(MONITOR_MODE);
        }
      }
    } 
    else if (appState == FLY_MODE) {
      if (y > 190 && !wasTouched) {
        if (x < 61) {
          appState = DASHBOARD;
          isLanding = false; isEmergency = false;
          drawDashboard();
        } else if (x > 69 && x < 125) {
          calibTimer = 20; 
          isLanding = false; isEmergency = false;
        } else if (x > 133 && x < 189) {
          previousState = appState;
          appState = FLIP_DIR_MODE;
          drawFlipInterface();
        } else if (x > 197 && x < 253) {
          isLanding = !isLanding;
          isEmergency = false;
          drawFlyInterface();
        } else if (x > 261) {
          isEmergency = !isEmergency;
          isLanding = false;
          drawFlyInterface();
        }
      } 
      else if (!isLanding && !isEmergency && calibTimer == 0 && !isFlipping && y < 190) {
        if (x < 160) {
          leftCursorX = constrain(x, 20, 140); leftCursorY = constrain(y, 30, 150);
          cmdYaw = map(leftCursorX, 20, 140, 0, 255);
          cmdThrottle = map(leftCursorY, 30, 150, 255, 0); 
        } else {
          rightCursorX = constrain(x, 180, 300); rightCursorY = constrain(y, 30, 150);
          cmdRoll = map(rightCursorX, 180, 300, 0, 255);
          cmdPitch = map(rightCursorY, 30, 150, 255, 0); 
        }
      }
    }
    else if (appState == FLIP_DIR_MODE && !wasTouched) {
      if (x > 5 && x < 75 && y > 5 && y < 35) { 
        appState = previousState; 
        if (appState == FLY_MODE) drawFlyInterface(); 
      } 
      // FLIP BUG FIX: Locks the joystick value in code (isFlipping = true) until flip timer runs out!
      else if (y > 40 && y < 90 && x > 110 && x < 210) { isFlipping = true; cmdPitch = 255; flipTimer = 25; appState = previousState; drawFlyInterface(); } // FWD
      else if (y > 150 && y < 200 && x > 110 && x < 210) { isFlipping = true; cmdPitch = 0; flipTimer = 25; appState = previousState; drawFlyInterface(); } // BACK
      else if (x > 10 && x < 100 && y > 95 && y < 145) { isFlipping = true; cmdRoll = 0; flipTimer = 25; appState = previousState; drawFlyInterface(); } // LEFT
      else if (x > 220 && x < 310 && y > 95 && y < 145) { isFlipping = true; cmdRoll = 255; flipTimer = 25; appState = previousState; drawFlyInterface(); } // RIGHT
    }
    else if (appState == CAMERA_MODE || appState == MONITOR_MODE) {
      if (!wasTouched && y > 195) {
        if (x < 100) { 
          // ROTATE VIDEO BUTTON: Toggles between 0 and 2 (the two fullscreen 90-degree sideways modes)
          videoRot = (videoRot == 0) ? 2 : 0;
          cameraUINeedsRedraw = true;
        } else if (x > 110 && x < 210) { 
          fireCameraSwitchHacks();
        } else if (x > 220) {
          stopCameraMode();
        }
      }
    }
  } else {
    // FINGER RELEASED
    // ONLY reset the flight sticks to center if we are not currently locked in a flip!
    if (appState == FLY_MODE && !isLanding && !isEmergency && calibTimer == 0 && !isFlipping) {
      cmdRoll = 128; cmdPitch = 128; cmdThrottle = 128; cmdYaw = 128;
      leftCursorX = 80; leftCursorY = 90;
      rightCursorX = 240; rightCursorY = 90;
    }
  }
  
  if (appState == FLY_MODE) updateCursors();
  wasTouched = isTouched;
}