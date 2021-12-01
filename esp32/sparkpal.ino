#include <SPI.h>
#include <Wire.h>
#include "BluetoothSerial.h" // https://github.com/espressif/arduino-esp32
#include "sparkCMDs.h"
//#include <BfButton.h> //https://github.com/mickey9801/ButtonFever
#include <Regexp.h>
#include "SevSeg.h"

#define LED_BUILTIN 2

// Device Info Definitions
const String DEVICE_NAME = "SparkPal";
const String VERSION = "0.3.1";

// GPIO Buttons/LEDs
#define BUTTON_PRESET_UP_GPIO 26
#define BUTTON_PRESET_DOWN_GPIO 25
#define BUTTON_DRIVE_GPIO 19
#define BUTTON_MOD_GPIO 18
#define BUTTON_DELAY_GPIO 32
#define BUTTON_1 34
#define BUTTON_2 39
#define BUTTON_3 19
#define BUTTON_4 18
#define LED_POWER 23

#define LED_DRIVE_GPIO 13
#define LED_MOD_GPIO 12
#define LED_DELAY_GPIO 14

// Buttons should be wired between GPIO and ground (input pull down).
//BfButton btn_drive(BfButton::STANDALONE_DIGITAL, BUTTON_DRIVE_GPIO);
//BfButton btn_mod(BfButton::STANDALONE_DIGITAL, BUTTON_MOD_GPIO);
//BfButton btn_delay(BfButton::STANDALONE_DIGITAL, BUTTON_DELAY_GPIO);
//BfButton btn_preset_up(BfButton::STANDALONE_DIGITAL, BUTTON_PRESET_UP_GPIO);
//BfButton btn_preset_down(BfButton::STANDALONE_DIGITAL, BUTTON_PRESET_DOWN_GPIO);
//BfButton btn_1(BfButton::STANDALONE_DIGITAL, BUTTON_1);
//BfButton btn_2(BfButton::STANDALONE_DIGITAL, BUTTON_2);
//BfButton btn_3(BfButton::STANDALONE_DIGITAL, BUTTON_3);
//BfButton btn_4(BfButton::STANDALONE_DIGITAL, BUTTON_4);

// OLED Screen config
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)

// ESP32 Bluetooth Serial Object
BluetoothSerial SerialBT;
// Regex Tester
MatchState ms;
SevSeg sevseg; //Instantiate a seven segment object

// Variables
int selectedPreset;
bool isBTConnected;
bool isInitBoot;
byte incoming_byte;
byte recBTData[1024];
byte recBTDataTrimmed[1024]; // Trim the prefix cmd bytes from the received data
char asciiData[1024];       // Converted from the byte data to ASCII for FX parse
int recBTDataIndex = 0;
const int FX_DRIVE = 0;
const int FX_MOD = 1;
const int FX_DELAY = 2;
int fxSelected[3]; // 0-> Drive, 1-> Mod, 2-> Delay
int fxEnabled[3];

//-------------------------------------------------------------------------------------
// BUTTONS
double now = millis();
double BUTTON_DEBOUNCE_MS = 20;
bool BUTTON_CHANGED[4] = { false, false, false, false };
double BUTTON_LAST_CHANGE_TIME[4] = { now, now, now, now };
int BUTTON_PINS[4] = { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 };
int BUTTON_STATE[4] = { LOW, LOW, LOW, LOW };
int BUTTON_LAST_STATE[4] = { LOW, LOW, LOW, LOW };


void IRAM_ATTR changeButton1() {
  BUTTON_CHANGED[0] = true;
}

void IRAM_ATTR changeButton2() {
  BUTTON_CHANGED[1] = true;
}

void IRAM_ATTR changeButton3() {
  BUTTON_CHANGED[2] = true;
}

void IRAM_ATTR changeButton4() {
  BUTTON_CHANGED[3] = true;
}

// Buttons can start high or low, we only care about change.
void setInitialButtonState() {
  BUTTON_STATE[0] = digitalRead(BUTTON_PINS[0]);
  BUTTON_STATE[1] = digitalRead(BUTTON_PINS[1]);
  BUTTON_STATE[2] = digitalRead(BUTTON_PINS[2]);
  BUTTON_STATE[3] = digitalRead(BUTTON_PINS[3]);
}

void setupButtons() {
  Serial.println("Setup buttons...");
  // Setup the button event handler
  //btn_drive.onPress(btnFXHandler);
  //btn_mod.onPress(btnFXHandler);
  //btn_delay.onPress(btnFXHandler);
  //btn_preset_up.onPress(btnPresetHandler);
  //btn_preset_down.onPress(btnPresetHandler);

  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);
  pinMode(BUTTON_4, INPUT_PULLUP);

  //setInitialButtonState();
    
  attachInterrupt(digitalPinToInterrupt(BUTTON_1), changeButton1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_2), changeButton2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_3), changeButton3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_4), changeButton4, CHANGE);
}
//-------------------------------------------------------------------------------------

void btEventCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  // On BT connection close
  if (event == ESP_SPP_CLOSE_EVT ) {    
    isBTConnected = false;
    selectedPreset = 0;
  }
}

void displayMessage(String message, int fontSize){
  //Serial.println("TODO: Display message");
}

void initDisplay() {
  Serial.println("TODO: Init display");
  // TODO
}

//void btnPresetNumHandler(BfButton *btn, BfButton::press_pattern_t pattern) {
//  if (pattern == BfButton::SINGLE_PRESS) {
//    int pressed_btn_gpio = btn->getID();    
//    // Debug    
//    Serial.println("");
//    Serial.print("Button pressed: ");
//    Serial.print(pressed_btn_gpio);    
//    
//    // Button 1
//    if (pressed_btn_gpio == BUTTON_1) {
//      Serial.println("- Preset 1");
//      selectedPreset = 1;
//    }
//    
//    // Button 2
//    if (pressed_btn_gpio == BUTTON_2) {
//      Serial.println("- Preset 2");
//      selectedPreset = 2;
//    }
//
//    // Button 3
//    if (pressed_btn_gpio == BUTTON_3) {
//      Serial.println("- Preset 3");
//      selectedPreset = 3;
//    }
//
//    // Button 4
//    if (pressed_btn_gpio == BUTTON_4) {
//      Serial.println("- Preset 4");
//      selectedPreset = 4;
//    }
//    
//    // Send the preset command
//    sendSetPresetCmd(PRESET_CMD_LIST[selectedPreset - 1]); // selectedPreset is 1-4, but the array is 0 based
//    // Delay 50ms and then ask for the preset's data
//    delay(50);
//    //Serial.println("Request for current preset's data");
//    SerialBT.write(GET_CURR_PRESET_DATA_CMD, 60);
//  }
//}

//void btnPresetHandler(BfButton *btn, BfButton::press_pattern_t pattern) {
//  if (pattern == BfButton::SINGLE_PRESS) {
//    int pressed_btn_gpio = btn->getID();    
//    // Debug    
//    Serial.println("");
//    Serial.print("Button pressed: ");
//    Serial.println(pressed_btn_gpio);    
//    //Up preset
//    if (pressed_btn_gpio == BUTTON_PRESET_UP_GPIO) {
//      Serial.println("Preset up");
//      if (selectedPreset >= 4) {
//        selectedPreset = 1;
//      }
//      else {
//        selectedPreset++;
//      }      
//    }
//    //Down preset
//    else if (pressed_btn_gpio == BUTTON_PRESET_DOWN_GPIO) {
//      Serial.println("Preset down");
//      if (selectedPreset <= 1) {
//        selectedPreset = 4;
//      }
//      else {
//        selectedPreset--;
//      }      
//    }
//    // Send the preset command
//    sendSetPresetCmd(PRESET_CMD_LIST[selectedPreset - 1]); // selectedPreset is 1-4, but the array is 0 based
//    // Delay 50ms and then ask for the preset's data
//    delay(50);
//    Serial.println("Requst for current preset's data");
//    SerialBT.write(GET_CURR_PRESET_DATA_CMD, 60);
//  }
//}
//
//void btnFXHandler(BfButton *btn, BfButton::press_pattern_t pattern) {
//  if (pattern == BfButton::SINGLE_PRESS) {
//    int pressed_btn_gpio = btn->getID();        
//    // Debug    
//    Serial.println("");
//    Serial.print("Button pressed: ");
//    Serial.println(pressed_btn_gpio);        
//    int fxType;
//    int fxStatus;
//    int LEDPin;
//    if(pressed_btn_gpio == BUTTON_DRIVE_GPIO){
//      fxType = 0;        
//      LEDPin = LED_DRIVE_GPIO;
//    }
//    else if(pressed_btn_gpio == BUTTON_MOD_GPIO){
//      fxType = 1;
//      LEDPin = LED_MOD_GPIO;
//    }
//    else if(pressed_btn_gpio == BUTTON_DELAY_GPIO){
//      fxType = 2;
//      LEDPin = LED_DELAY_GPIO;
//    }
//    //Toggle fx status      
//    fxStatus = fxEnabled[fxType];
//    //New status
//    if(fxStatus == 0){
//      fxStatus = 1;
//      digitalWrite (LEDPin, HIGH);
//    }
//    else{
//      fxStatus = 0;
//      digitalWrite (LEDPin, LOW);
//    }
//    //Upate variables
//    fxEnabled[fxType]= fxStatus;  
//    //Send fx on/off command
//    sendFXOnOffCmd(fxType, fxStatus);   
//  }
//}

void sendSetPresetCmd(byte* setPresetCmd) {
  SerialBT.write(setPresetCmd, SET_PRESET_CMD_SIZE);
  displayMessage(String(selectedPreset), 8);  
}

void sendFXOnOffCmd(int fxType, int fxNewStatus){
  char* fxTypeString;
  char* fxStatusString;
  // Debug info
  if(fxType == FX_DRIVE){ fxTypeString = "Drive";}
  else if(fxType == FX_MOD){ fxTypeString = "Mod";}
  else if(fxType == FX_DELAY){ fxTypeString = "Delay";}
  if(fxNewStatus == 0){ fxStatusString = "Off";}
  else if(fxNewStatus == 1){ fxStatusString = "On";}
  //Serial.print("Send command to toggle ");
  //Serial.print(fxTypeString);
  //Serial.print(" ");
  //Serial.println(fxStatusString);

  if(fxType == FX_DRIVE){ //Drive        
    if(fxNewStatus == 0){ //Off
      SerialBT.write(FX_DRIVE_OFF_CMD_LIST[fxSelected[fxType]], FX_DRIVE_CMD_SIZE_LIST[fxSelected[fxType]]);
    }
    else{ //On
      SerialBT.write(FX_DRIVE_ON_CMD_LIST[fxSelected[fxType]], FX_DRIVE_CMD_SIZE_LIST[fxSelected[fxType]]);
    } 
  }  
  else if(fxType == 1){ //Mod  
    if(fxNewStatus == 0){ //Off
      SerialBT.write(FX_MOD_OFF_CMD_LIST[fxSelected[fxType]], FX_MOD_CMD_SIZE_LIST[fxSelected[fxType]]);
    }
    else{ //On
      SerialBT.write(FX_MOD_ON_CMD_LIST[fxSelected[fxType]], FX_MOD_CMD_SIZE_LIST[fxSelected[fxType]]);
    }      
  }
  else if(fxType == 2){ //Delay
    if(fxNewStatus == 0){ //Off
      SerialBT.write(FX_DELAY_OFF_CMD_LIST[fxSelected[fxType]], FX_DELAY_CMD_SIZE_LIST[fxSelected[fxType]]);
    }
    else{ //On
      SerialBT.write(FX_DELAY_ON_CMD_LIST[fxSelected[fxType]], FX_DELAY_CMD_SIZE_LIST[fxSelected[fxType]]);
    }  
  }
}

void initBT() {
  // Register BT event callback method
  SerialBT.register_callback(btEventCallback);
  if (!SerialBT.begin(DEVICE_NAME, true)) { // Detect for BT failure on ESP32 chip
    displayMessage("BT init failed", 1);
    Serial.println("BT Init Failed!");
    // Loop infinitely until device shutdown/restart
    while (true) {};
  }
}

void connectToAmp() {
  // Loop until device establishes connection with amp
  while (!isBTConnected) {
    displayMessage("Connecting", 2);
    Serial.println("Connecting");
    isBTConnected = SerialBT.connect(SPARK_BT_NAME);    
    if (isBTConnected && SerialBT.hasClient()) {
      // Success
      displayMessage("Connected",2);
      digitalWrite(LED_POWER, HIGH);
      // Delay a sec to show the Connected message on the screen
      delay(1000);
      //Get the current selected preset number
      Serial.println("Asking for current selected preset");
      SerialBT.write(GET_CURR_PRESET_CMD, 24);
    } else { 
      // Failed. Retry the connection.
      isBTConnected = false;
      displayMessage("Reconnect",2);        
      Serial.println("Reconnect");
      delay(1000);
    }
  }
}

void convertPresetDataBytesToASCII() {
  int data_size = sizeof recBTDataTrimmed / sizeof recBTDataTrimmed[0];
  for (int i = 0; i < data_size; i++) {
    asciiData[i] = (char) recBTDataTrimmed[i];
    //Replace 0x00 with '&' since 0x00 is the null terminator
    if (recBTDataTrimmed[i] == 0x00) {
      asciiData[i] = (char) 0x26;
    }
    else {
      asciiData[i] = (char) recBTDataTrimmed[i];
    }
  }
}

void parseASCIIToFXStats() {
  char result;
  ms.Target(asciiData);
  // Serial.print(asciiData);
  char* fxTypeString;
  int totalPedalCount = 0;
  int LEDPin;
  for (int fxType = 0; fxType < 3; fxType++) {
    if (fxType == 0) { //Drive
      fxTypeString = "Drive";
      totalPedalCount = 10;
      LEDPin = LED_DRIVE_GPIO;
    }
    else if (fxType == 1) { //Mod
      fxTypeString = "Mod";
      totalPedalCount = 12;
      LEDPin = LED_MOD_GPIO;
    }
    else if (fxType == 2) { //Delay
      fxTypeString = "Delay";
      totalPedalCount = 6;
      LEDPin = LED_DELAY_GPIO;
    }
    //Reset the curent fx status
    fxSelected[fxType] = -1;
    fxEnabled[fxType] = 0;

    //Serial.println("-- FX status--");
    //Loop through each different possible pedal name regexp match
    for (int i = 0; i < totalPedalCount; i++) {
      if (fxType == FX_DRIVE) { //Drive
        result = ms.Match(exp_drive[i]);
      }
      else if (fxType == FX_MOD) { //Mod
        result = ms.Match(exp_mod[i]);
      }
      else if (fxType == FX_DELAY) { //Delay
        result = ms.Match(exp_delay[i]);
      }
      if (result == REGEXP_MATCHED) {
        fxSelected[fxType] = i;
        // Serial.println(ms.MatchStart);
        // Serial.println(ms.MatchLength);

        //In case there's unexpected char between the effect name and status 
        byte fxStatus;
        for (int a = 0; a < 13; a++){
          if (fxStatus != 0x43 && fxStatus != 0x42) {
            fxStatus = recBTDataTrimmed[ms.MatchStart + ms.MatchLength + a];  
          }  
          else{
            break;
          }
        }
        //Serial.println(fxStatus, HEX);  // enabled/disabled
        if (fxStatus == 0x43) {
          fxEnabled[fxType] = 1;
          digitalWrite (LEDPin, HIGH);
        }
        else if (fxStatus == 0x42) {
          fxEnabled[fxType] = 0;
          digitalWrite (LEDPin, LOW);
        }
        else {
          //Serial.println("Neither 0x43 nor 0x42!!!!!!!!!!!!!!!!!!!!!");          
        }
        //Serial.print(fxTypeString);
        //Serial.print(" matched --- Pedal Num: ");
        //Serial.print(fxSelected[fxType]);
        //Serial.print(", Status: ");
        // Serial.println(fxEnabled[fxType]);
        
        //Exit the looop
        break;
      }   
    }
    if (fxSelected[fxType] == -1){
      Serial.print("Oops... FX - ");
      Serial.print(fxTypeString);
      Serial.println(" is not matched");
    }
  }
}

void updateSelectedPreset(byte selectedPresetByte) {
  if (selectedPresetByte == 0x00) {
    selectedPreset = 1;
  }
  else if (selectedPresetByte == 0x01) {
    selectedPreset = 2;
  }
  else if (selectedPresetByte == 0x02) {
    selectedPreset = 3;
  }
  else if (selectedPresetByte == 0x03) {
    selectedPreset = 4;
  }
  displayMessage((String)selectedPreset, 8);
  //Serial.print("Current Preset is ");
  //Serial.println(selectedPreset);
}


void processRecBTData(const byte inByte) {
  //Gathering the received byte
  recBTData[recBTDataIndex] = inByte;
  recBTDataIndex++;
  int j = 0;

  //End of the incoming BT data and the last data is the ending char F7
  if ( !SerialBT.available() && inByte == (char)0xF7) {
    //Serial.println(" ");
    //Serial.print("Total data length: ");
    //Serial.println(recBTDataIndex);

    //Only process the preset data if it is long enough (detailed preset data) (300 is an arbitrary number)
    //TODO check the prefix of the response type
    if (recBTDataIndex > 300) {
      //Serial.println("Trimming the block/CMD control bytes");
      for (int i = 0; i < recBTDataIndex; i++) {
        //Serial.print(recBTData[i], HEX);
        if (recBTData[i] == 0xF0 && recBTData[i + 1] == 0x01) {
          i = i + 9;
          //Serial.println("trimmed block start");
        }
        else if (recBTData[i] == 0x01 && recBTData[i + 1] == 0xFE && recBTData[i + 2] == 0x00 && recBTData[i + 3] == 0x00 && recBTData[i + 4] == 0x41) {
          i = i + 15;
          //Serial.println("trimmed cmd start");
        }
        else if ( recBTData[i] == 0xf7) {
          //Serial.println("trimmed block ending");
        }
        else {
          recBTDataTrimmed[j] = recBTData[i];
          //Debug
          //Serial.print(" ");
          //if (recBTDataTrimmed[i] < 15){
          //Serial.print(0);
          //}
          //Serial.print(recBTDataTrimmed[j], HEX);
          j = j + 1;
        }
      }
      //Convert the trimmed data to ASCII
      convertPresetDataBytesToASCII();
      //Serial.println(asciiData);

      //Check ecah FX pedal's status
      parseASCIIToFXStats();
    }
    else {
      //Received the current selected preset response
      if (recBTDataIndex == 26 && recBTData[0] == 0x01 && recBTData[1] == 0xFE && recBTData[4] == 0x41 && recBTData[5] == 0xFF && recBTData[6] == 0x1A && recBTData[16] == 0xF0 && recBTData[17] == 0x01 && recBTData[18] == 0x08) {
        updateSelectedPreset(recBTData[24]);
        //Ask for the preset data if this is just booted
        if(isInitBoot == true){
          //Serial.println("Asking for current preset data");
          //SerialBT.write(GET_CURR_PRESET_DATA_CMD, 60);
          isInitBoot = false;
        }
      }
      //Received the preset changed response
      else if (recBTDataIndex == 23 && recBTData[0] == 0x01 && recBTData[1] == 0xFE && recBTData[4] == 0x41 && recBTData[5] == 0xFF && recBTData[6] == 0x17 && recBTData[18] == 0x24) {
        //TODO: revisit here. I liked the reactive nature here, but this can a bit slow. (Changed to send the reqeuest right after send the presetn change command.)
        //Request the current preset's configuration data to update the fx status
        //Serial.println("Asking for current preset data");
        //SerialBT.write(GET_CURR_PRESET_DATA_CMD, 60);
      }
    }

    //Reset index and clear the array since this is the end of the message
    recBTDataIndex = 0;
    memset(recBTData, 0, sizeof(recBTData));
    memset(recBTDataTrimmed, 0, sizeof(recBTDataTrimmed));
    memset(asciiData, 0, sizeof(asciiData));
  }
}

void blinkDelay(int PIN, int blinks = 1, int blinkDelay = 100) {
  for (int i = 0; i < blinks; i++) {
    digitalWrite(PIN, HIGH);
    delay(blinkDelay);

    digitalWrite(PIN, LOW);
    delay(blinkDelay);
  }
}

void setupDisplay() {
  byte numDigits = 1;
  byte digitPins[] = { };
  byte segmentPins[] = { 12, 14, 27, 26, 25, 33, 32, 35 };
  bool resistorsOnSegments = true; // 'false' means resistors are on digit pins
  byte hardwareConfig = COMMON_CATHODE; // See README.md for options
  bool updateWithDelays = false; // Default 'false' is Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  bool disableDecPoint = true; // Use 'true' if your decimal point doesn't exist or isn't connected. Then, you only need to specify 7 segmentPins[]

  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,
  updateWithDelays, leadingZeros, disableDecPoint);

  sevseg.setBrightness(90);
  setDisplayChar("-");
}

void setup() {
  digitalWrite(LED_BUILTIN, HIGH);
  isInitBoot = true;

  // SETUP LEDS
  pinMode(LED_DRIVE_GPIO, OUTPUT);
  pinMode(LED_MOD_GPIO, OUTPUT);
  pinMode(LED_DELAY_GPIO, OUTPUT);
  pinMode(LED_POWER, OUTPUT);

  // INIT LEDS
  digitalWrite(LED_POWER, LOW);

  // Blink power led 2 times to indicate power on.
  // It will remain off until connection is established.
  blinkDelay(LED_POWER, 2);
  
  // Start serial debug console monitoring
  Serial.begin(115200);
  while (!Serial);
  delay(50);
  Serial.println("-- Welcome to Bert's SparkPal! --");
  digitalWrite(LED_BUILTIN, LOW);
 
  // Set initial device state values
  isBTConnected = false;
  selectedPreset = 0;

  setupDisplay();
  setupButtons();
  initDisplay();
  initBT();
}

bool debounce(byte buttonNum) {
  double now = millis();
  double lastChangeTime = BUTTON_LAST_CHANGE_TIME[buttonNum];
  double ellapsed = now - lastChangeTime;

  //Serial.print("- elapsed: ");
  //Serial.println(ellapsed);

  // Debounce.
  if (ellapsed < BUTTON_DEBOUNCE_MS) {
    //Serial.print("Debounced B");
    //Serial.println(buttonNum);
    BUTTON_CHANGED[buttonNum] = false;
    return true;
  }

  BUTTON_LAST_CHANGE_TIME[buttonNum] = now;
  return false;
}

void selectPreset(byte presetNum) {
  // Send the preset command
  sendSetPresetCmd(PRESET_CMD_LIST[presetNum]); // selectedPreset is 1-4, but the array is 0 based
  // Delay 50ms and then ask for the preset's data
  delay(50);
  //Serial.println("Request for current preset's data");
  SerialBT.write(GET_CURR_PRESET_DATA_CMD, 60);
}

void setDisplayNum(int num) {
  sevseg.setNumber(num);
  sevseg.refreshDisplay();
}

void setDisplayChar(char c[]) {
  sevseg.setChars(c);
  sevseg.refreshDisplay();
}

void onButtonChange() {
  if (BUTTON_CHANGED[0]) {
    if (debounce(0)) return;
    selectPreset(0);
    BUTTON_CHANGED[0] = false;
    Serial.println("[ 1 ]");
    setDisplayNum(1);
    return;
  }
  if (BUTTON_CHANGED[1]) {
    if (debounce(1)) return;
    selectPreset(1);
    BUTTON_CHANGED[1] = false;
    Serial.println("[ 2 ]");
    setDisplayNum(2);
    return;
  }
  if (BUTTON_CHANGED[2]) {
    if (debounce(2)) return;
    selectPreset(2);
    BUTTON_CHANGED[2] = false;
    Serial.println("[ 3 ]");
    setDisplayNum(3);
    return;
  }
  if (BUTTON_CHANGED[3]) {
    if (debounce(3)) return;
    selectPreset(3);
    BUTTON_CHANGED[3] = false;
    Serial.println("[ 4 ]");
    setDisplayNum(4);
    return;
  }
}

void onButton(byte buttonNum) {
  int state = digitalRead(BUTTON_PINS[buttonNum]);
  int lastState = BUTTON_LAST_STATE[buttonNum];
  bool now = millis();
  bool lastChangeTime = BUTTON_LAST_CHANGE_TIME[buttonNum];
  bool ellapsed = now - lastChangeTime;

  // Debounce.
  Serial.print("- elapsed: ");
  Serial.println(ellapsed);
  if (ellapsed < BUTTON_DEBOUNCE_MS) {
    Serial.print("Debounced B");
    Serial.println(buttonNum);
    return;
  }

  // On change.
  if (state != lastState) {
    BUTTON_CHANGED[buttonNum] = true;
    BUTTON_LAST_STATE[buttonNum] = state;
    BUTTON_LAST_CHANGE_TIME[buttonNum] = now;
  } else {
    BUTTON_CHANGED[buttonNum] = false; // TODO: Remove?
  }
}

void loop() {
  // Check if amp is connected to device
  if (!isBTConnected) {
    // If not, attempt to establish a connection
    connectToAmp();
  } else { // If amp is connected to device over BT
    // Read button inputs
    onButtonChange();
    
    // Read in response data from amp, to clear BT message buffer
    if (SerialBT.available() > 0) { //If there is coming data in the pipeline
      processRecBTData(SerialBT.read()); //read 1 byte
    }
  }
}
