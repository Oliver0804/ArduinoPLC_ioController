#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// Pin definitions
const int PIN_X0 = A1;
const int PIN_X1 = A0;
const int PIN_X2 = 7;
const int PIN_X3 = 4;
const int PIN_X4 = 3; // Supports high-speed interrupt input 只支持NPN
const int PIN_X5 = 2; // Supports high-speed interrupt input 只支持NPN
const int PIN_AD1 = A6;
const int PIN_AD2 = A7;
const int PIN_Y0 = 5;
const int PIN_Y1 = 6;
const int PIN_Y2 = 8;
const int PIN_Y3 = 9;

// Instance for LCD and SoftwareSerial
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial Slave(A2, A3);

// Arduino的最大類比讀數（10位ADC）
const int MAX_ANALOG_READ = 1023;

// 函數聲明
void setupIO();
void readInputs();
void readVoltage();
void updateOutputs();
void handleSerialCommunication();
void handleSoftwareSerialCommunication();
void displayStatus();
void IOCheck();
void onChange();
void sendPacket();
byte calculateCRC(const byte *data, byte length);
void processCommand(const String &command);
void reportInputChange(int pin, bool state);

// Global variables
String comdata = "";
int inputStates[6] = {0};
int previousInputStates[6] = {0}; // 儲存先前的輸入狀態以檢測變化
float voltageADC[2] = {0};

int outputStates[4] = {0};
int PStates[4] = {0};
bool enableControl[4] = {true, true, true, true}; // 控制每個輸出的使能狀態
long ActNums = 0;

void setup()
{
  Serial.begin(115200);
  Slave.begin(9600);
  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  // GPIO
  setupIO();
  IOCheck();
  // SHOW LCD
  lcd.setCursor(0, 0);
  lcd.print("Oliver0804");
  Serial.println("Oliver0804");
}

void loop()
{
  readInputs();
  readVoltage();
  handleSerialCommunication();
  handleSoftwareSerialCommunication();
  updateOutputs();
  displayStatus();
}

void setupIO()
{
  // Initialize input and output pins
  pinMode(PIN_Y0, OUTPUT);
  pinMode(PIN_Y1, OUTPUT);
  pinMode(PIN_Y2, OUTPUT);
  pinMode(PIN_Y3, OUTPUT);

  pinMode(PIN_X0, INPUT);
  pinMode(PIN_X1, INPUT);
  pinMode(PIN_X2, INPUT);
  pinMode(PIN_X3, INPUT);
  pinMode(PIN_X4, INPUT_PULLUP);
  pinMode(PIN_X5, INPUT_PULLUP);
}

void readInputs()
{
  // Read input states
  for (int i = 0; i < 6; i++)
  {
    int newState = !digitalRead(PIN_X0 + i);
    if (inputStates[i] != newState)
    {
      inputStates[i] = newState;
      reportInputChange(i, inputStates[i]); // 報告輸入狀態變化
    }
  }
}

void handleSerialCommunication()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.println("Received command: " + command); // Debugging line
    processCommand(command);                        // Process command when data is received
  }
}

void handleSoftwareSerialCommunication()
{
  static unsigned long lastSendTime = 0; // Stores the last send time
  unsigned long currentMillis = millis();

  // Handle SoftwareSerial communication
  while (Slave.available())
  {
    byte combyte = Slave.read();
    comdata += char(combyte);
    delay(2);
  }
  if (comdata.length() > 0)
  {
    Serial.println("Received from Slave: " + comdata); // Debugging line
    comdata = "";
    lastSendTime = currentMillis; // Update the last send time
  }

  // Check if 1000ms have passed since the last packet was sent
  if (currentMillis - lastSendTime >= 1000)
  {
    sendPacket();                 // Send packet every 1000ms
    lastSendTime = currentMillis; // Update the last send time
  }
}

void updateOutputs()
{
  // Update output states based on inputs and serial data
  outputStates[0] = (inputStates[0] == LOW && enableControl[0]) || PStates[0];
  outputStates[1] = (inputStates[1] == LOW && enableControl[1]) || PStates[1];
  outputStates[2] = (inputStates[2] == LOW && enableControl[2]) || PStates[2];
  outputStates[3] = (inputStates[3] == LOW && enableControl[3]) || PStates[3];

  digitalWrite(PIN_Y0, outputStates[0] ? LOW : HIGH);
  digitalWrite(PIN_Y1, outputStates[1] ? LOW : HIGH);
  digitalWrite(PIN_Y2, outputStates[2] ? LOW : HIGH);
  digitalWrite(PIN_Y3, outputStates[3] ? LOW : HIGH);
}

void displayStatus()
{
  char strA0[6]; // 足夠的空間存儲最多5個字符（包括小數點和結束符）
  char strA1[6];
  // Display status on LCD
  lcd.setCursor(0, 1);
  lcd.print(inputStates[0]);
  lcd.setCursor(1, 1);
  lcd.print(inputStates[1]);
  lcd.setCursor(2, 1);
  lcd.print(inputStates[2]);
  lcd.setCursor(3, 1);
  lcd.print(inputStates[3]);
  lcd.setCursor(4, 1);
  lcd.print(inputStates[4]);
  lcd.setCursor(5, 1);
  lcd.print(inputStates[5]);

  // 將電壓值轉換為字符串，保留一位小數
  dtostrf(voltageADC[0], 3, 1, strA0); // 總寬度5字符（含小數點和小數位）
  dtostrf(voltageADC[1], 3, 1, strA1);
  lcd.setCursor(7, 1);
  lcd.print(strA0);

  lcd.setCursor(12, 1);
  lcd.print(strA1);
}

void IOCheck()
{
  // IO Check functionality
  digitalWrite(PIN_Y0, HIGH);
  delay(500);
  digitalWrite(PIN_Y0, LOW);

  digitalWrite(PIN_Y1, HIGH);
  delay(500);
  digitalWrite(PIN_Y1, LOW);

  digitalWrite(PIN_Y2, HIGH);
  delay(500);
  digitalWrite(PIN_Y2, LOW);

  digitalWrite(PIN_Y3, HIGH);
  delay(500);
  digitalWrite(PIN_Y3, LOW);
}

void onChange()
{
  // Interrupt handler
  ActNums += 1;
}

void sendPacket()
{
  byte packetLength = 1 + 1 + 6 + 8 + 1; // 封包頭1byte + 長度1byte + inputStates 6byte + voltageADC 8byte + CRC 1byte
  byte packet[packetLength];

  // 構建封包
  packet[0] = 0xA5;         // 封包頭
  packet[1] = packetLength; // 數據長度

  // 添加inputStates數據
  for (int i = 0; i < 6; i++)
  {
    packet[2 + i] = inputStates[i];
  }

  // 添加voltageADC數據
  for (int i = 0; i < 2; i++)
  {
    // 正確處理從float到byte的轉換
    union
    {
      float voltage;
      byte voltageBytes[4];
    } converter;

    converter.voltage = voltageADC[i];
    for (int j = 0; j < 4; j++)
    {
      packet[8 + i * 4 + j] = converter.voltageBytes[j];
    }
  }

  // 計算並添加CRC
  packet[packetLength - 1] = calculateCRC(packet, packetLength - 1);

  // 通過串口發送封包
  Slave.write(packet, packetLength);
}

byte calculateCRC(const byte *data, byte length)
{
  byte crc = 0; // 簡單的CRC計算，需要根據實際需求進行修改
  for (byte i = 0; i < length; i++)
  {
    crc ^= data[i];
  }
  return crc;
}

void readVoltage()
{
  int analogValueA0 = analogRead(PIN_AD1);
  int analogValueA1 = analogRead(PIN_AD2);

  // 將0-1023的讀數轉換為0-10V的電壓
  voltageADC[0] = (analogValueA0 * 10.0) / MAX_ANALOG_READ;
  voltageADC[1] = (analogValueA1 * 10.0) / MAX_ANALOG_READ;

  voltageADC[0] = round(voltageADC[0] * 10) / 10.0;
  voltageADC[1] = round(voltageADC[1] * 10) / 10.0;
}

void processCommand(const String &command)
{
  bool commandProcessed = false; // 用於追蹤命令是否已被處理
  // 示例命令格式: ENABLE Y0, DISABLE Y0
  if (command.startsWith("ENABLE"))
  {
    if (command.indexOf("Y0") != -1)
    {
      enableControl[0] = true;
      commandProcessed = true;
    }
    if (command.indexOf("Y1") != -1)
    {
      enableControl[1] = true;
      commandProcessed = true;
    }
    if (command.indexOf("Y2") != -1)
    {
      enableControl[2] = true;
      commandProcessed = true;
    }
    if (command.indexOf("Y3") != -1)
    {
      enableControl[3] = true;
      commandProcessed = true;
    }
  }
  else if (command.startsWith("DISABLE"))
  {
    if (command.indexOf("Y0") != -1)
    {
      enableControl[0] = false;
      commandProcessed = true;
    }
    if (command.indexOf("Y1") != -1)
    {
      enableControl[1] = false;
      commandProcessed = true;
    }
    if (command.indexOf("Y2") != -1)
    {
      enableControl[2] = false;
      commandProcessed = true;
    }
    if (command.indexOf("Y3") != -1)
    {
      enableControl[3] = false;
      commandProcessed = true;
    }
  }

  // 根據命令處理結果給出反饋
  if (commandProcessed)
  {
    Serial.println("Command executed: " + command);
  }
  else
  {
    Serial.println("Invalid command: " + command);
  }
}

void reportInputChange(int pin, bool state)
{
  Serial.print("Input change on X");
  Serial.print(pin);
  Serial.print(": ");
  Serial.println(state ? "HIGH" : "LOW");
}
