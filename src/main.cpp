/*
* MIT License
* 
* Copyright (c) 2024 Kouhei Ito
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/


//Controller for M5Fly
//#define DEBUG

#include <Arduino.h>
#include <M5AtomS3.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <MPU6886.h>
#include <MadgwickAHRS.h>
#include <atoms3joy.h>
#include <FS.h>
#include <SPIFFS.h>
#include "buzzer.h"

#define CHANNEL 1

// TDMA Settings
#define TDMA_DEVICE_ID 0         // Device ID: 0=Master, 1-9=Slave (manual setting)
#define TDMA_FRAME_US 10000      // 1 frame = 10ms
#define TDMA_SLOT_US 1000        // 1 slot = 1ms
#define TDMA_NUM_SLOTS 10        // 10 slots per frame
#define TDMA_BEACON_ADVANCE_US 250  // Beacon fires 250us before frame start

#define ANGLECONTROL 0
#define RATECONTROL 1
#define ANGLECONTROL_W_LOG 2
#define RATECONTROL_W_LOG 3
#define ALT_CONTROL_MODE 1
#define NOT_ALT_CONTROL_MODE 0
#define RESO10BIT (4096)


esp_now_peer_info_t peerInfo;

uint16_t Throttle;
uint16_t Phi, Theta, Psi;
int16_t Phi_bias =0;
int16_t Theta_bias = 0;
int16_t Psi_bias =0;
int16_t Throttle_bias = 0;
short xstick=0;
short ystick=0;
uint8_t Mode=ANGLECONTROL;
uint8_t AltMode=NOT_ALT_CONTROL_MODE;
volatile uint8_t Loop_flag=0;
float Timer = 0.0;
float dTime = 0.01;
uint8_t Timer_state = 0;
uint8_t StickMode = 2;
uint32_t espnow_version;
volatile uint8_t proactive_flag          = 0;
unsigned long stime,etime,dtime;
uint8_t axp_cnt=0;
uint8_t is_peering=0;
uint8_t senddata[25];//19->22->23->24->25
uint8_t disp_counter=0;

//StampFly MAC ADDRESS
//1 F4:12:FA:66:80:54 (Yellow)
//2 F4:12:FA:66:77:A4
uint8_t Addr1[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Addr2[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


//Channel
uint8_t Ch_counter;
volatile uint8_t Received_flag = 0;
volatile uint8_t Channel = CHANNEL;

// Loop synchronization (removed hw_timer, using TDMA semaphore only)
// volatile uint8_t Loop_flag=0;  // Deprecated - use beacon_sem instead

//TDMA
static esp_timer_handle_t beacon_timer;
static SemaphoreHandle_t beacon_sem;
static volatile uint32_t epoch_next_us;

// PLL for synchronization
static volatile int32_t pll_error_us = 0;      // Phase error in microseconds
static volatile int32_t pll_integral = 0;       // Integral term for PLL
static const float PLL_KP = 0.1;                // Proportional gain
static const float PLL_KI = 0.01;               // Integral gain
static const int32_t PLL_ERROR_CLAMP = 1000;    // Max error for P term: ±1ms
static const int32_t PLL_RESYNC_THRESHOLD = 5000; // Resync if error > 5ms
static volatile bool first_beacon_received = false; // First beacon flag for slaves

void rc_init(void);
void data_send(void);
void show_battery_info();
void voltage_print(void);

// TDMA beacon timer callback
void IRAM_ATTR beacon_timer_callback(void* arg)
{
    // Give semaphore to signal that it's time to send
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(beacon_sem, &xHigherPriorityTaskWoken);

    if (TDMA_DEVICE_ID == 0) {
        // Master: No PLL correction needed, autonomous timing
        epoch_next_us = esp_timer_get_time() + TDMA_FRAME_US + TDMA_BEACON_ADVANCE_US;
    } else {
        // Slave: Apply PLL correction with clamping
        // Clamp error to prevent excessive correction
        int32_t clamped_error = pll_error_us;
        if (clamped_error > PLL_ERROR_CLAMP) clamped_error = PLL_ERROR_CLAMP;
        if (clamped_error < -PLL_ERROR_CLAMP) clamped_error = -PLL_ERROR_CLAMP;

        int32_t correction = (int32_t)(PLL_KP * clamped_error + PLL_KI * pll_integral);
        epoch_next_us = esp_timer_get_time() + TDMA_FRAME_US + TDMA_BEACON_ADVANCE_US - correction;
    }

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

float limit(float v, float vmin, float vmax)
{
  if (v<vmin)v=vmin;
  if (v>vmax)v=vmax;
  return v;
}

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len)
{
  if (is_peering) {
    if (recv_data[7] == 0xaa && recv_data[8] == 0x55 && recv_data[9] == 0x16 && recv_data[10] == 0x88) {
        Received_flag = 1;
        // TDMA mode: Use manually configured CHANNEL, ignore drone's channel
        // Channel = recv_data[0];  // Disabled for TDMA
        Addr2[0]      = recv_data[1];
        Addr2[1]      = recv_data[2];
        Addr2[2]      = recv_data[3];
        Addr2[3]      = recv_data[4];
        Addr2[4]      = recv_data[5];
        Addr2[5]      = recv_data[6];
        USBSerial.printf("Receive ! (Using CHANNEL=%d)\n", CHANNEL);
    }
  }
  else {
    // Check if this is a beacon packet (for TDMA synchronization)
    // Beacon: exactly 2 bytes with 0xBE 0xAC header
    if (data_len == 2 && recv_data[0] == 0xBE && recv_data[1] == 0xAC) {
      // Beacon packet detected
      if (TDMA_DEVICE_ID != 0) {
        // Slave device received beacon from master
        uint32_t current_time = esp_timer_get_time();

        if (!first_beacon_received) {
          // First beacon: Immediate synchronization without PLL
          epoch_next_us = current_time + TDMA_FRAME_US;
          pll_error_us = 0;
          pll_integral = 0;
          first_beacon_received = true;
          USBSerial.printf("First beacon sync at %u us\n", current_time);
        } else {
          // Subsequent beacons: Normal PLL operation
          int32_t expected_time = epoch_next_us - TDMA_BEACON_ADVANCE_US;
          pll_error_us = current_time - expected_time;

          // Check for large error - indicates lost sync
          if (pll_error_us > PLL_RESYNC_THRESHOLD || pll_error_us < -PLL_RESYNC_THRESHOLD) {
            // Large error detected - resync immediately
            epoch_next_us = current_time + TDMA_FRAME_US;
            pll_integral = 0;  // Reset integral term
            USBSerial.printf("Large error %d us - resyncing\n", pll_error_us);
            pll_error_us = 0;  // Clear error after resync
          } else {
            // Normal PLL update
            pll_integral += pll_error_us;

            // Limit integral term to prevent windup
            if (pll_integral > 10000) pll_integral = 10000;
            if (pll_integral < -10000) pll_integral = -10000;

            // Update next epoch time
            epoch_next_us = current_time + TDMA_FRAME_US;
          }
        }
      }
      // Master device ignores beacon (own echo-back)
      return;
    }

    // Check minimum length for telemetry data
    if (data_len < 2) {
      return;  // Too short, ignore
    }

    //データ受信時に実行したい内容をここに書く。
    float a;
    uint8_t *dummy;
    uint8_t offset = 2;

    //Channel_detected_flag++;
    //if(Channel_detected_flag>10)Channel_detected_flag=10;
    //Serial.printf("Channel=%d  ",Channel);
    dummy=(uint8_t*)&a;
    dummy[0]=recv_data[0];
    dummy[1]=recv_data[1];

    // Filter out invalid packets
    if (dummy[0]==0xF4)return;
    if (dummy[0]==0xBE && dummy[1]==0xAC)return;  // Double-check beacon filter

    // Validate data length before processing telemetry
    // Telemetry format: 2-byte header + N*4-byte floats
    if ((data_len - offset) % 4 != 0 || data_len < (offset + 4)) {
      // Invalid telemetry format, ignore
      return;
    }

    if ((dummy[0]==99)&&(dummy[1]==99))Serial.printf("#PID Gain P Ti Td Eta ");

    uint8_t num_floats = (data_len-offset)/4;
    USBSerial.printf("%d ", num_floats);

    for (uint8_t i=0; i < num_floats; i++)
    {
      // Bounds check before accessing array
      uint16_t base_idx = i*4 + offset;
      if (base_idx + 3 < data_len) {
        dummy[0]=recv_data[base_idx + 0];
        dummy[1]=recv_data[base_idx + 1];
        dummy[2]=recv_data[base_idx + 2];
        dummy[3]=recv_data[base_idx + 3];
        USBSerial.printf("%9.4f ", a);
      }
    }
    USBSerial.printf("\r\n");
  }
}

#define BUF_SIZE 128
// EEPROMにデータを保存する
void save_data(void)
{
  SPIFFS.begin(true);
  /* CREATE FILE */
  File fp = SPIFFS.open("/peer_info.txt", FILE_WRITE); // 書き込み、存在すれば上書き
  char buf[BUF_SIZE + 1];
  // TDMA mode: Always save CHANNEL define value (not variable Channel)
  sprintf(buf, "%d,%02X,%02X,%02X,%02X,%02X,%02X",
          CHANNEL,  // Use define value, not variable
          Addr2[0],
          Addr2[1],
          Addr2[2],
          Addr2[3],
          Addr2[4],
          Addr2[5]);
  fp.write((uint8_t *)buf, BUF_SIZE);
  fp.close();
  SPIFFS.end();

  USBSerial.printf("Saved Data:%d,[%02X:%02X:%02X:%02X:%02X:%02X]",
      CHANNEL,  // Use define value
      Addr2[0],
      Addr2[1],
      Addr2[2],
      Addr2[3],
      Addr2[4],
      Addr2[5]);
}

// EEPROMからデータを読み出す
void load_data(void)
{
  SPIFFS.begin(true);
  File fp = SPIFFS.open("/peer_info.txt", FILE_READ);
  char buf[BUF_SIZE + 1];
  while (fp.read((uint8_t *)buf, BUF_SIZE) == BUF_SIZE)
  {
    //USBSerial.print(buf);
    uint8_t saved_channel;  // Temporary variable for saved channel
    sscanf(buf,"%hhd,%hhX,%hhX,%hhX,%hhX,%hhX,%hhX",
          &saved_channel,  // Read but don't use for TDMA
          &Addr2[0],
          &Addr2[1],
          &Addr2[2],
          &Addr2[3],
          &Addr2[4],
          &Addr2[5]);
    // TDMA mode: Always use CHANNEL define, ignore saved channel
    Channel = CHANNEL;
    USBSerial.printf("Loaded MAC (using CHANNEL=%d): %02X:%02X:%02X:%02X:%02X:%02X\n\r",
          CHANNEL,
          Addr2[0],
          Addr2[1],
          Addr2[2],
          Addr2[3],
          Addr2[4],
          Addr2[5]);
  }
  fp.close();
  SPIFFS.end();
}

void rc_init(uint8_t ch, uint8_t* addr)
{  
    // ESP-NOW初期化
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        esp_now_unregister_recv_cb();
        esp_now_register_recv_cb(OnDataRecv);
        USBSerial.println("ESPNow Init Success");
    } else {
        USBSerial.println("ESPNow Init Failed");
        ESP.restart();
    }

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, addr, 6);
    peerInfo.channel = ch;
    peerInfo.encrypt = false;
    uint8_t peer_mac_addre;
    while (esp_now_add_peer(&peerInfo) != ESP_OK) {
        USBSerial.println("Failed to add peer");
    }
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

void peering(void)
{
  uint8_t break_flag;
  uint32_t beep_delay = 0;
  //StampFlyはMACアドレスをFF:FF:FF:FF:FF:FFとして
  //StampFlyのMACアドレスをでブロードキャストする
  //その際にChannelが機体と送信機で同一でない場合は受け取れない
  // ESP-NOWコールバック登録
  esp_now_register_recv_cb(OnDataRecv);

  //ペアリング
  // TDMA mode: Use only the manually configured CHANNEL
  USBSerial.printf("TDMA: Using fixed channel %02d.\n\r", CHANNEL);
  peerInfo.channel = CHANNEL;
  peerInfo.encrypt = false;
  while (esp_now_mod_peer(&peerInfo) != ESP_OK)
  {
      USBSerial.println("Failed to mod peer");
  }
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

  //Wait receive StampFly MAC Address on the configured channel
  while(1)
  {
    for (uint8_t i =0;i<100;i++)
    {
          break_flag = 0;
          if (Received_flag == 1)
          {
            break_flag = 1;
            break;
          }
          usleep(100);
    }
    if (millis() - beep_delay >= 500) {
        beep();
        beep_delay = millis();
    }

    if (break_flag)break;
  }
  // Force Channel to be CHANNEL define value
  Channel = CHANNEL;

  save_data();
  is_peering = 0;
  USBSerial.printf("Channel:%02d\n\r", Channel);
  USBSerial.printf("MAC2:%02X:%02X:%02X:%02X:%02X:%02X:\n\r",
                    Addr2[0],Addr2[1],Addr2[2],Addr2[3],Addr2[4],Addr2[5]);
  USBSerial.printf("MAC1:%02X:%02X:%02X:%02X:%02X:%02X:\n\r",
                    Addr1[0],Addr1[1],Addr1[2],Addr1[3],Addr1[4],Addr1[5]);

  //Peering
  while (esp_now_del_peer(Addr1) != ESP_OK) {
    Serial.println("Failed to delete peer1");
  }
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, Addr2, 6);//Addr1->Addr2 ////////////////////////////
  peerInfo.channel = Channel;
  peerInfo.encrypt = false;
  while (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
        USBSerial.println("Failed to add peer2");
  }  
  esp_wifi_set_channel(Channel, WIFI_SECOND_CHAN_NONE);
}

void change_channel(uint8_t ch)
{
  peerInfo.channel = ch;
  if (esp_now_mod_peer(&peerInfo)!=ESP_OK)
  {
        USBSerial.println("Failed to modify peer");
        return;
  }
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

// hw_timer removed - using esp_timer (beacon_timer) for synchronization

void setup() {
  M5.begin();
  Wire1.begin(38, 39, 400*1000);
  load_data();
  M5.update();
  setup_pwm_buzzer();
  M5.Lcd.setRotation( 2 );
  M5.Lcd.setTextFont(2);
  M5.Lcd.setCursor(4, 2);
  
  if (M5.Btn.isPressed() || (Addr2[0] == 0xFF && Addr2[1] == 0xFF && Addr2[2] == 0xFF && Addr2[3] == 0xFF &&
                               Addr2[4] == 0xFF && Addr2[5] == 0xFF)) {
    M5.Lcd.println("Push LCD panel!");
    while (1) {
      M5.update();
      if (M5.Btn.wasPressed()) {
        is_peering = 1;
        break;
      }
    }
    rc_init(CHANNEL, Addr1);  // TDMA: Use CHANNEL define
    USBSerial.printf("Button pressed!\n\r");
    M5.Lcd.println(" ");
    M5.Lcd.println("Push StampFly");
    M5.Lcd.println("    Reset Button!");
    M5.Lcd.println(" ");
    M5.Lcd.println("Pairing...");
    peering();
  }
  else rc_init(CHANNEL, Addr2);  // TDMA: Use CHANNEL define
  M5.Lcd.fillScreen(BLACK);
  joy_update();

  StickMode = 2;
  if(getOptionButton())
  {
    StickMode = 3;
    M5.Lcd.println("Please release button.");
    while(getOptionButton())joy_update();
  }
  AltMode =NOT_ALT_CONTROL_MODE;
  delay(500);

  if (StickMode == 3)
  {
    THROTTLE = RIGHTY;
    AILERON = LEFTX;
    ELEVATOR = LEFTY;
    RUDDER = RIGHTX;
    ARM_BUTTON = RIGHT_STICK_BUTTON;
    FLIP_BUTTON = LEFT_STICK_BUTTON;
    MODE_BUTTON = RIGHT_BUTTON;
    OPTION_BUTTON = LEFT_BUTTON;
  }
  else
  {
    THROTTLE = LEFTY;
    AILERON = RIGHTX;
    ELEVATOR = RIGHTY;
    RUDDER = LEFTX;
    ARM_BUTTON = LEFT_STICK_BUTTON;
    FLIP_BUTTON = RIGHT_STICK_BUTTON;
    MODE_BUTTON = RIGHT_BUTTON;
    OPTION_BUTTON = LEFT_BUTTON;
  }

  uint8_t error, address;
  int nDevices;

////////////////////////////////////////////////////////
  USBSerial.println("Scanning... Wire1");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
      USBSerial.print("I2C device found at address 0x");
      if (address < 16)
        USBSerial.print("0");
      USBSerial.print(address, HEX);
      USBSerial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      USBSerial.print("Unknown error at address 0x");
      if (address < 16)
        USBSerial.print("0");
      USBSerial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    USBSerial.println("No I2C devices found\n");
  else
    USBSerial.println("done\n");

  esp_now_get_version(&espnow_version);
  USBSerial.printf("ESP-NOW Version %d\n", espnow_version);

  // TDMA初期化 (hw_timer removed - using esp_timer only)
  beacon_sem = xSemaphoreCreateBinary();
  if (beacon_sem == NULL) {
    USBSerial.println("Failed to create beacon semaphore");
  }

  // TDMA timer setup
  esp_timer_create_args_t beacon_timer_args;
  beacon_timer_args.callback = &beacon_timer_callback;
  beacon_timer_args.arg = NULL;
  beacon_timer_args.dispatch_method = ESP_TIMER_TASK;
  beacon_timer_args.name = "beacon_timer";

  esp_err_t err = esp_timer_create(&beacon_timer_args, &beacon_timer);
  if (err != ESP_OK) {
    USBSerial.printf("Failed to create TDMA timer: %d\n", err);
  }

  // Initialize epoch time
  epoch_next_us = esp_timer_get_time() + TDMA_FRAME_US;

  // Start TDMA timer (both master and slave use it for synchronization)
  err = esp_timer_start_periodic(beacon_timer, TDMA_FRAME_US);
  if (err != ESP_OK) {
    USBSerial.printf("Failed to start TDMA timer: %d\n", err);
  } else {
    if (TDMA_DEVICE_ID == 0) {
      USBSerial.printf("TDMA Master started (ID=%d)\n", TDMA_DEVICE_ID);
    } else {
      USBSerial.printf("TDMA Slave started (ID=%d)\n", TDMA_DEVICE_ID);
    }
  }
}

uint8_t check_control_mode_change(void)
{
  uint8_t state;
  static uint8_t flag =0;
  state = 0;
  if (flag==0)
  {
    if (getModeButton() == 1)
    {
      flag = 1;
    }
  }
  else
  {
    if (getModeButton() == 0)
    {
      flag = 0;
      state = 1;
    }
  }
  //USBSerial.printf("%d %d\n\r", state, flag);
  return state;
}

uint8_t check_alt_mode_change(void)
{
  uint8_t state;
  static uint8_t flag =0;
  state = 0;
  if (flag==0)
  {
    if (getOptionButton() == 1)
    {
      flag = 1;
    }
  }
  else
  {
    if (getOptionButton() == 0)
    {
      flag = 0;
      state = 1;
    }
  }
  //USBSerial.printf("%d %d\n\r", state, flag);
  return state;
}


uint8_t average_counter = 0;
void loop() {
  int16_t _throttle;// = getThrottle();
  int16_t _phi;// = getAileron();
  int16_t _theta;// = getElevator();
  int16_t _psi;// = getRudder();
  static uint8_t loop_counter = 0;

  // Removed Loop_flag wait - synchronization now handled by TDMA semaphore
  etime = stime;
  stime = micros();
  dtime = stime - etime;
  loop_counter++;
  M5.update();
  joy_update();

  //Stop Watch Start&Stop&Reset  
  if(M5.Btn.wasPressed()==true)
  {
    if (Timer_state == 0)Timer_state = 1;
    else if (Timer_state == 1)Timer_state = 0;
  }

  if(M5.Btn.pressedFor(400)==true)
  {
    Timer_state = 2;
  }

  if (Timer_state == 1)
  {
    //カウントアップ
    Timer = Timer + dTime;
  }
  else if (Timer_state == 2)
  {
    //タイマリセット
    Timer = 0.0;
    Timer_state = 0;
  }

  if (check_control_mode_change() == 1)
  {
    if (Mode==ANGLECONTROL)Mode=RATECONTROL;
    else Mode = ANGLECONTROL;
  }

  if (check_alt_mode_change() == 1)
  {
    if (AltMode==ALT_CONTROL_MODE)AltMode=NOT_ALT_CONTROL_MODE;
    else AltMode = ALT_CONTROL_MODE;
  }

  _throttle = getThrottle();
  _phi = getAileron();
  _theta = getElevator();
  _psi = getRudder();


  if(average_counter<50)
  {
    average_counter++;
    Throttle_bias += (_throttle - 2048);
    Phi_bias += (_phi - 2048);
    Theta_bias += (_theta - 2048);
    Psi_bias += (_psi - 2048);
  }
  else if (average_counter==50)
  {
    average_counter++;
    Throttle_bias = Throttle_bias/50;
    Phi_bias = Phi_bias/50;
    Theta_bias = Theta_bias/50;
    Psi_bias = Psi_bias/50;
    average_counter=51;
  }
  else
  {
    _throttle -= Throttle_bias;
    _phi -= Phi_bias;
    _theta -= Theta_bias;
    _psi -= Psi_bias;
  }

  //量産版
  //Throttle = -(float)(_throttle - 2048)/(float)(RESO10BIT*0.5);
  //Phi =       (float)(_phi - 2048)/(float)(RESO10BIT*0.5); 
  //Theta =     (float)(_theta - 2048)/(float)(RESO10BIT*0.5);
  //Psi =       (float)(_psi - 2048)/(float)(RESO10BIT*0.5);
  Throttle = 4095 - _throttle;
  Phi = _phi; 
  Theta = _theta;
  Psi = _psi;



  //最終試作版
  #if 0
  Throttle = (float)(_throttle - 2048)/(float)(RESO10BIT*0.5);
  Phi =      (float)(_phi - 2048)/(float)(RESO10BIT*0.5); 
  Theta =   -(float)(_theta - 2048)/(float)(RESO10BIT*0.5);
  Psi =      (float)(_psi - 2048)/(float)(RESO10BIT*0.5);
  #endif

  //Throttle = limit(Throttle, -1.0, 1.0);
  //Phi = limit(Phi, -1.0, 1.0);
  //Theta = limit(Theta, -1.0, 1.0);
  //Psi = limit(Psi, -1.0, 1.0);


  uint8_t* d_int;
  
  //ブロードキャストの混信を防止するためこの機体のMACアドレスに送られてきたものか判断する
  senddata[0] = peerInfo.peer_addr[3];////////////////////////////
  senddata[1] = peerInfo.peer_addr[4];////////////////////////////
  senddata[2] = peerInfo.peer_addr[5];////////////////////////////

  d_int = (uint8_t*)&Throttle;
  senddata[3]=d_int[0];
  senddata[4]=d_int[1];

  d_int = (uint8_t*)&Phi;
  senddata[5]=d_int[0];
  senddata[6]=d_int[1];

  d_int = (uint8_t*)&Theta;
  senddata[7]=d_int[0];
  senddata[8]=d_int[1];

  d_int = (uint8_t*)&Psi;
  senddata[9]=d_int[0];
  senddata[10]=d_int[1];

  senddata[11]=(0x01&AltMode)<<3|(0x01&Mode)<<2|(0x0001&getFlipButton())<<1|(0x0001&getArmButton());
  //senddata[11]=getArmButton();
  //senddata[12]=getFlipButton();
  //senddata[13]=Mode;
  //senddata[14]=AltMode;
  senddata[12]=proactive_flag;
  
  //checksum
  senddata[13]=0;
  for(uint8_t i=0;i<13;i++)senddata[13]=senddata[13]+senddata[i];

  // TDMA synchronized transmission
  // Wait for beacon timer semaphore
  if (xSemaphoreTake(beacon_sem, pdMS_TO_TICKS(20)) == pdTRUE) {
    // Semaphore received - it's time to send

    // Master device sends beacon first
    if (TDMA_DEVICE_ID == 0) {
      // Send beacon packet
      uint8_t beacon_data[2] = {0xBE, 0xAC};  // Beacon marker
      esp_now_send(peerInfo.peer_addr, beacon_data, sizeof(beacon_data));
      delayMicroseconds(TDMA_BEACON_ADVANCE_US);  // Wait until frame start
    }

    // Calculate slot start time
    uint32_t slot_start_us = epoch_next_us - TDMA_BEACON_ADVANCE_US + (TDMA_DEVICE_ID * TDMA_SLOT_US);
    uint32_t current_us = esp_timer_get_time();

    // Precise microsecond timing for TDMA slot synchronization
    if (slot_start_us > current_us) {
      int32_t wait_us = slot_start_us - current_us;

      // If wait time > 20us, use delayMicroseconds for precise timing
      if (wait_us > 20) {
        delayMicroseconds(wait_us - 10);  // Wait until 10us before slot start
      }

      // Final precise timing with busy wait
      while (esp_timer_get_time() < slot_start_us) {
        // Busy wait for ultimate precision
      }
    }

    // Send control data in our assigned slot
    esp_err_t result = esp_now_send(peerInfo.peer_addr, senddata, sizeof(senddata));
  } else {
    // Timeout - send anyway (fallback for non-TDMA mode)
    esp_err_t result = esp_now_send(peerInfo.peer_addr, senddata, sizeof(senddata));
  }
  #ifdef DEBUG
  USBSerial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
    peerInfo.peer_addr[0],
    peerInfo.peer_addr[1],
    peerInfo.peer_addr[2],
    peerInfo.peer_addr[3],
    peerInfo.peer_addr[4],
    peerInfo.peer_addr[5]);
  #endif
  //Display information
  //float vbat =0.0;// M5.Axp.GetBatVoltage();
  //int8_t bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  
  M5.Lcd.setCursor(4, 2+disp_counter*17);
  switch (disp_counter)
  {
    case 0:
      M5.Lcd.printf("MAC ADR %02X:%02X    ", peerInfo.peer_addr[4],peerInfo.peer_addr[5]);
      break;
    case 1:
      M5.Lcd.printf("BAT 1:%4.1f 2:%4.1f", Battery_voltage[0],Battery_voltage[1]);
      //M5.Lcd.printf("X:%4d",xstick);
      break;
    case 2:
      #ifdef NEW_ATOM_JOY
      M5.Lcd.printf("MODE: %d", StickMode);
      //M5.Lcd.printf("X:%4d",xstick);
      #endif
      break;
    case 3:
      M5.Lcd.printf("CHL: %02d",peerInfo.channel);
      break;
    case 4:
      if( AltMode == ALT_CONTROL_MODE ) M5.Lcd.printf("-Auto ALT-  ");
      else if ( AltMode == NOT_ALT_CONTROL_MODE )   M5.Lcd.printf("-Mnual ALT- ");
      break;
    case 5:
      if( Mode == ANGLECONTROL )      M5.Lcd.printf("-STABILIZE-");
      else if ( Mode == RATECONTROL ) M5.Lcd.printf("-ACRO-     ");
      break;
    case 6:
      //M5.Lcd.printf("Time:%7.2f",Timer);
      M5.Lcd.printf("Freq:%4d  [%3d] ",1000000/dtime, loop_counter);

      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
  }
  disp_counter++;
  if(disp_counter==11)disp_counter=0;

  //Reset
  if( /*M5.Axp.GetBtnPress() == 2*/ 0 ){
    // 電源ボタンクリック
    //M5.Lcd.println("AtomFly2.0"); 
    esp_restart();
  } 

}

void show_battery_info(){
  #if 0
  // バッテリー電圧表示
  double vbat = 0.0;
  int8_t bat_charge_p = 0;

  vbat = M5.Axp.GetBatVoltage();
  M5.Lcd.setCursor(5, 100);
  //M5.Lcd.setTextSize(1);
  M5.Lcd.printf("Volt:\n %8.2fV", vbat);

  // バッテリー残量表示
  bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  M5.Lcd.setCursor(5, 140);
  M5.Lcd.printf("Charge:\n %8d%%", bat_charge_p);
#endif
}

void voltage_print(void)
{

  M5.Lcd.setCursor(0, 17, 2);
  M5.Lcd.printf("%3.1fV", Battery_voltage);
}
