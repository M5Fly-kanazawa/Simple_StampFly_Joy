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

esp_now_peer_info_t dronePeer; // Peer for drone communication
esp_now_peer_info_t beaconPeer;  // Peer for beacon multicast

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
volatile uint8_t proactive_flag = 0;
unsigned long stime, etime, dtime;
uint8_t axp_cnt = 0;
uint8_t is_peering = 0;
uint8_t senddata[14]; //19->22->23->24->25->14
uint8_t disp_counter = 0;

//StampFly MAC ADDRESS
uint8_t Drone_mac[6] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
// Broadcast address for TDMA beacons
uint8_t Beacon_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Channel
uint8_t Ch_counter;
volatile uint8_t Received_flag = 0;
volatile uint8_t Channel = CHANNEL;

// Loop synchronization (removed hw_timer, using TDMA semaphore only)
// volatile uint8_t Loop_flag=0;  // Deprecated - use beacon_sem instead

//TDMA
static esp_timer_handle_t beacon_timer;
static SemaphoreHandle_t beacon_sem;
static TaskHandle_t beacon_task_handle = NULL;  // Task for beacon transmission
static volatile int64_t epoch_next_us;  // 64-bit for overflow protection

// PLL for synchronization
static volatile int32_t pll_error_us = 0;      // Phase error in microseconds
static volatile int32_t pll_integral = 0;       // Integral term for PLL
static const float PLL_KP = 0.1;                // Proportional gain
static const float PLL_KI = 0.01;               // Integral gain
static const int32_t PLL_ERROR_CLAMP = 500;     // Max error for P term: ±0.5ms (half slot)
static const int32_t PLL_RESYNC_THRESHOLD = 800; // Resync if error > 0.8ms (80% of slot)
static volatile bool first_beacon_received = false; // First beacon flag for slaves
static volatile int64_t last_beacon_time_us = 0;    // Last beacon reception time (64-bit for overflow protection)
static const uint32_t BEACON_TIMEOUT_US = 50000;    // 50ms = 5 frames

// Function declarations
void wifi_esp_now_init(void);
void broadcast_beacon_init(void);
void drone_peer_init(void);
void peering(void);
void change_channel(uint8_t ch);
void save_data(void);
void load_data(void);
void data_send(void);
void show_battery_info();
void voltage_print(void);
void beacon_task(void* parameter);
void IRAM_ATTR beacon_timer_callback(void* arg);
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
float limit(float v, float vmin, float vmax);
uint8_t check_control_mode_change(void);
uint8_t check_alt_mode_change(void);

// Send callback statistics
static volatile uint32_t send_success_count = 0;
static volatile uint32_t send_fail_count = 0;
static volatile uint32_t beacon_cb_success = 0;
static volatile uint32_t beacon_cb_fail = 0;
static volatile uint32_t drone_cb_success = 0;
static volatile uint32_t drone_cb_fail = 0;

// Drone connectivity detection
static volatile uint32_t drone_consecutive_failures = 0;
static volatile bool drone_available = true;  // Assume available at start
static const uint32_t DRONE_FAILURE_THRESHOLD = 50;  // 50 consecutive failures = drone offline

// Beacon transmission task (for master only)
// This task waits for notification from timer ISR and sends beacon
void beacon_task(void* parameter)
{
    uint8_t beacon_data[2] = {0xBE, 0xAC};
    static int64_t last_beacon_sent_us = 0;
    static uint32_t beacon_count = 0;
    static uint32_t error_count = 0;
    static uint32_t last_re_init = 0;

    while (1) {
        // Wait for notification from timer ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Check if beacon peer exists in peer table before sending
        if (!esp_now_is_peer_exist(beaconPeer.peer_addr)) {
            // Peer not found - try to re-register
            USBSerial.printf("Beacon peer not found! Re-registering...\n");
            broadcast_beacon_init();
            beacon_count++;
            continue;  // Skip this beacon and wait for next timer
        }

        // Send beacon immediately upon notification (no delay from logging)
        esp_err_t result = esp_now_send(beaconPeer.peer_addr, beacon_data, sizeof(beacon_data));

        // Handle send errors
        if (result != ESP_OK) {
            error_count++;

            // ESP_ERR_ESPNOW_NOT_FOUND (12391 = 0x3067) - peer not found
            if (result == ESP_ERR_ESPNOW_NOT_FOUND || result == 12391) {
                // Double-check if peer really exists (debugging)
                bool peer_exists = esp_now_is_peer_exist(beaconPeer.peer_addr);
                if (beacon_count % 10 == 0 || beacon_count <= 100) {
                    USBSerial.printf("ESP_ERR_ESPNOW_NOT_FOUND but peer_exists=%d at #%u\n",
                                   peer_exists, beacon_count);
                }
                // Retry immediately once
                delayMicroseconds(50);
                result = esp_now_send(beaconPeer.peer_addr, beacon_data, sizeof(beacon_data));

                // If still failing, re-initialize peer (once per second max)
                if (result != ESP_OK && beacon_count - last_re_init > 100) {
                    // Try to get peer info before deleting
                    esp_now_peer_info_t peer_info;
                    esp_err_t get_result = esp_now_get_peer(beaconPeer.peer_addr, &peer_info);
                    USBSerial.printf("Peer get_result=%d (0=found) at #%u\n", get_result, beacon_count);

                    if (get_result == ESP_OK) {
                        USBSerial.printf("Peer details: ch=%d, ifidx=%d, encrypt=%d\n",
                                       peer_info.channel, peer_info.ifidx, peer_info.encrypt);
                    }

                    esp_now_del_peer(beaconPeer.peer_addr);  // Remove old peer
                    delay(10);
                    broadcast_beacon_init();  // Re-register
                    last_re_init = beacon_count;
                    USBSerial.printf("Beacon peer re-init at #%u (err=%d, total_err=%u)\n",
                                   beacon_count, result, error_count);
                }
            }
            // ESP_ERR_ESPNOW_NO_MEM or other errors
            else {
                // Just log other errors (buffer full, etc.)
                if (beacon_count % 100 == 0) {
                    USBSerial.printf("Beacon send error: %d at #%u\n", result, beacon_count);
                }
            }
        }

        beacon_count++;

        #if 1  // Enable logging for debugging
        int64_t current_time = esp_timer_get_time();
        int64_t interval_us = current_time - last_beacon_sent_us;

        // Log first 100 beacons with detailed interval timing
        if (beacon_count <= 100) {
            USBSerial.printf("Beacon TX #%u: interval=%lld us, result=%d\n",
                           beacon_count, interval_us, result);
        }
        // Then log every 100 beacons to monitor stability
        else if (beacon_count % 100 == 0) {
            USBSerial.printf("Beacon TX #%u: interval=%lld us, result=%d, errors=%u\n",
                           beacon_count, interval_us, result, error_count);
        }
        // Every 1000 beacons show full statistics
        if (beacon_count % 1000 == 0) {
            USBSerial.printf("  Total CB: ok=%u fail=%u | Beacon CB: ok=%u fail=%u | Drone CB: ok=%u fail=%u\n",
                           send_success_count, send_fail_count,
                           beacon_cb_success, beacon_cb_fail,
                           drone_cb_success, drone_cb_fail);
        }

        last_beacon_sent_us = current_time;
        #endif
    }
}

// TDMA beacon timer callback
// Note: Cannot call esp_now_send() here - not ISR safe!
void IRAM_ATTR beacon_timer_callback(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint32_t callback_count = 0;
    static int64_t last_callback_time = 0;
    callback_count++;

    // Measure timer callback interval (master only, for debugging)
    #if 1
    if (TDMA_DEVICE_ID == 0 && callback_count <= 100) {
        int64_t now = esp_timer_get_time();
        int64_t timer_interval = now - last_callback_time;
        if (callback_count > 1) {
            // Use ets_printf for ISR-safe logging
            ets_printf("Timer CB #%u: interval=%lld us\n", callback_count, timer_interval);
        }
        last_callback_time = now;
    }
    #endif

    // Master: Notify beacon task to send beacon immediately
    if (TDMA_DEVICE_ID == 0 && beacon_task_handle != NULL) {
        vTaskNotifyGiveFromISR(beacon_task_handle, &xHigherPriorityTaskWoken);
    }

    // Update epoch time for next frame
    if (TDMA_DEVICE_ID == 0) {
        // Master: Autonomous timing based on timer callback
        epoch_next_us = esp_timer_get_time() + TDMA_FRAME_US + TDMA_BEACON_ADVANCE_US;
    }
    // Slave: epoch_next_us is updated ONLY in beacon reception callback
    // Do NOT update epoch_next_us here, as slave timer is not synchronized with master

    // Give semaphore to signal that it's time to send control data
    xSemaphoreGiveFromISR(beacon_sem, &xHigherPriorityTaskWoken);

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

// 送信コールバック
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Check if this is beacon (FF:FF:FF:FF:FF:FF) or drone packet
  bool is_beacon = (mac_addr[0] == 0xFF && mac_addr[1] == 0xFF &&
                    mac_addr[2] == 0xFF && mac_addr[3] == 0xFF &&
                    mac_addr[4] == 0xFF && mac_addr[5] == 0xFF);

  // Track send statistics
  if (status == ESP_NOW_SEND_SUCCESS) {
    send_success_count++;
    if (is_beacon) {
      beacon_cb_success++;
    } else {
      drone_cb_success++;
      drone_consecutive_failures = 0;  // Reset failure counter on success
      if (!drone_available) {
        drone_available = true;  // Drone is back online
        USBSerial.printf("Drone reconnected!\n");
      }
    }
  } else {
    send_fail_count++;
    if (is_beacon) {
      beacon_cb_fail++;
    } else {
      drone_cb_fail++;
      drone_consecutive_failures++;

      // Check if drone should be marked offline
      if (drone_available && drone_consecutive_failures >= DRONE_FAILURE_THRESHOLD) {
        drone_available = false;
        USBSerial.printf("Drone offline detected (failures=%u). Stopping drone transmissions.\n",
                       drone_consecutive_failures);
      }
    }

    // Log send failures for debugging (rate limited)
    static uint32_t last_log_time = 0;
    uint32_t now = millis();
    if (now - last_log_time > 1000) {  // Log once per second max
      if (is_beacon) {
        USBSerial.printf("Send CB: FAIL BEACON (beacon_ok=%u, beacon_fail=%u)\n",
                       beacon_cb_success, beacon_cb_fail);
      } else {
        USBSerial.printf("Send CB: FAIL DRONE %02X:%02X:%02X:%02X:%02X:%02X (drone_ok=%u, drone_fail=%u)\n",
                       mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
                       drone_cb_success, drone_cb_fail);
      }
      last_log_time = now;
    }
  }
}

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len)
{
  if (is_peering) {
    if (recv_data[7] == 0xaa && recv_data[8] == 0x55 && recv_data[9] == 0x16 && recv_data[10] == 0x88) {
        Received_flag = 1;
        // TDMA mode: Use manually configured CHANNEL, ignore drone's channel
        // Channel = recv_data[0];  // Disabled for TDMA
        Drone_mac[0]      = recv_data[1];
        Drone_mac[1]      = recv_data[2];
        Drone_mac[2]      = recv_data[3];
        Drone_mac[3]      = recv_data[4];
        Drone_mac[4]      = recv_data[5];
        Drone_mac[5]      = recv_data[6];
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
        int64_t current_time = esp_timer_get_time();
        last_beacon_time_us = current_time;  // Update last beacon time

        #if 1  // Enable logging to verify beacon reception on slave
        static int64_t last_recv_time = 0;
        int64_t interval_us = current_time - last_recv_time;
        static uint32_t beacon_recv_count = 0;
        beacon_recv_count++;

        // Calculate expected time and error BEFORE processing (for accurate logging)
        int64_t expected_time_for_log = 0;
        int32_t error_for_log = 0;
        if (beacon_recv_count > 1) {
          expected_time_for_log = epoch_next_us;
          error_for_log = current_time - expected_time_for_log;
        }

        // Log first 100 beacons with detailed info
        if (beacon_recv_count <= 100) {
          if (beacon_recv_count == 1) {
            USBSerial.printf("Beacon RX #%u: interval=%lld us (first beacon)\n",
                           beacon_recv_count, interval_us);
          } else {
            USBSerial.printf("Beacon RX #%u: interval=%lld us, error=%d us\n",
                           beacon_recv_count, interval_us, error_for_log);
          }
        }
        // Then log every 100 beacons (every 1 second)
        else if (beacon_recv_count % 100 == 0) {
          USBSerial.printf("Beacon RX #%u: interval=%lld us, error=%d us\n",
                         beacon_recv_count, interval_us, error_for_log);
        }

        last_recv_time = current_time;
        #endif

        if (!first_beacon_received) {
          // First beacon: Immediate synchronization without PLL
          epoch_next_us = current_time + TDMA_FRAME_US;
          pll_error_us = 0;
          pll_integral = 0;
          first_beacon_received = true;
          // USBSerial.printf("First beacon sync\n");  // Disabled to prevent timing jitter
        } else {
          // Subsequent beacons: Calculate PLL error and update epoch_next_us
          int32_t expected_time = epoch_next_us;
          pll_error_us = current_time - expected_time;

          // Check for large error - indicates lost sync
          if (pll_error_us > PLL_RESYNC_THRESHOLD || pll_error_us < -PLL_RESYNC_THRESHOLD) {
            // Large error detected - resync immediately
            static uint32_t resync_count = 0;
            resync_count++;

            // Log resync with original error (before clearing)
            if (resync_count <= 100 || resync_count % 100 == 0) {
              USBSerial.printf("RESYNC #%u: error=%d us (threshold=±%d us)\n",
                             resync_count, (int)pll_error_us, PLL_RESYNC_THRESHOLD);
            }

            // Resync: next beacon expected in exactly 10ms
            epoch_next_us = current_time + TDMA_FRAME_US;
            pll_integral = 0;  // Reset integral term
            pll_error_us = 0;  // Clear error after resync
          } else {
            // Normal PLL update
            pll_integral += pll_error_us;

            // Limit integral term to prevent windup
            if (pll_integral > 10000) pll_integral = 10000;
            if (pll_integral < -10000) pll_integral = -10000;

            // Apply PLL correction to next expected beacon time
            // Clamp error to prevent excessive correction
            int32_t clamped_error = pll_error_us;
            if (clamped_error > PLL_ERROR_CLAMP) clamped_error = PLL_ERROR_CLAMP;
            if (clamped_error < -PLL_ERROR_CLAMP) clamped_error = -PLL_ERROR_CLAMP;

            int32_t correction = (int32_t)(PLL_KP * clamped_error + PLL_KI * pll_integral);

            // Update epoch_next_us: base time (10ms from now) + PLL correction
            // Correction is subtracted because:
            //   - If error > 0 (late), correction > 0, so we ADD to next expected time (wait longer)
            //   - If error < 0 (early), correction < 0, so we SUBTRACT from next expected time (wait less)
            // But we want the opposite: if beacon came early, expect it earlier next time
            epoch_next_us = current_time + TDMA_FRAME_US - correction;
          }
        }
      }
      // Master device ignores beacon (own echo-back)
      return;
    }

    #if 0
    //テレメトリーデータ受信
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
    #endif
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
          Drone_mac[0],
          Drone_mac[1],
          Drone_mac[2],
          Drone_mac[3],
          Drone_mac[4],
          Drone_mac[5]);
  fp.write((uint8_t *)buf, BUF_SIZE);
  fp.close();
  SPIFFS.end();

  USBSerial.printf("Saved Data:%d,[%02X:%02X:%02X:%02X:%02X:%02X]",
      CHANNEL,  // Use define value
      Drone_mac[0],
      Drone_mac[1],
      Drone_mac[2],
      Drone_mac[3],
      Drone_mac[4],
      Drone_mac[5]);
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
          &Drone_mac[0],
          &Drone_mac[1],
          &Drone_mac[2],
          &Drone_mac[3],
          &Drone_mac[4],
          &Drone_mac[5]);
    // TDMA mode: Always use CHANNEL define, ignore saved channel
    Channel = CHANNEL;
    USBSerial.printf("Loaded MAC (using CHANNEL=%d): %02X:%02X:%02X:%02X:%02X:%02X\n\r",
          CHANNEL,
          Drone_mac[0],
          Drone_mac[1],
          Drone_mac[2],
          Drone_mac[3],
          Drone_mac[4],
          Drone_mac[5]);
  }
  fp.close();
  SPIFFS.end();
}

void wifi_esp_now_init(void)
{  
    // ESP-NOW初期化
    //WiFi.mode(WIFI_STA);
    //WiFi.disconnect();

    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);                      // 省電力OFF推奨
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);


    //esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() == ESP_OK) {
        esp_now_unregister_recv_cb();
        esp_now_register_recv_cb(OnDataRecv);
        esp_now_register_send_cb(OnDataSent);  // Register send callback
        USBSerial.println("ESPNow Init Success");
    } else {
        USBSerial.println("ESPNow Init Failed");
        ESP.restart();
    }
  // ESP-NOWコールバック登録
  //esp_now_register_recv_cb(OnDataRecv);

#if 0
    memset(&dronePeer, 0, sizeof(dronePeer));
    memcpy(dronePeer.peer_addr, addr, 6);
    dronePeer.channel = ch;
    dronePeer.encrypt = false;
    uint8_t peer_mac_addre;
    while (esp_now_add_peer(&dronePeer) != ESP_OK) {
        USBSerial.println("Failed to add peer");
    }
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
#endif
}

void broadcast_beacon_init(void)
{
    memset(&beaconPeer, 0, sizeof(beaconPeer));
    memcpy(beaconPeer.peer_addr, Beacon_mac, 6);
    beaconPeer.channel = CHANNEL;
    beaconPeer.encrypt = false;
    beaconPeer.ifidx = WIFI_IF_STA;  // Explicitly set interface

    // Try to add peer, retry if failed
    esp_err_t result = esp_now_add_peer(&beaconPeer);
    uint8_t retry = 0;
    while (result != ESP_OK && retry < 5) {
        USBSerial.printf("Failed to add beacon peer: %d (retry %d)\n", result, retry);
        delay(100);
        result = esp_now_add_peer(&beaconPeer);
        retry++;
    }

    if (result == ESP_OK) {
        USBSerial.printf("Beacon peer added: %02X:%02X:%02X:%02X:%02X:%02X\n",
                       Beacon_mac[0], Beacon_mac[1], Beacon_mac[2],
                       Beacon_mac[3], Beacon_mac[4], Beacon_mac[5]);
    } else {
        USBSerial.printf("Failed to add beacon peer after retries: %d\n", result);
    }

    // チャンネルの設定はESP-NOWの設定の前に行う必要があるかもしれない
    //esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(500);
}

void drone_peer_init(void)
{
    memset(&dronePeer, 0, sizeof(dronePeer));
    memcpy(dronePeer.peer_addr, Drone_mac, 6);
    dronePeer.channel = CHANNEL;
    dronePeer.encrypt = false;
    dronePeer.ifidx = WIFI_IF_STA;
    while (esp_now_add_peer(&dronePeer) != ESP_OK) {
        USBSerial.println("Failed to add drone peer");
        delay(100);
    }
    USBSerial.println("Success to add drone peer");
    // チャンネルの設定はESP-NOWの設定の前に行う必要があるかもしれない
    //esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(500);
}

void peering(void)
{
  uint8_t break_flag;
  uint32_t beep_delay = 0;

  if (M5.Btn.isPressed() || (Drone_mac[0] == 0xFF && Drone_mac[1] == 0xFF && Drone_mac[2] == 0xFF && Drone_mac[3] == 0xFF &&
                               Drone_mac[4] == 0xFF && Drone_mac[5] == 0xFF)) {
    M5.Lcd.println("Push LCD panel!");
    while (1) {
      M5.update();
      if (M5.Btn.wasPressed()) {
        is_peering = 1;
        break;
      }
    }
    USBSerial.printf("Button pressed!\n\r");
    M5.Lcd.println(" ");
    M5.Lcd.println("Push StampFly");
    M5.Lcd.println("    Reset Button!");
    M5.Lcd.println(" ");
    M5.Lcd.println("Pairing...");

    //StampFlyはMACアドレスをFF:FF:FF:FF:FF:FFとして
    //StampFlyのMACアドレスをでブロードキャストする
    //その際にChannelが機体と送信機で同一でない場合は受け取れない
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
    USBSerial.printf("StampFly MAC:%02X:%02X:%02X:%02X:%02X:%02X:\n\r",
                      Drone_mac[0],Drone_mac[1],Drone_mac[2],Drone_mac[3],Drone_mac[4],Drone_mac[5]);
  }
}

void change_channel(uint8_t ch)
{
  #if 0
  dronePeer.channel = ch;
  if (esp_now_mod_peer(&dronePeer)!=ESP_OK)
  {
        USBSerial.println("Failed to modify peer");
        return;
  }
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  #endif
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
  //3秒待つ
  delay(3000);
  wifi_esp_now_init();

  // Broadcast peer needed for both master (beacon TX) and slave (pairing)
  broadcast_beacon_init();
  peering();
  drone_peer_init();
  
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

  // Create beacon transmission task BEFORE starting timer (master only)
  // This ensures beacon_task_handle is set when timer callback fires
  if (TDMA_DEVICE_ID == 0) {
    BaseType_t task_result = xTaskCreatePinnedToCore(
        beacon_task,           // Task function
        "BeaconTask",          // Task name
        4096,                  // Stack size (bytes)
        NULL,                  // Task parameter
        configMAX_PRIORITIES - 1,  // High priority for precise timing
        &beacon_task_handle,   // Task handle
        1                      // Core 1 (Arduino loop runs on core 1)
    );

    if (task_result != pdPASS) {
      USBSerial.println("Failed to create beacon task");
      beacon_task_handle = NULL;  // Ensure it's NULL on failure
    } else {
      USBSerial.printf("Beacon task created (handle=%p)\n", beacon_task_handle);
    }
  }

  // Start TDMA timer AFTER creating beacon task (both master and slave use it for synchronization)
  err = esp_timer_start_periodic(beacon_timer, TDMA_FRAME_US);
  if (err != ESP_OK) {
    USBSerial.printf("Failed to start TDMA timer: %d\n", err);
  } else {
    if (TDMA_DEVICE_ID == 0) {
      USBSerial.printf("TDMA Master timer started (ID=%d, period=%d us)\n", TDMA_DEVICE_ID, TDMA_FRAME_US);
    } else {
      USBSerial.printf("TDMA Slave timer started (ID=%d, period=%d us)\n", TDMA_DEVICE_ID, TDMA_FRAME_US);
    }
  }

  USBSerial.printf("TDMA initialized. Device ID=%d, Channel=%d\n", TDMA_DEVICE_ID, CHANNEL);
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
  senddata[0] = dronePeer.peer_addr[3];////////////////////////////
  senddata[1] = dronePeer.peer_addr[4];////////////////////////////
  senddata[2] = dronePeer.peer_addr[5];////////////////////////////

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

  // Check for beacon loss (slave only)
  static uint32_t last_beep_time = 0;
  if (TDMA_DEVICE_ID != 0 && first_beacon_received) {
    int64_t time_since_beacon = esp_timer_get_time() - last_beacon_time_us;
    if (time_since_beacon > BEACON_TIMEOUT_US) {
      // Beacon lost - beep every 500ms
      uint32_t current_millis = millis();
      if (current_millis - last_beep_time >= 500) {
        beep();
        last_beep_time = current_millis;
      }
    }
  }

  // TDMA synchronized transmission
  // Wait for beacon timer semaphore (short timeout to not block drone control)
  // Note: Beacon is sent by dedicated FreeRTOS task (master only), not here
  if (xSemaphoreTake(beacon_sem, pdMS_TO_TICKS(2)) == pdTRUE) {
    // Semaphore received - it's time to send control data

    // Calculate slot start time
    int64_t slot_start_us = epoch_next_us - TDMA_BEACON_ADVANCE_US + (TDMA_DEVICE_ID * TDMA_SLOT_US);
    int64_t current_us = esp_timer_get_time();

    // Precise microsecond timing for TDMA slot synchronization
    if (slot_start_us > current_us) {
      int64_t wait_us = slot_start_us - current_us;

      // If wait time > 20us, use delayMicroseconds for precise timing
      if (wait_us > 20) {
        delayMicroseconds(wait_us - 10);  // Wait until 10us before slot start
      }

      // Final precise timing with busy wait
      while (esp_timer_get_time() < slot_start_us) {
        // Busy wait for ultimate precision
      }
    }

    // Check if drone is available before sending (prevent queue overflow)
    if (!drone_available) {
      // Skip sending to offline drone to prevent queue overflow
      // Beacon transmission will not be affected
      return;  // Early return - don't send to offline drone
    }

    // Check if drone peer exists before sending
    static uint32_t peer_check_fail_count = 0;
    if (!esp_now_is_peer_exist(dronePeer.peer_addr)) {
      peer_check_fail_count++;
      if (peer_check_fail_count % 100 == 1) {  // Log first occurrence and every 100th
        USBSerial.printf("Drone peer not found! Re-registering... (count=%u)\n", peer_check_fail_count);
      }
      drone_peer_init();  // Re-register drone peer
    }

    // Send control data in our assigned slot
    esp_err_t result = esp_now_send(dronePeer.peer_addr, senddata, sizeof(senddata));
  } else {
    // Timeout - send anyway (fallback for non-TDMA mode)

    // Check if drone is available before sending (prevent queue overflow)
    if (!drone_available) {
      // Skip sending to offline drone to prevent queue overflow
      return;  // Early return - don't send to offline drone
    }

    // Check if drone peer exists before sending
    static uint32_t peer_check_fail_count_fallback = 0;
    if (!esp_now_is_peer_exist(dronePeer.peer_addr)) {
      peer_check_fail_count_fallback++;
      if (peer_check_fail_count_fallback % 100 == 1) {
        USBSerial.printf("Drone peer not found (fallback)! Re-registering... (count=%u)\n", peer_check_fail_count_fallback);
      }
      drone_peer_init();
    }

    esp_err_t result = esp_now_send(dronePeer.peer_addr, senddata, sizeof(senddata));
  }
  #ifdef DEBUG
  USBSerial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
    dronePeer.peer_addr[0],
    dronePeer.peer_addr[1],
    dronePeer.peer_addr[2],
    dronePeer.peer_addr[3],
    dronePeer.peer_addr[4],
    dronePeer.peer_addr[5]);
  #endif
  //Display information
  //float vbat =0.0;// M5.Axp.GetBatVoltage();
  //int8_t bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  
  M5.Lcd.setCursor(4, 2+disp_counter*17);
  switch (disp_counter)
  {
    case 0:
      M5.Lcd.printf("MAC ADR %02X:%02X    ", dronePeer.peer_addr[4],dronePeer.peer_addr[5]);
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
      M5.Lcd.printf("CHL: %02d",dronePeer.channel);
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
      #if TDMA_DEVICE_ID == 0
        // Master device - show frequency and role
        M5.Lcd.printf("Freq:%4d M[%3d]", 1000000/dtime, loop_counter);
      #else
        // Slave device - show frequency and PLL sync error or beacon loss
        if (first_beacon_received) {
          int64_t time_since_beacon = esp_timer_get_time() - last_beacon_time_us;
          if (time_since_beacon > BEACON_TIMEOUT_US) {
            M5.Lcd.printf("Freq:%4d LOST!  ", 1000000/dtime);
          } else {
            M5.Lcd.printf("Freq:%4d E:%+4d", 1000000/dtime, (int)pll_error_us);
          }
        } else {
          M5.Lcd.printf("Freq:%4d WAIT   ", 1000000/dtime);
        }
      #endif
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
