# TDMA（時分割多元接続方式）使用ガイド

## 概要

このプログラムは、複数のStampFlyコントローラが同一チャンネルで干渉なく通信できるよう、TDMA（Time Division Multiple Access）方式を実装しています。

## TDMA方式の仕組み

### タイミング構造

```
フレーム（10ms）
├─ ビーコン（親機のみ送信、フレーム開始250μs前）
├─ スロット0（1ms）親機の制御データ送信
├─ スロット1（1ms）子機ID=1の制御データ送信
├─ スロット2（1ms）子機ID=2の制御データ送信
├─ ...
└─ スロット9（1ms）子機ID=9の制御データ送信
```

### 動作原理

1. **親機（ID=0）**: 自律的にビーコンを10ms周期で送信し、スロット0で制御データを送信
2. **子機（ID=1-9）**: 親機のビーコンを受信してPLLで同期し、割り当てられたスロットで送信
3. **PLL同期**: 位相誤差を検出して自動補正することで、長時間の同期を維持

---

## 設定方法

### 1. デバイスIDの設定

コントローラのデバイスIDを設定します。**親機は必ず1台のみ、ID=0に設定してください。**

#### 設定箇所

[src/main.cpp:44](src/main.cpp#L44)

```cpp
#define TDMA_DEVICE_ID 0         // Device ID: 0=Master, 1-9=Slave (manual setting)
```

#### 設定値

| デバイスID | 役割 | スロット | 備考 |
|-----------|------|---------|------|
| 0 | 親機（Master） | スロット0 | **必ず1台のみ** |
| 1 | 子機1（Slave） | スロット1 | |
| 2 | 子機2（Slave） | スロット2 | |
| ... | ... | ... | |
| 9 | 子機9（Slave） | スロット9 | |

#### 設定例

```cpp
// 親機の設定
#define TDMA_DEVICE_ID 0

// 子機1の設定
#define TDMA_DEVICE_ID 1

// 子機2の設定
#define TDMA_DEVICE_ID 2
```

### 2. WiFiチャンネルの設定

すべてのデバイスで**同一チャンネル**を使用する必要があります。

#### 設定箇所

[src/main.cpp:41](src/main.cpp#L41)

```cpp
#define CHANNEL 1
```

#### 設定値

- **範囲**: 1～14（日本国内の場合は1～13推奨）
- **注意**: **親機と子機で必ず同じ値にする**

#### 設定例

```cpp
// すべてのデバイスで同じチャンネルを設定
#define CHANNEL 6
```

### 3. TDMA詳細パラメータ（通常は変更不要）

[src/main.cpp:45-48](src/main.cpp#L45-L48)

```cpp
#define TDMA_FRAME_US 10000          // 1フレーム = 10ms
#define TDMA_SLOT_US 1000            // 1スロット = 1ms
#define TDMA_NUM_SLOTS 10            // スロット数 = 10
#define TDMA_BEACON_ADVANCE_US 250   // ビーコン先行時間 = 250μs
```

**通常はこれらの値を変更する必要はありません。**

### 4. PLL同期パラメータ（上級者向け）

[src/main.cpp:97-98](src/main.cpp#L97-L98)

```cpp
static const float PLL_KP = 0.1;     // 比例ゲイン
static const float PLL_KI = 0.01;    // 積分ゲイン
```

同期性能に問題がある場合のみ調整してください。

---

## ビルドと書き込み手順

### 1. 設定確認

各コントローラのプログラムで以下を確認：

- ✅ デバイスIDが重複していないか（親機は1台のみID=0）
- ✅ すべてのデバイスで同じチャンネルを使用しているか

### 2. ビルド

PlatformIOでビルドします：

```bash
pio run
```

### 3. 書き込み

```bash
pio run --target upload
```

### 4. 動作確認

シリアルモニタで起動メッセージを確認：

```bash
pio device monitor
```

#### 親機の起動メッセージ例

```
ESP-NOW Version 123
TDMA Master started (ID=0)
```

#### 子機の起動メッセージ例

```
ESP-NOW Version 123
TDMA Slave initialized (ID=1)
```

---

## 使用例：3台のコントローラを使用する場合

### コントローラ1（親機）の設定

```cpp
#define CHANNEL 6
#define TDMA_DEVICE_ID 0
```

### コントローラ2（子機1）の設定

```cpp
#define CHANNEL 6
#define TDMA_DEVICE_ID 1
```

### コントローラ3（子機2）の設定

```cpp
#define CHANNEL 6
#define TDMA_DEVICE_ID 2
```

### タイミングチャート

```
時刻 0ms: 親機がビーコン送信
         ↓
時刻 0.25ms: フレーム開始
時刻 0-1ms: 親機（ID=0）が制御データ送信
時刻 1-2ms: 子機1（ID=1）が制御データ送信
時刻 2-3ms: 子機2（ID=2）が制御データ送信
時刻 3-10ms: 未使用スロット
         ↓
時刻 10ms: 次のフレーム開始（親機がビーコン送信）
```

---

## トラブルシューティング

### Q1: 子機が同期しない

**確認事項:**
- 親機が正常に起動しているか（起動メッセージを確認）
- チャンネルが親機と子機で一致しているか
- ペアリングが完了しているか

**対処法:**
1. すべてのデバイスを再起動
2. シリアルモニタでエラーメッセージを確認
3. PLL同期が動作しているかログで確認

### Q2: 複数の親機を使いたい

**仕様制限:**
現在の実装では、**1つのネットワークに親機は1台のみ**です。複数の親機を使用する場合は、異なるチャンネルを使用してネットワークを分離してください。

**例:**
- ネットワーク1: チャンネル6、親機（ID=0）+ 子機（ID=1,2）
- ネットワーク2: チャンネル11、親機（ID=0）+ 子機（ID=1,2）

### Q3: デバイスIDを動的に変更したい

現在のバージョンでは、デバイスIDはコンパイル時の定数です。将来のバージョンで、実行時設定やSPIFFS保存機能を追加予定です。

### Q4: 10台以上のコントローラを使いたい

スロット数を増やす必要があります：

```cpp
#define TDMA_FRAME_US 20000      // 20msに延長
#define TDMA_NUM_SLOTS 20        // スロット数を20に増加
```

ただし、フレーム時間が長くなると制御の遅延が増加します。

### Q5: タイムアウトエラーが発生する

```cpp
// タイムアウト時間を延長（デフォルト20ms）
if (xSemaphoreTake(beacon_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
```

[src/main.cpp:638](src/main.cpp#L638) の値を調整してください。

---

## 動作確認とデバッグ

### デバッグモードの有効化

[src/main.cpp:27](src/main.cpp#L27)

```cpp
#define DEBUG  // この行のコメントを外す
```

デバッグモードでは、送信先MACアドレスがシリアル出力されます。

### PLL同期状態の確認

子機のシリアルモニタでPLL誤差を確認したい場合は、以下のコードを追加：

```cpp
// OnDataRecv関数内、PLL更新後に追加
USBSerial.printf("PLL error: %d us, integral: %d\n", pll_error_us, pll_integral);
```

正常に同期している場合、誤差は数十μs以内に収束します。

---

## パケットフォーマット

### ビーコンパケット（親機のみ送信）

| バイト | 値 | 説明 |
|-------|-----|------|
| 0 | 0xBE | ビーコンヘッダ1 |
| 1 | 0xAC | ビーコンヘッダ2 |

**合計**: 2バイト

### 制御データパケット

| バイト | 内容 | 説明 |
|-------|------|------|
| 0-2 | MAC下位3バイト | 宛先識別用 |
| 3-4 | Throttle | スロットル（16bit整数） |
| 5-6 | Phi | ロール角（16bit整数） |
| 7-8 | Theta | ピッチ角（16bit整数） |
| 9-10 | Psi | ヨー角（16bit整数） |
| 11 | フラグ | Arm/Flip/Mode/AltMode（各1bit） |
| 12 | proactive_flag | プロアクティブフラグ |
| 13 | チェックサム | バイト0-12の合計 |

**合計**: 14バイト（senddata配列は25バイト確保）

### テレメトリパケット（ドローン→コントローラ）

| バイト | 内容 | 説明 |
|-------|------|------|
| 0-1 | ヘッダ | パケット識別用 |
| 2-5 | float値1 | 4バイトfloat |
| 6-9 | float値2 | 4バイトfloat |
| ... | ... | 4バイト単位で続く |

**注意**: テレメトリは `(data_len - 2) % 4 == 0` を満たす必要があります。

---

## 性能仕様

| 項目 | 値 |
|-----|-----|
| フレーム周期 | 10ms（100Hz） |
| スロット幅 | 1ms |
| 最大同時接続数 | 10台 |
| ビーコン送信タイミング精度 | ±数μs（esp_timer） |
| PLL同期精度 | ±数十μs（定常状態） |
| 送信遅延 | 最大10ms（自スロットまでの待ち時間） |

---

## 将来の拡張予定

- [ ] デバイスIDの実行時設定（SPIFFS保存）
- [ ] チャンネルの実行時設定
- [ ] 親機の自動選出（親機不在時の自動昇格）
- [ ] 動的なスロット割当（デバイスの参加/離脱対応）
- [ ] スロット使用状況のモニタリング
- [ ] ビーコン未受信時の自動再接続

---

## 参考情報

### 関連ファイル

- [src/main.cpp](src/main.cpp) - メインプログラム
- [platformio.ini](platformio.ini) - ビルド設定

### 主要関数

- `beacon_timer_callback()` ([main.cpp:106](src/main.cpp#L106)) - ビーコンタイマー割り込み
- `OnDataRecv()` ([main.cpp:129](src/main.cpp#L129)) - パケット受信コールバック
- `setup()` ([main.cpp:441-473](src/main.cpp#L441-L473)) - TDMA初期化
- `loop()` ([main.cpp:636-662](src/main.cpp#L636-L662)) - TDMA同期送信

### 技術資料

- ESP-NOW: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
- ESP32 Timer: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html

---

## ライセンス

MIT License - 詳細は [LICENSE](LICENSE) を参照

## 作者

Kouhei Ito - kouhei.ito@itolab-ktc.com

## 更新履歴

- 2025-11-07: TDMA初版実装
