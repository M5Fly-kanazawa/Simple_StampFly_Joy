# StampFlyの操作方法

## 安全に飛ばすために
### 電池の管理
- 専用充電器を用いて充電してください
- 膨らんだ場合は、塩水等につけて１週間ほどおき完全に放電してから、お住まいの自治体に処理方法を問い合わせてください
- StampFlyに電池を差し間違えると回路が焼損し、電池も発火の危険がありますので慎重に差し込んでください
### 安全メガネの着用
- プロペラを覗き込むような場合や、プロペラが緩くすぐ外れるような場合はプロペラが飛んで目に入り失明する恐れがありますから、安全メガネを着用してください。

## 使い方
### 電池の挿入
- ATOMJoyとStampFlyに電池を挿入してください。
- オスとメスの端子に合うように差し込んでください。
- StampFlyに電池を取り付ける際に電池アダプタと機体の端子をずらして刺してしまうと、電池のアダプタから煙が出て焦げ付くか、電池が発火する可能がありますので注意してください。
### Peering (Pairing)
初めて飛ばす時は、ピアリング（ペアリング）をしてください。
- ATOMJoyの液晶パネルを押しながらスイッチを入れる
- もう一度、液晶パネルを押す
- StampFlyに電池を接続するか、リセットする。
- AtomJoyの液晶パネルの表示が変わったらピアリング完了
### 飛行可能になるまで
- 電池を接続するとLEDが白く点灯します
- LEDが白く点灯しているうちに水平な面においてください
- しばらくするとLEDが紫色になります。紫色の間にセンサのオフセットを取得しているの触らないでください
- 最後にLEDがイルミネーション表示に変わり飛行可能となります
### 手動での飛ばし方(スタビライズモード)
- デフォルトではモード２です。以下はモード２での説明です。モード３では左右反対です
- デフォルトでスタビライズモードです。姿勢を自動的に保ちます
- スロットルのスティックを押し込むとStampFlyのLEDが黄色になりArming状態になります
- スロットルのスティックを上に倒すとプロペラの回転数が上がり、上昇します
- スロットルのスティックを下げるとプロペラの回転数が下がり、下降します
- スロットルのスティックの中立でプロペラが止まります。
- スロットルのスティックの中立より下は意味はありません
- スロットルのスティックを左右に倒すと倒した方に機首が方向を変えます
- スロットルのスティックの前後左右で機体がその方向に移動します
- スロットルのスティックの倒した量に応じて機体が傾きます
- 再度、スロットルのスティックを押し込むとDisArmingとなりモータが停止しLEDがイルミネーション状態に戻ります
- 飛行中に、スロットルのスティックを押し込むと自動着陸します。その最中にもう一度押し込むとモータが止まり落下しますので注意してください
### 電池切れサイン
- LEDの色が水色になったら電池切れサインです。もうしばらくは飛べますが、続けるとリセットがかかり落下します。電池を充電しましょう
### 宙返り
- 空中に静止したホバリング状態を維持して、ロールピッチのスティックを押し込むと自動的に宙返りをします
### アクロモード
- ATOMJoyの前方にある右ボタンを一度押すとアクロモードに切り替わります。もう一度押すとスタビライずモードに戻ります。ATOMJoyの液晶画面でモードは確認できます。
- アクロモードは姿勢を自動的に維持しません。自分で姿勢を保つように操縦します
- スロットルスティックの倒し量に応じた回転速度で機体が回転します。中立に戻すと回転速度０です。回転を止めても傾きはそのままなので自分で戻す必要があります
### 高度維持モード（開発中）
- StampFlyは全て常に開発中ですが、高度制御は未完成です
- 着陸した状態で、ATOMJoyの前方の左のボタンを１度押すと高度維持モードになります。
- 高度維持モードは、スロットルレバーを押してArmingすると自動で離陸し高度50cmまで上がります。LEDがピンク色が高度維持モードを示します
- スロットルステイックの上下で高さを徐々に変えることができます
- 高度維持モードは高さの変化は緩やかで素早く高さを変化させることはできません
- 床面が段差となり、機体と床面との高さが変化すると、それに応じて高度を調整しようとします
- あまり急激な高度変化が伴うと、機体が勢いよく運動し不安定になるか、リセットがかかって墜落する事があります。
- 着陸はスロットルレバーを押し込むと自動着陸します。
- 自動着陸中に、再度スロットルレバーを押し込むと即モータが止まります。
### モード３
- 開発者のお気に入りモードです
- ATMJoyの前方の左のボタンを押しながらATOMJoyのスイッチを入れるとモード３になります。スイッチを切るまで変えられません
- モード３はモード２のスティック操作が左右逆です。ただし、前方ボタンの役割は入れ替わりません
### 衝撃停止機能
- 大きな衝撃が加わると自動的にモータが停止します

### テレメトリ機能
Atom JoyStickのATOMS3のUSB-CポートにPCからケーブルを接続すると、Joyのスイッチを入れなくても給電され動作します。その際にPC側でお好きなシリアル通信端末（例えばPlatformIOのシリアルモニタ）を開くと機体の各種情報がリアルタイムに流れます。これを保存することでログが取れます。

### TDMA多機同時運用機能
このコントローラーは、TDMA（時分割多重アクセス）方式により、最大10台のコントローラー/機体を同時に運用できます。

#### システム構成
- **デバイスID**: 0〜9の10台をサポート（ID=0が親機、ID=1〜9が子機）
- **WiFiチャンネル**: チャンネル1固定（main.cpp内の`CHANNEL`定義で変更可能）
- **制御周波数**: 50Hz（20ms周期）
- **タイムスロット**: 各デバイスに2msのスロットを割り当て

#### デバイスID設定方法
各コントローラーに異なるIDを割り当てる必要があります：
1. `src/main.cpp`の`TDMA_DEVICE_ID`定義を編集（56行目付近）
2. 親機（マスター）は必ず`#define TDMA_DEVICE_ID 0`に設定
3. 子機（スレーブ）は`#define TDMA_DEVICE_ID 1`〜`9`に設定
4. ビルド＆アップロードして各コントローラーに書き込む

#### LCD画面表示
液晶画面の下部に以下の情報が表示されます：
- **親機（ID=0）**: `Freq:50 M[counter]` - M=マスター、実測周波数と送信カウンタ
- **子機（ID=1〜9）**: `F:50 E:±error` - 実測周波数と同期誤差（マイクロ秒）
- **ビーコン喪失時**: `F:50 LOST!` - 親機からのビーコンが受信できない状態

#### ビープ音による状態識別
コントローラーは異なる周波数とパターンのビープ音で状態を通知します：

1. **ビーコン喪失** - 4000Hz シングルビープ（500ms毎）
   - 意味: 親機からのビーコンが受信できない（緊急度：高）
   - 対処: 親機の電源を確認、または距離を近づける

2. **スロットタイミングエラー** - 3000Hz ダブルビープ（100回に1回）
   - 意味: 割り当てられた時間スロットに間に合わなかった
   - 対処: 通常は自動回復。頻発する場合はログを確認

3. **ドローンオフライン** - 1000Hz ロングビープ（100回に1回）
   - 意味: StampFlyとのESP-NOW接続が失われた
   - 対処: StampFlyの電源を確認、ペアリングをやり直す

4. **Mutexタイムアウト** - 2000Hz トリプルビープ（100回に1回）
   - 意味: 内部タスク間のデータ競合が発生（まれ）
   - 対処: 通常は自動回復。頻発する場合は再起動

#### 5台以上での運用時の注意
- タイムスロットは2ms、制御周波数は50Hzに設定済み（5台以上の安定運用向け）
- WiFi混雑環境では干渉が発生する可能性があります
- 親機と子機の距離は5m以内を推奨
- 全ての機体を同時に飛行させる場合、十分な空間を確保してください


# StampFly Operation Manual

## For Safe Operation
### Battery Management
- Please use a dedicated charger for charging.
- If the battery swells, submerge it in salt water for about a week to completely discharge it, and then consult your local municipality for disposal instructions.
- Be cautious when inserting the battery into StampFly, as incorrect insertion may cause circuit damage or a battery fire.

### Wearing Safety Glasses
- When looking into the propeller or if the propeller is loose and prone to detachment, please wear safety glasses to prevent potential eye injuries, which could result in blindness.

## How to Use
### Inserting the Battery
- Insert the battery into both ATOMJoy and StampFly.
- Ensure the connectors match the male and female terminals before inserting.
- When attaching the battery to StampFly, be careful not to misalign the battery adapter with the terminal, as this may cause the adapter to emit smoke, scorch, or possibly ignite.

### Peering (Pairing)
When flying for the first time, perform pairing.
- Press the LCD panel on ATOMJoy while turning on the switch.
- Press the LCD panel again.
- Connect the battery to StampFly or reset it.
- Pairing is complete when the display on the LCD panel of ATOMJoy changes.

### Before Flight
- The LED will light up white when the battery is connected.
- Place the device on a flat surface while the LED is white.
- After a while, the LED will turn purple. Do not touch the device during this time as it is calibrating the sensor offset.
- Finally, the LED will switch to an illumination display, indicating that the device is ready to fly.

### Manual Flight (Stabilize Mode)
- The default setting is Mode 2. The following explanation is for Mode 2. In Mode 3, left and right controls are reversed.
- The default setting is Stabilize Mode, which automatically maintains the posture.
- Press the throttle stick to engage Arming mode, where the StampFly's LED turns yellow.
- Pushing the throttle stick up increases the propeller speed, causing the device to ascend.
- Pulling the throttle stick down decreases the propeller speed, causing the device to descend.
- The propeller stops when the throttle stick is in neutral.
- There is no function when the throttle stick is below neutral.
- Moving the throttle stick left or right changes the direction of the nose.
- Moving the throttle stick forward, backward, left, or right moves the device in that direction.
- The tilt of the device corresponds to the amount the throttle stick is pushed.
- Pressing the throttle stick again will disengage (DisArming), stopping the motors and returning the LED to the illumination state.
- During flight, pressing the throttle stick will initiate an automatic landing. If pressed again during this process, the motors will stop, and the device will drop, so proceed with caution.

### Low Battery Indicator
- When the LED turns cyan, it indicates low battery. You can still fly for a while, but continuing may cause a reset and result in a crash. It's time to recharge the battery.

### Flip Maneuver
- When hovering in a stationary position, pressing the roll/pitch stick will automatically perform a flip.

### Acro Mode
- Pressing the right button on the front of ATOMJoy switches to Acro Mode. Press it again to return to Stabilize Mode. You can check the mode on ATOMJoy's LCD screen.
- Acro Mode does not automatically maintain posture. You must control it manually.
- The device rotates at a speed corresponding to how much you push the throttle stick. Returning the stick to neutral stops the rotation, but the tilt remains, so you need to correct it manually.

### Altitude Hold Mode (In Development)
- StampFly is always in development, and altitude control is not yet complete.
- When the device is on the ground, pressing the left button on the front of ATOMJoy once will activate Altitude Hold Mode.
- In Altitude Hold Mode, pressing the throttle lever to Arm will automatically take off and rise to an altitude of 50 cm. The LED turns pink to indicate Altitude Hold Mode.
- You can gradually change the height by moving the throttle stick up or down.
- Altitude Hold Mode changes height slowly and cannot make quick altitude adjustments.
- If the ground surface has steps or the distance between the device and the ground changes, the altitude will be adjusted accordingly.
- Sudden altitude changes may cause the device to move rapidly and become unstable, possibly triggering a reset and causing a crash.
- Pressing the throttle lever will initiate an automatic landing.
- During automatic landing, pressing the throttle lever again will immediately stop the motors.

### Mode 3
- This is the developer's favorite mode.
- Holding the left button on the front of ATOMJoy while turning on the switch will activate Mode 3. It cannot be changed until the switch is turned off.
- Mode 3 reverses the left and right stick operations of Mode 2. However, the function of the front button does not change.

### Impact Stop Function
- If a significant impact occurs, the motors will automatically stop.

### Telemetry Function
When you connect a cable from your PC to the USB-C port of the ATOMS3 on the Atom JoyStick, it will be powered and operate without needing to turn on the switch. During this time, if you open your preferred serial communication terminal on the PC (e.g., PlatformIO's Serial Monitor), various real-time information about the device will be displayed. You can save this data to log it.

### TDMA Multi-Unit Simultaneous Operation
This controller supports up to 10 simultaneous controllers/drones using TDMA (Time Division Multiple Access) technology.

#### System Configuration
- **Device ID**: Supports 10 units (ID=0 is master, ID=1-9 are slaves)
- **WiFi Channel**: Fixed to channel 1 (configurable via `CHANNEL` definition in main.cpp)
- **Control Frequency**: 50Hz (20ms period)
- **Time Slots**: 2ms slot allocated to each device

#### Device ID Configuration
Each controller must be assigned a unique ID:
1. Edit the `TDMA_DEVICE_ID` definition in `src/main.cpp` (around line 56)
2. Master device must be set to `#define TDMA_DEVICE_ID 0`
3. Slave devices should be set to `#define TDMA_DEVICE_ID 1` through `9`
4. Build and upload to each controller

#### LCD Display Information
The bottom of the LCD screen displays the following:
- **Master (ID=0)**: `Freq:50 M[counter]` - M=Master, measured frequency and transmission counter
- **Slave (ID=1-9)**: `F:50 E:±error` - Measured frequency and synchronization error (microseconds)
- **Beacon Lost**: `F:50 LOST!` - Unable to receive beacon from master

#### Error Identification via Beep Sounds
The controller uses different beep frequencies and patterns to indicate various states:

1. **Beacon Loss** - 4000Hz single beep (every 500ms)
   - Meaning: Unable to receive beacon from master (high urgency)
   - Solution: Check master power or move closer

2. **Slot Timing Error** - 3000Hz double beep (every 100 occurrences)
   - Meaning: Missed assigned time slot
   - Solution: Usually auto-recovers. Check logs if frequent

3. **Drone Offline** - 1000Hz long beep (every 100 occurrences)
   - Meaning: Lost ESP-NOW connection with StampFly
   - Solution: Check StampFly power or re-pair

4. **Mutex Timeout** - 2000Hz triple beep (every 100 occurrences)
   - Meaning: Internal task data contention (rare)
   - Solution: Usually auto-recovers. Restart if frequent

#### Precautions for 5+ Unit Operation
- Time slots are set to 2ms and control frequency to 50Hz (optimized for 5+ stable operation)
- Interference may occur in congested WiFi environments
- Recommended distance between master and slaves: within 5m
- Ensure adequate space when flying multiple drones simultaneously

