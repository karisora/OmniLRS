# Lumos on OmniLRS Lunaryard — ROS 2 Jazzy / Gazebo Harmonic

既定の地形は **OmniLRS Lunaryard** です。リポジトリのルートから起動します。

```bash
bash scripts/run_lumos_gazebo.sh
# GUIなしでもROS 2のステレオ映像を配信
bash scripts/run_lumos_gazebo.sh gui:=false
# 生成シード・スポーン位置を変更（m、rad）
bash scripts/run_lumos_gazebo.sh seed:=43 x:=20 y:=20 yaw:=1.57
```

`cfg/environment/lunaryard_40m.yaml` と
`src.terrain_management.terrain_generation.GenerateProceduralMoonYard` を直接使用して、
OmniLRSと同じ起伏・クレータープロファイル・配置を生成します。
標準は40 m四方、解像度2 cm、シード42です。生成結果を静的なheightmapとしてGazeboへ渡し、
表示と車輪の接触の両方に同じ高さを使います。Isaac Simの起動は不要です。
標本間隔を変えずに2000 × 2000から2049 × 2049へ端の高さでパディングし、
元のLunaryard座標（+X/+Y、実データ範囲は0〜39.98 m）を維持します。
スポーンは標準 `(20, 20)`、高さは地面から自動算出します。
地面はOmniLRSの元設定 `texture_path: /Lunaryard/Looks/Basalt` に従います。
標準の `assets/Textures/GravelStones/` の色画像・法線マップを使い、
元MDLの `texture_scale=5` に合わせて0.2 m周期で繰り返します。
色画像は元ファイルを直接参照し、EXR法線マップはGazebo用にPNGへ形式変換します。
`texture_size` の標準値 `auto` は、選択した標準材質のUV倍率を使用します。

照明も元設定の `sun_settings` に従い、標準では太陽高度45°・方位180°・白色です。
OmniLRSの `create_sun()` と同じ回転を適用し、光の進行方向は概ね `(0, +0.707, -0.707)`、
つまり −Y側から斜め下向きに照らします。拡散・鏡面光の倍率も元設定の1.0を使用し、
以前追加した環境光による底上げは行いません。
`sun_elevation`、`sun_azimuth`、`sun_intensity` の標準値は `auto` です。
元のUsdLux強度1750はGazeboの相対強度1.0に対応させ、設定値の比率を維持します。
UsdLuxとGazeboの強度単位・描画方式は異なるため、画素輝度を校正した一致ではありません。
ここでは `sun_settings` の初期設定を固定で使用し、ネイティブの天体暦による時刻更新は行いません。
元設定の太陽角直径0.53°・色温度6500 Kも `spawn.json` に記録しますが、
Gazeboの方向光源には元のRTXと同じ面積光源の影・色温度処理はありません。

生成物は `build/gazebo_lunaryard/` の `lunaryard.sdf`、`lunaryard_dem.npy`、
`lunaryard_mask.npy`、`lunaryard_height.png`、`craters.json`、`spawn.json` です。
`spawn.json` に元設定のパス・チェックサム・シード・解像度・生成器を記録します。
別の正方形Lunaryard設定は `config:=/absolute/path/to/lunaryard_20m.yaml` などで指定できます。
40 m以外を使う場合は、マップ内の `x`、`y` も指定してください。
ネイティブ設定を使うため、このOmniLRSチェックアウトの `src/`、`cfg/`、
クレータープロファイルが必要です。`run_lumos_gazebo.sh` がルートを自動設定します。

ROS 2パッケージの既定エントリーポイントもLunaryardです。

```bash
source /opt/ros/jazzy/setup.bash
source install/gazebo_ros/setup.bash
ros2 launch lumos1_rockerless_URDF_description gazebo.launch.py
```

## Site20を明示的に使う場合

OmniLRS にある NASA Site20 DEM (`3200 × 3200`, `5 m/pixel`) を使い、
月重力 `1.62 m/s²` の地形に Lumos をスポーンします。Isaac Sim は不要です。
低い太陽角（標準 5°）の照明、レゴリスの色・ノーマルマップ、ランダムな人工クレーターを含みます。

リポジトリのルートから実行します（Jazzy / Harmonic がインストール済みであること）。

```bash
bash scripts/run_site20_gazebo.sh
```

ビルド、DEM 変換、Gazebo、robot_state_publisher、ROS/Gazebo bridge、スポーンを一括実行します。
標準では全 DEM を保持し、5 m の標本間隔を変えずに 4097 × 4097 に端の値でパディングします。
実測範囲は X = −8000…7995 m、Y = −7995…8000 m です。パディング部分は実測地形ではありません。
軽く起動する場合は中心の 1280 × 1280 m を使用できます。

```bash
bash scripts/run_site20_gazebo.sh samples:=257
# GUI なし（EGLでステレオカメラの描画・ROS配信も継続）
bash scripts/run_site20_gazebo.sh gui:=false samples:=257
# 別の位置へスポーン（m、rad）。高さは地形から自動算出します。
bash scripts/run_site20_gazebo.sh x:=10 y:=20 yaw:=1.57
```

`samples` は `0`（全域）、`257`、`513`、`1025`、`2049` など `2^n+1`。
切り出した場合も標本間隔は 5 m です。地図画像や標高を拡大縮小しません。
ワールド座標は DEM の中央の画素を原点、X を画像右方向、Y を画像上方向に取ります。
これは月の極域投影のグリッド座標で、地理的な真東・真北とは異なります。
中央画素の標高を Z の基準にして数 km の絶対標高を差し引きます。
16-bit PNG の標高量子化誤差は全域でも最大約 2.8 cm です。
衝突と表示に同一の heightmap を使い、ローバーの占有範囲の標高に余裕を加えてスポーンします。

照明と地面の設定:

```bash
# 太陽の高度・方位（度）を変更
bash scripts/run_site20_gazebo.sh sun_elevation:=5 sun_azimuth:=135
# 見やすい高めの太陽と、地面テクスチャの繰り返し寸法（m）
bash scripts/run_site20_gazebo.sh sun_elevation:=15 sun_intensity:=10 texture_size:=0.375
# 毎回違うクレーター配置
bash scripts/run_site20_gazebo.sh crater_count:=64 crater_seed:=-1
# 特定の配置を再現、または追加クレーターを無効化
bash scripts/run_site20_gazebo.sh crater_seed:=42
bash scripts/run_site20_gazebo.sh crater_count:=0
```

`sun_azimuth` は地図グリッド +X から +Y 向きに測った太陽の方位です。
黒い背景と弱い環境光を使い、低い太陽からの直接光と長い影を表現します。
南極の低い日照角を参考にした固定照明で、日時による天体暦や照度の測定値は再現していません。
`sun_intensity` は Gazebo の照明・撮影時の明るさに相当する表示用倍率（標準 `10`）です。
[NASA の月の南極の照明資料](https://science.nasa.gov/resource/shadows-near-the-moons-south-pole/)を参考にしています。

地面には [NASA POLAR Terrain13](https://ti.arc.nasa.gov/dataset/IRG_PolarDB/) の
明るい月高地模擬材 NU-LHT の撮影画像から作った、無彩色の 2K テクスチャを使います。
岩・設備・縁の影を避けた部分を抽出し、大きな照明むらと繰り返しの境界を抑えています。
月高地の参照アルベド `0.30` を用い、微細な法線の強さを抑えて、低い太陽でも黒い斑点が
過剰に生じない淡い灰色の粉体表面にしています。タイル寸法は標準約 `0.375 m` です。
原画像は地上実験の月面模擬材で、Site20 の実際の色分布を測った画像ではありません。
PGDA の Site20 データは標高データとして使用します。
粒状の凹凸は表示用で、車輪の接触地形は引き続き 5 m の DEM です。
出典・加工条件・再生成は [材質の説明](materials/README.md) と
`materials/textures/polar_lht.json` を参照してください。原画像も同梱しており、起動時の通信は不要です。
Gazebo Harmonic の Ogre2 には、heightmap に roughness map がない場合に粗さを 0 にする
描画処理があります。このマップでは `build/gazebo_site20/rendering/` に描画リソースをコピーし、
該当する既定値だけを `0.95` にして乾いた地面の強い鏡面反射を抑えます。
この Gazebo プロセスだけに適用し、インストール済みの Gazebo ファイルは変更しません。

標準で 64 個の人工クレーター（直径 40〜160 m、くぼみと盛り上がった縁）を追加します。
約 1/3 はスポーン地点から 240 m 以内、残りは地図全体に配置します。
クレーターは DEM のコピーを加工するため、表示と接触形状の両方に反映されます。
元の `dem.npy` は変更せず、開始地点から半径 12 m は掘削・盛り土から保護します。
これらは実測の Site20 に追加した合成地形であり、実在クレーターの位置ではありません。
標準シードは `20`。`-1` で新しい乱数シードを選び、実際の値を `spawn.json` に記録します。
配置と寸法は `build/gazebo_site20/craters.json` で確認できます。

生成物は `build/gazebo_site20/`（`site20.sdf`, `lumos.urdf`, `spawn.json` など）です。
既存の `dem.npy` と隣の `dem.yaml` が必要です。存在しない場合には明示的なエラーを出します。
別の場所にあるデータは `dem:=/absolute/path/to/dem.npy` で指定できます。

別ターミナルから、走行と再スポーンを行えます。

```bash
source /opt/ros/jazzy/setup.bash
source install/gazebo_ros/setup.bash
# 車体前方へ 0.1 m/s。Ctrl+C 後は必ず停止コマンドを送ります。
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
# 左旋回（rad/s）。linear.x と angular.z は同時指定もできます。
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.3}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
# Lunaryardのモデルを削除して、保存された初期位置・姿勢へ再作成
ros2 run lumos1_rockerless_URDF_description respawn_rover.py --output build/gazebo_lunaryard
# Site20で起動した場合は --output build/gazebo_site20 を使用
```

ROS トピック: `/cmd_vel`, `/odom`, `/joint_states`, `/tf`, `/tf_static`, `/clock`。
CAD メッシュは元の mm→m スケールを維持し、`base_footprint` で ROS の +X 前方へ変換しています。
左右の車輪軸を統一し、4 輪の DiffDrive と円柱の車輪衝突形状を使用します。
これは剛体地形の接触モデルです。OmniLRS の土壌変形は移植していません。
旧 ROS 1 の `.launch` ファイルは ROS 2 では使用しません。
ROS 2の既定は `lunaryard.launch.py`（別名 `gazebo.launch.py`）、Site20は `site20.launch.py` です。

車体前方には基線長 12 cm の平行ステレオカメラを標準装備しています。
各カメラは RGB 640 × 480、シミュレーション時刻で 15 fps、水平画角 90°です。
レンズ中心は `base_footprint` に対して前方 12.3 cm、高さ約 29 cm、左右 ±6 cm にあります。
光学フレームは `stereo_left_optical_frame` と `stereo_right_optical_frame` で、
ROS の +Z 前方・+X 右・+Y 下に合わせた固定 TF を配信します。

| ROS 2 トピック | メッセージ型 |
| --- | --- |
| `/stereo/left/image_raw` | `sensor_msgs/msg/Image` |
| `/stereo/right/image_raw` | `sensor_msgs/msg/Image` |
| `/stereo/left/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/stereo/right/camera_info` | `sensor_msgs/msg/CameraInfo` |

すべて Gazebo → ROS 2 の片方向配信で、QoS は Sensor Data（Best Effort / Volatile）です。
画像・CameraInfo はシミュレーション時刻を使います。左右の内部パラメーターは共通で、
右 CameraInfo の `P[3] = -fx × 0.12 = -38.4` に基線長を含みます。
歪みなしの平行ピンホールモデルです。視差画像や深度画像の計算は含めていません。

映像の表示と受信確認（起動後、別ターミナル）:

```bash
source /opt/ros/jazzy/setup.bash
# 左右を別ウィンドウで表示。それぞれ別ターミナルから起動できます。
ros2 run rqt_image_view rqt_image_view /stereo/left/image_raw
ros2 run rqt_image_view rqt_image_view /stereo/right/image_raw
# フレームレート、キャリブレーションの確認
ros2 topic hz /stereo/left/image_raw
ros2 topic echo /stereo/right/camera_info --once --qos-reliability best_effort
```

RViz2 の Image 表示を使う場合は対象の画像トピックを選び、Reliability を
`Best Effort` にします。TF は `odom` → `base_footprint` → 各カメラに接続されます。
`gui:=false` でも描画用の OpenGL / EGL 環境が必要です。
EGL を使わず既存の X ディスプレイで描画する場合は `gui:=false headless_rendering:=false` を指定します。

リポジトリのルートから自動確認も実行できます。

```bash
# 左右の同時刻画像・CameraInfo・光学TF・速度指令の購読先を確認
python3 scripts/check_lumos_ros2.py
# さらに短時間の前進・左旋回・停止を実行し、オドメトリで応答を確認
python3 scripts/check_lumos_ros2.py --drive
```

確認結果は `build/lumos_ros2_check/ros2_check.json`、左右画像は
`build/lumos_ros2_check/stereo.png` に保存します。`--drive` は実際にローバーを動かすため、
他の速度指令ノードを停止して実行してください。

依存パッケージを追加する必要がある環境では:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-rqt-image-view ros-jazzy-tf2-ros ros-jazzy-rclpy \
  python3-colcon-common-extensions python3-numpy python3-pil python3-yaml \
  python3-scipy python3-opencv python3-matplotlib
```

参照: [Gazebo の ROS 2 移行手順](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/)、
[モデルのスポーン](https://gazebosim.org/docs/harmonic/ros2_spawn_model/)、
[物理エンジンと衝突検出器](https://gazebosim.org/api/sim/8/physics.html)。
