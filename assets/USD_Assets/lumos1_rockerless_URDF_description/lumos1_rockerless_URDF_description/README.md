# Lumos on Site20 — ROS 2 Jazzy / Gazebo Harmonic

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
# GUI なし（物理シミュレーションのみ）
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
bash scripts/run_site20_gazebo.sh sun_elevation:=15 sun_intensity:=3 texture_size:=2
# 毎回違うクレーター配置
bash scripts/run_site20_gazebo.sh crater_count:=64 crater_seed:=-1
# 特定の配置を再現、または追加クレーターを無効化
bash scripts/run_site20_gazebo.sh crater_seed:=42
bash scripts/run_site20_gazebo.sh crater_count:=0
```

`sun_azimuth` は地図グリッド +X から +Y 向きに測った太陽の方位です。
黒い背景と弱い環境光を使い、低い太陽からの直接光と長い影を表現します。
南極の低い日照角を参考にした固定照明で、日時による天体暦や照度の測定値は再現していません。
`sun_intensity` は Gazebo の表示用倍率です。
[NASA の月の南極の照明資料](https://science.nasa.gov/resource/shadows-near-the-moons-south-pole/)を参考にしています。

地面には既存の `assets/Textures/LunarRegolith8k/` から作った 2K テクスチャを同梱しています。
色画像と OpenGL ノーマルマップを使い、2 m 四方で繰り返します。
粒状の凹凸は見た目のみで、車輪の接触地形は 5 m の DEM です。
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
# +X 方向へ 0.1 m/s。Ctrl+C 後は必ず停止コマンドを送ります。
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
# モデルを削除して、保存された初期位置・姿勢へ再作成
ros2 run lumos1_rockerless_URDF_description respawn_rover.py --output build/gazebo_site20
```

ROS トピック: `/cmd_vel`, `/odom`, `/joint_states`, `/tf`, `/tf_static`, `/clock`。
CAD メッシュは元の mm→m スケールを維持し、`base_footprint` で ROS の +X 前方へ変換しています。
左右の車輪軸を統一し、4 輪の DiffDrive と円柱の車輪衝突形状を使用します。
これは剛体地形の接触モデルです。OmniLRS の土壌変形は移植していません。
旧 ROS 1 の `.launch` ファイルは ROS 2 では使用しません。
ROS 2 のエントリーポイントは `site20.launch.py`（別名 `gazebo.launch.py`）です。

依存パッケージを追加する必要がある環境では:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  python3-colcon-common-extensions python3-numpy python3-pil python3-yaml
```

参照: [Gazebo の ROS 2 移行手順](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/)、
[モデルのスポーン](https://gazebosim.org/docs/harmonic/ros2_spawn_model/)、
[物理エンジンと衝突検出器](https://gazebosim.org/api/sim/8/physics.html)。
