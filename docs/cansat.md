# CanSatをOmniLRSで動かす

`assets/cansat.fbx`から変換したロボットは
`assets/USD_Assets/robots/cansat/cansat.usda`です。
外観は同じディレクトリの`cansat_visual.usdc`を相対参照します。
この2ファイルを一緒に配置してください。生成済みなので初回起動時の変換は不要です。

## 起動と操作

リポジトリのルートから、既存のROS 2実行スクリプトで起動します。

```bash
bash scripts/run_ros2.sh sim environment=lunalab robot=cansat
```

別のターミナルで操作します。両方のスクリプトは同じROSドメイン（既定42）を使います。

```bash
bash scripts/run_ros2.sh teleop --speed 0.1 --turn 0.5
```

`i`: 前進、`,`: 後退、`j`/`l`: 左右旋回、`k`/スペース: 停止。
`geometry_msgs/msg/Twist`の`/cmd_vel`に、前進速度[m/s]を`linear.x`、
旋回速度[rad/s]を`angular.z`で送っても操作できます。
0.5秒間指令が届かなければ車輪の目標速度を0にします。

Lunaryardでも読み込めますが、初期位置は環境の地形高さに合わせて指定してください。

```bash
bash scripts/run_ros2.sh sim environment=lunaryard_20m robot=cansat \
  'environment.robots_settings.parameters.pose.position=[10.0,10.0,0.3]'
```

## 設定した物理構造

- 座標系: +Xが前方（カメラ側）、+Yが左、+Zが上。原点は車軸の中点。
- `base_link`: 車体と電子部品をまとめた剛体。衝突は車体の箱と後方スタビライザの凸包。
- `left_wheel_link`、`right_wheel_link`: タイヤ、ハブ、軸をまとめた左右の剛体。
- `left_wheel_joint`、`right_wheel_joint`: +Y軸回りの連続回転関節。
  速度駆動、位置剛性0、最大トルク0.2 N·m、最大速度20 rad/s。
- タイヤの衝突形状: 半径60.706 mm、幅25 mmの円柱。
  トレッドを滑らかな円柱に近似しています。左右の接触形状中心間は168.6 mm。
- 車軸の関節位置間は179.6 mm。接触中心間と異なるため、走行計算には168.6 mmを使います。
- 車体0.8 kg、車輪各0.1 kg（合計1.0 kg）。**質量・摩擦・モータ定数は仮値**です。
  実機の値に合わせて`cfg/robot/cansat_physics.json`を編集し、再変換してください。
  既存の月面重力設定1.62 m/s²を使います。

左右関節の正速度が前進になるよう軸を統一しています。
ROSの指令はシミュレーションスレッドで既存の`ArticulationControl`に渡します。
`differential_drive`設定を持つロボットだけにこの購読を追加するため、既存ロボットの
USD内のOmniGraphとは競合しません。
この構成では、[NVIDIAの車輪制御の説明](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/ros2_tutorials/tutorial_ros2_drive_turtlebot.html)
と同じく、機体速度を車輪速度に変換してArticulation Controllerに渡しています。

カメラ等のCAD外観は保持していますが、このモデルにカメラ・IMUのセンサ機能は付与していません。
ROS 2走行用の設定です。Yamcsのテレメトリ・ミッション固有機能は別途設定が必要です。

## 再変換と検証

```bash
pixi run -e default python scripts/convert_cansat.py
pixi run -e default python scripts/check_cansat.py
pixi run -e default python -m unittest discover -s test -p 'test_cansat*.py'
```

変換はKitを起動せず、Isaac Sim付属のUSDライブラリを使用します。
FBXのmm単位、各部品のローカル変換（メッシュの10倍スケールを含む）、
179個のメッシュインスタンス、面ごとのマテリアル・法線・UVを反映します。
このFusionの静的CADエクスポート用で、アニメーションや特殊なピボット等は拒否します。
再エクスポート後にタイヤのアセンブリ名を変更すると、変換側の選択条件も修正が必要です。
寸法が変わった場合は`manifest.json`の値を`cfg/robot/cansat.yaml`の
`wheel_radius`・`wheel_track`に反映してください。
最大速度を変更した場合もJSONとYAMLを揃えてください。

GPUが使用できる環境では、平面上での実際の前進・左旋回・停止を確認できます。

```bash
pixi run -e ros2 python scripts/check_cansat.py --simulate
```

構造・制御の検証は実施済みです。GPUの利用にはサンドボックス外での実行が必要でした。
RTX 3060上で前進走行と描画を確認しています。
単独の旋回テストには期待角度との不一致が残っています。PVは確認済みの直進走行を使用しています。

## PVの録画

`scripts/record_cansat_pv.py`は、`environment=cansat_pv`と`robot=cansat`を使い、
20秒・30 fps・1280×720の動画を録画します。暖色の斜光と寒色の補助光を配置し、
カメラが前方斜めから側面・後方へ移動しながらローバーを追います。
走行速度は0.06 m/sで、車輪の位置から砂のDEMを変形させます。
PV用の轍は深さ約2〜4 mmに設定しています。表示用の砂が変形し、
衝突には初期地形のコピーを使います（既存のOmniLRS変形モードと同じ構成）。

```bash
bash scripts/run_ros2.sh pv --output recordings/cansat_pv_new.mp4
```

生成済みの共有用動画は`recordings/cansat_pv_20s_h264.mp4`です。
走行軌跡・フレーム数・変形量は`recordings/cansat_pv_20s.json`に保存します。
PVは砂の地形が沈み込むことと実際の移動距離を検証してから完了とします。
