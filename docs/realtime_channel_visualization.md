# Real-time expected-channel visualization

The active monitor is a single self-contained ROS node:

```text
realtime_channel_visualizer_row.py
```

It renders one horizontal row:

```text
CARLA real scene | ST speed channel | SL straight-road channel | AT acceleration channel
```

The monitor waits for the fixed vehicle whose `VehicleConfig.is_random_behavior_vehicle` is `true`. Sampling begins only after that vehicle appears in `/veh_state_sequences`.

## Channel definitions

### ST: speed-time expected channel

```text
slow_speed_threshold <= speed <= overspeed_threshold
```

Default:

```text
6.0 m/s <= speed <= 15.0 m/s
```

### SL: straight-road lateral expected channel

The vehicle is projected onto its frozen planned route. Default channel:

```text
-1.5 m <= lateral offset <= 1.5 m
```

SL is evaluated only for straight-road scenarios. For `road_option=1` or `road_option=2`, the panel displays `NOT EVALUATED`.

### AT: acceleration-time expected channel

```text
-3.0 m/s² <= acceleration <= 3.0 m/s²
```

## Pull and build

```bash
git switch feature/realtime-channel-visualization
git pull --ff-only origin feature/realtime-channel-visualization
catkin_make
source devel/setup.bash
```

A rebuild is required after pulling because Catkin must refresh the installed Python node.

## Run

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --visualize_channels \
  --save_gif
```

Default GIF directory:

```text
~/scenario_gifs/
```

The launcher now verifies that `/realtime_channel_visualizer` appears in `rosnode list`. If it does not start within 30 seconds, the scenario is stopped and an explicit build/source error is printed.

## GIF reliability

The GIF encoder is opened only after the first complete Matplotlib frame has been rendered. Its input dimensions are taken from the actual canvas rather than assumed from the YAML values.

Recording is first written to:

```text
<scenario>_<time>_channels.part.gif
```

After ffmpeg exits successfully and at least one frame exists, it is atomically renamed to:

```text
<scenario>_<time>_channels.gif
```

A failed or zero-frame recording is deleted instead of leaving an empty GIF.

Expected logs:

```text
Expected-channel monitor started
Channel monitor ROS node is running
Target appeared in /veh_state_sequences
GIF encoder opened after first rendered frame
GIF recording active
GIF saved
```

## Stopping midway

Press `Ctrl+C` once. The launcher sends `SIGINT` to roslaunch and waits up to 25 seconds for the visualization node and ffmpeg to finish. Do not repeatedly press `Ctrl+C` or force-close the terminal during this period.

## Dependencies

```bash
sudo apt install python3-numpy python3-matplotlib python3-pil ffmpeg xdotool
python3 -m pip install mss
```

`mss` is optional but recommended for faster CARLA-window capture.

## Configuration

```text
src/behavior_identification/config/realtime_channel_visualizer.yaml
src/behavior_identification/config/behavior_identification.yaml
```

Main parameters:

```yaml
slow_speed_threshold: 6.0
overspeed_threshold: 15.0
sl_lateral_limit: 1.5
max_accel: 3.0
```
