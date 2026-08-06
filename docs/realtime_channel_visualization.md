# Real-time expected-channel visualization

The optional monitor is arranged in one row:

```text
CARLA real scene | ST speed channel | SL straight-road channel | AT acceleration channel
```

The monitor waits for the fixed vehicle whose `VehicleConfig.is_random_behavior_vehicle` is `true`. Sampling, drawing, and GIF recording begin only after that vehicle appears in `/veh_state_sequences`.

## Channel definitions

### ST: speed-time expected channel

The actual speed curve is compared with the configured expected interval:

```text
slow_speed_threshold <= speed <= overspeed_threshold
```

Default bounds:

```text
6.0 m/s <= speed <= 15.0 m/s
```

Samples below the lower boundary are marked `TOO SLOW`; samples above the upper boundary are marked `TOO FAST`.

### SL: straight-road lateral expected channel

SL is currently evaluated only for straight-road scenarios. The vehicle position is projected onto the frozen planned route and the lateral offset is compared with:

```text
-sl_lateral_limit <= lateral offset <= sl_lateral_limit
```

Default limit:

```text
±1.5 m
```

For `road_option=1` or `road_option=2` (left/right turn), the SL panel remains visible but displays `NOT EVALUATED`. This avoids treating normal turning motion as an abnormal lane departure.

### AT: acceleration-time expected channel

The actual longitudinal acceleration is compared with:

```text
-max_accel <= acceleration <= max_accel
```

Default limit:

```text
±3.0 m/s²
```

## Pull and build

```bash
git switch feature/realtime-channel-visualization
git pull --ff-only origin feature/realtime-channel-visualization
catkin_make
source devel/setup.bash
```

## Run

Live window:

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --visualize_channels
```

Live window and GIF:

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

## Stopping midway

Pressing `Ctrl+C` is supported. The visualization node handles `SIGINT` and `SIGTERM`, closes the ffmpeg input pipe, and finalizes the current GIF before exiting. Use the normal `Ctrl+C` path rather than closing the terminal or force-killing the process.

## Configuration

```text
src/behavior_identification/config/realtime_channel_visualizer.yaml
src/behavior_identification/config/behavior_identification.yaml
```

The main parameters are:

```yaml
slow_speed_threshold: 6.0
overspeed_threshold: 15.0
sl_lateral_limit: 1.5
max_accel: 3.0
```
