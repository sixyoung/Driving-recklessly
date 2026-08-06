# Real-time expected-channel visualization

The active node is:

```text
realtime_channel_visualizer_row.py
```

It renders a wide, low horizontal layout:

```text
CARLA scene | ST speed | SL lateral | AT acceleration
             live behavior-identification result strip
```

The monitor waits for the fixed reckless-driving vehicle. Sampling starts only after that vehicle appears in `/veh_state_sequences`.

## Channel behavior

### ST speed channel

Default expected interval:

```text
6.0 m/s <= speed <= 15.0 m/s
```

The initial low-speed period is not classified as abnormal. It is shown as a grey `Startup exemption` region. With the default configuration, low-speed evaluation begins only after five seconds and after the vehicle has reached the normal speed interval at least once.

After activation, speed below the lower bound or above the upper bound is drawn as a thick red segment.

### SL lateral channel

For straight-road scenarios, the complete Frenet lateral-offset history is compared with:

```text
-1.5 m <= lateral offset <= 1.5 m
```

Out-of-channel sections are drawn as thick red segments. Left- and right-turn scenarios currently display `NOT EVALUATED` to avoid false positives.

### AT acceleration channel

Default expected interval:

```text
-3.0 m/s² <= acceleration <= 3.0 m/s²
```

Out-of-channel sections are drawn as thick red segments.

## Behavior result strip

The lower strip displays four live channel-based categories:

```text
正常 | 速度异常 | 换道异常 | 加速度异常
```

An active anomaly is highlighted in red. `正常` is highlighted in green when no channel is abnormal. Results are held briefly to avoid flicker.

## Complete history and layout

The monitor retains the complete trajectory from the target vehicle's first state until shutdown. The default canvas is:

```text
2880 x 560
```

The three channel plots receive more width than the CARLA scene pane. Set `keep_full_history: false` only when a rolling time window is explicitly required.

## Pull, build, and run

```bash
git switch feature/realtime-channel-visualization
git pull --ff-only origin feature/realtime-channel-visualization
catkin_make
source devel/setup.bash
```

Run with a live window and animated GIF:

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --visualize_channels \
  --save_gif
```

Default output directory:

```text
~/scenario_gifs/
```

## GIF finalization

During recording, the encoder writes:

```text
<scenario>_<time>_channels.part.gif
```

This temporary name is expected while the scenario is running. ffmpeg now runs in an independent process group, so roslaunch `Ctrl+C` cannot terminate it prematurely. Press `Ctrl+C` once and wait for:

```text
Finalizing animated GIF with ... frames...
GIF saved: ..._channels.gif
```

After a successful shutdown, the `.part.gif` is atomically renamed to the final `.gif`. At least two frames are written, so a successfully saved result is an animated GIF rather than a static image.

## Main configuration

```yaml
gif_width: 2880
gif_height: 560
keep_full_history: true
max_history_seconds: 0.0
slow_speed_threshold: 6.0
overspeed_threshold: 15.0
speed_startup_grace_seconds: 5.0
slow_require_previous_normal_speed: true
sl_lateral_limit: 1.5
max_accel: 3.0
result_hold_seconds: 1.5
```

Configuration files:

```text
src/behavior_identification/config/realtime_channel_visualizer.yaml
src/behavior_identification/config/behavior_identification.yaml
```
