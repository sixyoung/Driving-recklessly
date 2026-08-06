# Real-time expected-channel visualization

The active node is:

```text
realtime_channel_visualizer_row.py
```

Layout:

```text
CARLA real scene | ST longitudinal-time | SL lateral | AT acceleration
                  behavior-identification result strip
```

## Channel definitions

### ST: longitudinal displacement-time channel

ST is an S-T channel. Its axes are:

```text
x: time from target appearance (s)
y: cumulative longitudinal displacement s (m)
```

The configured speed bounds generate the lower and upper S-T envelope after
the startup-exemption interval:

```text
s_lower(t) = s_anchor + slow_speed_threshold * (t - t_anchor)
s_upper(t) = s_anchor + overspeed_threshold * (t - t_anchor)
```

The y-axis is not instantaneous speed. Current speed is shown only in the ST
title as auxiliary information.

### SL: straight-road lateral channel

The target is projected onto its frozen planned route. The default legal band
is `+-1.5 m`. SL is not evaluated for left/right-turn scenarios.

### AT: acceleration-time channel

The default legal band is `+-3.0 m/s^2`.

## Abnormal highlighting

All channels use the same visual rules:

- expected corridor: light green;
- normal trajectory: blue;
- channel violation: thick red segment;
- startup exemption: grey region.

## Fonts

The node selects fonts in this order:

Chinese:

```text
SimSun / NSimSun / Songti SC / STSong / AR PL UMing CN /
Noto Serif CJK SC / Source Han Serif SC
```

Latin letters, numbers and symbols:

```text
Times New Roman / Liberation Serif / Nimbus Roman
```

Check available fonts:

```bash
fc-list | grep -Ei 'SimSun|Songti|STSong|UMing|Noto Serif CJK|Source Han Serif|Times New Roman|Liberation Serif|Nimbus Roman'
```

When no Song/Ming CJK font is found, the node prints a warning. Install a
Song/Ming-style Chinese font before running again; do not rely on DejaVu Serif
for Chinese text.

## High-quality GIF encoding

Frames are first written losslessly to:

```text
<scenario>_<time>_channels.part.mkv
```

On shutdown, ffmpeg builds a full 256-color palette and creates:

```text
<scenario>_<time>_channels.part.gif
```

After successful conversion it is atomically renamed to:

```text
<scenario>_<time>_channels.gif
```

This two-stage process avoids the yellow/green edge contamination produced by
direct low-quality GIF encoding. The ffmpeg processes use independent process
groups, so roslaunch SIGINT cannot terminate them before finalization.

## Pull and build

```bash
git switch feature/realtime-channel-visualization
git pull --ff-only origin feature/realtime-channel-visualization
catkin_make
source devel/setup.bash
```

## Run

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

Press `Ctrl+C` once. The launcher waits up to 180 seconds for lossless-video
closure and palette conversion. Wait until both of these messages appear:

```text
Finalizing high-quality animated GIF ...
GIF saved: ..._channels.gif
```

## Main parameters

```yaml
gif_width: 3000
gif_height: 620
slow_speed_threshold: 6.0
overspeed_threshold: 15.0
speed_startup_grace_seconds: 5.0
slow_require_previous_normal_speed: true
max_position_step: 20.0
sl_lateral_limit: 1.5
max_accel: 3.0
result_hold_seconds: 1.5
```
