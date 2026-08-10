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

## Target-following CARLA scene

The left panel no longer depends on desktop capture as its primary source. The
monitor connects to CARLA through the Python API and attaches a
`sensor.camera.rgb` directly above the selected target vehicle. The default
view is a square `720 x 720` bird's-eye image at `35 m`, pitched straight down
at `-90 degrees`. `Rigid` attachment is preferred so the view follows the
vehicle position and heading exactly; `SpringArmGhost` and `SpringArm` remain
compatibility fallbacks. The sensor is destroyed when the target changes or the
monitor exits. CarlaUE4 window capture remains available only as a live-panel
fallback and is never written into the synchronized camera GIF.

The target is selected from the active scenario JSON. The vehicle marked with
`is_random_behavior_vehicle: true` (or the unique vehicle whose behavior has
`parameters.abnormal: true`) supplies the stable `role_name`. Its numeric CARLA
actor ID is resolved from `/carla/vehicle_config` after spawning, so no actor ID
is fixed across scenario runs. If a scenario has no abnormal or designated
vehicle, the first normal vehicle role is used instead. Explicit role or ID
launch arguments remain available only as diagnostic overrides.

## Channel definitions

### ST: rolling longitudinal displacement-time channel

ST is an S-T channel. Its axes are:

```text
x: time from target appearance (s)
y: cumulative longitudinal displacement s (m)
```

For ordinary speed/lane scenarios, before the target reaches the configured
legal-speed range the complete output remains blank: there is no CARLA frame
and no ST/SL/AT drawing. The first legal-speed sample atomically starts all
four panels, resets time to `t=0`, resets ST displacement to `s=0`, and creates
the first prediction. Acceleration-anomaly scenarios are the explicit
exception: the first vehicle state at birth starts the camera, actual ST
trajectory, SL and AT immediately and resets them to `t=0/s=0`, even when the
reported initial speed is `0 m/s`. The expected ST corridor remains absent
until the target first reaches `st_activation_speed`. Its first
segment is then anchored at that sample without resetting the birth-based time
or displacement history. Further predictions are created every
`st_segment_seconds`:

```text
v_ref = clamp(v_anchor, legal speed range)
a_ref = clamp(a_anchor, +-st_reference_accel_limit)
s_center(t) = s_anchor + v_ref * dt + 0.5 * a_ref * dt^2
s_lower(t) = s_center(t) - st_channel_half_width
s_upper(t) = s_center(t) + st_channel_half_width
```

The first segment is anchored to the measured vehicle state. Each later refresh
starts at the previous prediction's endpoint and only updates the expected
slope. A bounded tracking correction uses the measured position error to adjust
the next slope without moving the join. Acceleration is bounded and the expected
speed cannot move below or above the configured legal range outside a signal
braking zone, so slow and overspeed trajectories cannot drag the channel along
with an anomaly. The displayed history is filled as one continuous corridor, so
refresh boundaries never introduce gaps or sawtooth jumps. Its two boundaries
remain parallel and the configured `12 m` half-width stays constant instead of
growing for the whole scenario.
The y-axis is not instantaneous speed; current speed is shown only in the ST
title as auxiliary information.

For an acceleration-anomaly target, the interval from birth until the first
`st_activation_speed` sample is shown as a grey waiting region. The actual ST
trajectory may start at zero speed there, but there is no green expected
corridor and no ST violation evaluation. Once the target reaches the
activation threshold, the normal `[6, 15] m/s` speed limits, progress limits,
and signal constraints apply continuously. AT values are retained and
evaluated from the first
available sample, including the birth transition.

ST evaluates both displacement and slope. A sample is abnormal when it leaves
the displacement corridor, exceeds `overspeed_threshold`, remains below
`slow_speed_threshold` for `st_slow_violation_duration`, or crosses a matched
red/yellow stop line. This keeps the wider visual corridor from hiding a speed
anomaly.

The monitor also checks every vehicle in `/veh_state_sequences` for a credible
same-lane lead vehicle. Heading and lateral gates reject crossing or adjacent
traffic. A lead vehicle becomes a longitudinal constraint only when its bumper
gap is inside a safe-following envelope composed of standstill gap, time
headway, relative-speed braking distance and a small tolerance. While that
constraint is active, a rolling ST segment may legitimately reduce its
reference speed below `slow_speed_threshold`, including a complete stop, and
the title reports `FOLLOWING LEAD VEHICLE`. Slow progress is then not an
anomaly, while overspeed, excessive progress and stop-line
crossing remain fully evaluated. A merely visible but distant vehicle does not
disable slow-driving detection.

Longitudinal displacement uses position increments with a speed/time-aware
plausibility limit. When rendering delays make state samples sparse, a normal
step larger than `max_position_step` is accepted if it agrees with the elapsed
time and vehicle speed. A true teleport is rejected and replaced by
speed-integrated progress, so one dropped sample cannot freeze the ST distance
and create a false `PROGRESS TOO SLOW` result.

### Fixed axes and progressive drawing

All three channel plots use stationary axes by default:

```text
time: 0 .. 60 s
ST displacement: -10 .. 1000 m
SL lateral offset: -3 .. 3 m
AT acceleration: -5 .. 5 m/s^2
```

The limits do not rescale as samples arrive. ST appends each rolling prediction
segment, while the SL and AT expected bands extend only to the latest sample.
This leaves the remaining fixed time range blank and makes channel generation
visible from left to right. All channel legends remain anchored in the upper
left corner.

### Signal and stop-line constraint

The node subscribes to `/traffic_light/phases` and projects each reported stop
line onto the target's frozen route. This logic is enabled only when the target
`VehicleConfig.scene_type` is exactly `tl_intersection`; pseudo-light messages
in `untl_intersection`, merge or other scenes are ignored. Only the nearest
route-matched line is used.

- Green: the rolling corridor proceeds normally. Within
  `st_signal_braking_distance` of a matched stop line, bounded observed
  deceleration and low speed are accepted so a normal vehicle can prepare to
  stop without a slow-driving false positive.
- Red or yellow: the expected center line applies a comfortable braking profile,
  then stays behind `stop_line_buffer` when the constraint arrives with enough
  stopping distance.
- A late phase change never moves an existing corridor backward. Continuity is
  preserved and any unavoidable stop-line crossing is reported as a violation.
- Crossing the matched stop line on red/yellow is highlighted as a violation.
- Stale signal messages are ignored, preventing an old red state from affecting
  a later or non-signalized scene.

### SL: straight-road lateral channel

The target is projected onto its frozen planned route. The default legal band
is `+-1.5 m`. SL is not evaluated for left/right-turn scenarios.

### AT: acceleration-time channel

The default legal band is `+-3.0 m/s^2`.

## Plot styling

Visualization follows these rules:

- expected corridor: light green;
- actual ST trajectory: one continuous blue line in every detection state;
- ST violations: the violating trajectory samples are overlaid as one black
  dashed segment and are also reported in the title and result strip;
- traffic-light constraints: one red/yellow dash-dot stop line only while that
  signal actually constrains the vehicle;
- waiting-for-speed interval: grey region; acceleration scenes show their
  actual ST trajectory there but no expected corridor.

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

Camera and channel frames are sampled as one pair by a shared clock. Recording
starts only after both the legal-speed channel data and the real CARLA camera
frame are available. If either side is unavailable, neither frame is written.
Both outputs therefore have the same FPS, frame count, start frame and duration,
which allows them to play together on one PowerPoint slide.

Rendering the `3000 x 620` channel canvas can be slower than the configured GIF
frame rate. Every paired frame therefore stores the current channel simulation
time. During final conversion, frame delays are proportional to simulation-time
gaps and the complete timeline is rescaled to `gif_playback_duration_seconds`
(4.5 seconds by default). Later sparse frames no longer play at the same delay
as early dense frames, so vehicle speed remains visually meaningful. Set this
parameter to `0` to use the recorded wall-clock duration for the total playback
time. Camera and channel outputs use the same timestamps and target duration,
so their PowerPoint playback remains synchronized.

The paired frames are first written losslessly to:

```text
<scenario>_<time>_camera.part.mkv
<scenario>_<time>_channels.part.mkv
```

On shutdown, ffmpeg builds a full 256-color palette and creates:

```text
<scenario>_<time>_camera.part.gif
<scenario>_<time>_channels.part.gif
```

After successful conversion it is atomically renamed to:

```text
<scenario>_<time>_camera.gif
<scenario>_<time>_channels.gif
<scenario>_<time>_expected_channels.gif  # only with --showall
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
  --save_gif \
  --headless_channels
```

For the presentation page, add `--showall`. This enables synchronized output
of three files: `*_expected_channels.gif` (upper triptych with generated
expected channels and no behavior-identification overlays), `*_channels.gif`
(lower evaluated triptych), and `*_camera.gif` (the CARLA follow camera).
All three files share the same frame timestamps and playback duration.

`--headless_channels` is recommended when only the GIF pair is needed. In this
mode the hidden four-panel dashboard is not rendered; only the bird's-eye
camera and the three-channel GIF canvas are processed. This removes a complete
duplicate plot pass and raises the effective recorded frame rate without
changing the GIF timeline or target selection.

Default output directory:

```text
~/scenario_gifs/
```

Press `Ctrl+C` once. The launcher waits for both lossless videos to close and
both palette conversions to finish. Wait for the finalization message and two
`GIF saved` messages:

```text
Finalizing synchronized camera/channel GIFs ...
GIF saved: ..._camera.gif
GIF saved: ..._channels.gif
GIF saved: ..._expected_channels.gif
```

## Main parameters

```yaml
gif_width: 3000
gif_height: 620
gif_playback_duration_seconds: 4.5
follow_camera_enabled: true
carla_host: "localhost"
carla_port: 2000
follow_camera_width: 720
follow_camera_height: 720
follow_camera_fov: 75.0
follow_camera_sensor_tick: 0.1
follow_camera_offset_x: 0.0
follow_camera_offset_z: 35.0
follow_camera_pitch: -90.0
slow_speed_threshold: 6.0
overspeed_threshold: 15.0
st_activation_speed: 6.0
speed_startup_grace_seconds: 0.0
st_segment_seconds: 1.5
st_channel_half_width: 8.0
st_tracking_correction_gain: 1.0
st_reference_accel_limit: 3.0
st_signal_braking_distance: 80.0
st_slow_violation_duration: 2.0
fixed_channel_axes: true
channel_time_axis_seconds: 60.0
st_axis_min_distance: -10.0
st_axis_max_distance: 1000.0
sl_axis_abs_limit: 3.0
at_axis_abs_limit: 5.0
max_position_step: 20.0
traffic_light_msg_timeout: 1.0
use_scene_type_for_signal_control: true
signalized_scene_type_value: "tl_intersection"
stop_line_lateral_gate: 8.0
stop_line_cross_margin: 0.5
stop_line_buffer: 2.0
traffic_stop_decel: 3.0
sl_lateral_limit: 1.5
max_accel: 3.0
result_hold_seconds: 1.5
```
