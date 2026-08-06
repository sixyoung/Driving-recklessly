# Real-time CARLA / ST / SL / AT visualization

This branch adds a one-row visualization and GIF recorder:

```text
| CARLA real scene | ST channel | SL channel | AT channel |
```

The node does not start channel sampling immediately. It waits for the fixed vehicle whose `VehicleConfig.is_random_behavior_vehicle` field is `true`. It then waits until the same CARLA actor ID appears in `/veh_state_sequences`. Only after both conditions are satisfied does it:

- start the time axis;
- draw the channel curves;
- capture CARLA frames;
- start writing GIF frames.

No blank waiting frames are written to the GIF.

## Pull and build

```bash
git switch feature/realtime-channel-visualization
git pull --ff-only origin feature/realtime-channel-visualization

catkin_make
source devel/setup.bash
```

Install the screen-capture dependencies:

```bash
sudo apt update
sudo apt install python3-numpy python3-matplotlib python3-pil ffmpeg xdotool
```

`mss` is optional but faster than Pillow:

```bash
python3 -m pip install mss
```

## Run

Start CARLA first, then run:

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --visualize_channels
```

Save the one-row GIF:

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

The monitor can still be forced to a specific role or actor ID:

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --visualize_channels \
  --visualized_role heroL0
```

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --visualize_channels \
  --vehicle_id 123
```

## CARLA real-scene panel

The first panel captures the actual `CarlaUE4` window. The node uses `xdotool` to locate that window by title and then captures it with `mss` or Pillow.

Configuration:

```yaml
carla_window_title: "CarlaUE4"
screen_auto_window: true
screen_left: 0
screen_top: 0
screen_width: 1280
screen_height: 720
```

The configured screen rectangle is only a fallback when the CARLA window cannot be detected automatically.

Check the actual window title with:

```bash
xdotool search --name Carla getwindowname %@
```

Then update `carla_window_title` in:

```text
src/behavior_identification/config/realtime_channel_visualizer.yaml
```

## Target selection

Automatic mode is strict:

```text
VehicleConfig.is_random_behavior_vehicle == true
```

The node no longer falls back to the first configured vehicle. Before the fixed reckless vehicle appears, the window stays on a static waiting screen.

Expected log sequence:

```text
Locked fixed reckless vehicle: heroL0 (123), random=1
Target appeared in /veh_state_sequences; drawing and GIF start now.
Frozen expected path loaded for heroL0.
```

## Channel data

The node subscribes to:

```text
/carla/vehicle_config
/veh_state_sequences
/traffic_light/phases
/scenario_status
/carla_waypoint_publisher/get_path
```

Each valid target-state update prints a throttled diagnostic line:

```text
Channel input heroL0(123): speed=8.412 accel=0.537 pose=120 speed_n=120 accel_n=120
```

This line is the first check when the plots appear empty or constant.

The AT panel reads speed from `VehStateSequence.speed`. When that field is absent, it calculates speed from `VehStateSequence.twist`. The classifier already stores scalar longitudinal acceleration in `accel.linear.x`, so the visualization uses that value directly rather than projecting it by vehicle heading a second time.

## SL definition

SL uses the frozen initial route returned by the waypoint service:

```text
l(t) = (p(t) - p_r(s*))^T n(s*)
Delta l(t) = l(t) - l_baseline
```

A normal curved intersection turn remains near zero, while a true road-relative lane shift produces a persistent change in `Delta l`.

## Troubleshooting

### The window remains on the waiting screen

Check the reckless-vehicle flag:

```bash
rostopic echo /carla/vehicle_config | grep -E "role_name|carla_id|is_random_behavior_vehicle"
```

Check that the same ID appears in the state sequence:

```bash
rostopic echo -n 1 /veh_state_sequences
```

### The channel values remain zero

Read the diagnostic line printed by the visualization node. Also inspect the selected actor directly:

```bash
rostopic echo -n 1 /veh_state_sequences
rostopic echo -n 1 /carla/objects
```

The title must show the expected reckless role and actor ID. The new version never silently selects the first vehicle.

### The CARLA panel is blank

```bash
which xdotool
xdotool search --name CarlaUE4 getwindowgeometry
python3 -c "from PIL import ImageGrab; print('Pillow capture available')"
```

When the CARLA title differs, change `carla_window_title`. When automatic window detection is unavailable, set the fallback screen rectangle in the YAML file.

### GIF is empty or incomplete

Use `Ctrl+C` or normal scenario completion so ffmpeg receives EOF and finalizes the GIF:

```bash
ffmpeg -version
ls -lh ~/scenario_gifs
```

The GIF starts only after the fixed reckless vehicle first appears in `/veh_state_sequences`.
