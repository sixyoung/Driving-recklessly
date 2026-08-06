# Real-time ST / SL / AT channel visualization

This branch adds an optional four-panel Matplotlib monitor to the existing CARLA scenario launcher.

## Panels

1. **Scenario**: live top-down positions, target trajectory, frozen expected route and traffic-light stop points.
2. **ST**: signed longitudinal distance from the target vehicle to the active signal-controlled stop line.
3. **SL**: true lateral shift relative to the frozen initial route.
4. **AT**: longitudinal acceleration and speed with channel thresholds.

The SL panel does not use the straight chord between the first and last observed positions. Each vehicle position is projected into the Frenet frame of the initial planned route:

```text
l(t) = (p(t) - p_r(s*))^T n(s*)
Delta l(t) = l(t) - l_baseline
```

A normal curved turn therefore remains close to zero, while an actual lane shift produces a persistent change in `Delta l`.

## Pull and build

```bash
git switch feature/realtime-channel-visualization
git pull --ff-only origin feature/realtime-channel-visualization

catkin_make
source devel/setup.bash
```

Required runtime commands/packages:

```bash
python3 -c "import numpy, matplotlib, rospy"
ffmpeg -version
```

On Ubuntu/ROS Noetic, missing visualization dependencies can normally be installed with:

```bash
sudo apt update
sudo apt install python3-numpy python3-matplotlib ffmpeg
```

## Run with a live window

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --visualize_channels
```

The monitor first tries to select the vehicle marked as the random-behavior vehicle. If no such configuration is received, it selects the first configured vehicle after the fallback delay.

## Select a target explicitly

By CARLA actor ID:

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --visualize_channels \
  --vehicle_id 123
```

By role name:

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --visualize_channels \
  --visualized_role heroL0
```

When `--random_role` is supplied and `--visualized_role` is omitted, the same role is used as the visualization target:

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --random_role heroL0 \
  --visualize_channels
```

## Save a GIF

`--save_gif` automatically enables the monitor:

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --save_gif
```

Default output directory:

```text
~/scenario_gifs/
```

Custom output directory:

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --save_gif \
  --gif_output_dir /path/to/results
```

For a server without a desktop window:

```bash
python3 src/scenario_library/scripts/run_scenario.py \
  <scenario.json> \
  --save_gif \
  --headless_channels
```

Stop the scenario with `Ctrl+C`. ROS shutdown closes the ffmpeg input pipe and finalizes the GIF.

## Run the monitor separately

After starting the scenario normally:

```bash
rosrun behavior_identification realtime_channel_visualizer.py
```

Example private parameters:

```bash
rosrun behavior_identification realtime_channel_visualizer.py \
  _vehicle_id:=123 \
  _save_gif:=true \
  _gif_output_dir:=/tmp/channel_gifs
```

## Main ROS inputs

```text
/veh_state_sequences
/carla/vehicle_config
/traffic_light/phases
/scenario_status
/carla_waypoint_publisher/get_path
```

The route returned by `get_path` is frozen after target selection. Later abnormal lane-change replanning is not allowed to redefine the SL reference line.

## Configuration

Visualization settings:

```text
src/behavior_identification/config/realtime_channel_visualizer.yaml
```

The launch file also loads:

```text
src/behavior_identification/config/behavior_identification.yaml
```

This keeps speed thresholds in the monitor aligned with the identification node.

## Expected startup messages

A successful target and route setup prints messages similar to:

```text
Channel visualizer selected vehicle heroL0 (123), random=1
Frozen expected path loaded for heroL0 (123): 240 points.
```

When GIF output is enabled:

```text
Channel GIF output: /home/user/scenario_gifs/<scenario>_<time>_channels.gif
```

## Troubleshooting

### The Matplotlib window does not appear

Check the display environment:

```bash
echo $DISPLAY
python3 -c "import matplotlib.pyplot as plt; plt.plot([0,1]); plt.show()"
```

Use `--headless_channels --save_gif` on a machine without X11.

### The SL panel says it is waiting for the route

Check the route service:

```bash
rosservice list | grep get_path
rosservice info /carla_waypoint_publisher/get_path
```

Also verify that the selected target has published a `VehicleConfig` message.

### No target vehicle is selected

Inspect vehicle configurations and state IDs:

```bash
rostopic echo -n 1 /carla/vehicle_config
rostopic echo -n 1 /veh_state_sequences
```

Then pass `--vehicle_id` or `--visualized_role` explicitly.

### GIF is missing or empty

Check ffmpeg and the output directory:

```bash
which ffmpeg
ffmpeg -version
ls -ld ~/scenario_gifs
```

Always stop through `Ctrl+C` or normal scenario completion so the GIF encoder can finalize the file.
