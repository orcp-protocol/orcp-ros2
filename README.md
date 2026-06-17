# orcp-ros2

ROS 2 driver for [ORCP](https://github.com/orcp-protocol/orcp)-compliant motor
controllers. A `rclpy` node bridges ROS 2 to a controller using the
[`orcp`](https://github.com/orcp-protocol/orcp-python) Python client library.

It works against real hardware **or**, with no hardware at all, the
[ORCP simulator](https://github.com/orcp-protocol/orcp-sim) — so you can develop
and prove the whole stack on a laptop.

## What the node does

`orcp_driver`:

| Topic / TF | Direction | Type | Purpose |
|------------|-----------|------|---------|
| `/cmd_vel` | subscribe | `geometry_msgs/Twist` | drive commands → ORCP `CMD_VEL` |
| `/odom` | publish | `nav_msgs/Odometry` | dead-reckoned pose + velocity |
| `/battery_state` | publish | `sensor_msgs/BatteryState` | battery voltage |
| `odom → base_link` | publish | TF | robot pose transform |

Parameters: `port` (controller location), `preset` (`SLOW`/`NORMAL`),
`odom_frame`, `base_frame`, `publish_rate`.

## Running on Linux (native ROS 2)

With ROS 2 Jazzy installed and `orcp` (+ optionally `orcp-sim`) on your Python path:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch orcp_ros2 sim_bringup.launch.py          # sim + driver
# in another terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard  # drive with the keyboard
```

## Running on macOS / Windows (containerised)

ROS 2 has no native macOS build, so run it in a Linux container. The included
`Dockerfile` builds an image with ROS 2 Jazzy, the teleop app, and our libraries'
dependencies.

```bash
# one-time: build the dev image (from this repo's root)
docker build -t orcp-ros2-dev .

# start a container with the ORCP repos mounted (orcp-ros2 next to orcp-python/orcp-sim)
docker run -dit --init --name orcp -v /path/to/orcp:/work orcp-ros2-dev sleep infinity

# install the ORCP libraries into the container (editable)
docker exec orcp pip install --break-system-packages --no-deps -e /work/orcp-python -e /work/orcp-sim

# build the ROS 2 package
docker exec orcp bash -lc 'source /opt/ros/jazzy/setup.bash && cd /work/orcp-ros2 && colcon build --symlink-install'
```

Then use it (each command in its own terminal; `-it` gives an interactive shell):

```bash
# Terminal 1 — simulator + driver
docker exec -it orcp bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /work/orcp-ros2/install/setup.bash; \
   ros2 launch orcp_ros2 sim_bringup.launch.py'

# Terminal 2 — keyboard teleop (publishes /cmd_vel)
docker exec -it orcp bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /work/orcp-ros2/install/setup.bash; \
   ros2 run teleop_twist_keyboard teleop_twist_keyboard'

# Terminal 3 (optional) — watch the robot move
docker exec -it orcp bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /work/orcp-ros2/install/setup.bash; \
   ros2 topic echo /odom --field pose.pose.position'
```

## Visualization (Foxglove)

To *see* the robot move in 3D, use [Foxglove](https://foxglove.dev) — it connects
to ROS 2 over a websocket, so it needs no X11/GUI forwarding (ideal on macOS).

Launch the sim + driver **plus** the robot model and a Foxglove bridge:

```bash
docker exec -it orcp bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /work/orcp-ros2/install/setup.bash; \
   ros2 launch orcp_ros2 viz.launch.py'
```

This also runs `foxglove_bridge` on port **8765** (published to your host by the
`docker run -p 8765:8765` above). Then in Foxglove (the web app at
https://app.foxglove.dev or the desktop app):

1. **Open connection** → choose the **"Foxglove WebSocket"** connection type.
   > ⚠️ Not "Rosbridge" — that's a different protocol/server. We run
   > `foxglove_bridge`, which is the *Foxglove WebSocket* type.
2. URL: **`ws://localhost:8765`** → **Open**.
3. Add a **3D** panel and set its **fixed frame** to **`odom`**. You'll see the
   robot — a blue box with two wheels.
4. Drive it with the keyboard teleop (a separate terminal) and watch it move:
   ```bash
   docker exec -it orcp bash -lc \
     'source /opt/ros/jazzy/setup.bash; source /work/orcp-ros2/install/setup.bash; \
      ros2 run teleop_twist_keyboard teleop_twist_keyboard'
   ```

> If the browser app refuses to connect to `ws://localhost` from an `https://`
> page (mixed-content blocking), use the **Foxglove desktop app** — same steps,
> no restriction.

## Targeting real hardware

Skip the simulator and point the driver at a real controller:

```bash
ros2 run orcp_ros2 orcp_driver --ros-args -p port:=/dev/ttyACM0
# or over WiFi/TCP:
ros2 run orcp_ros2 orcp_driver --ros-args -p port:=socket://192.168.4.1:3333
```

## License

MIT — see [LICENSE](src/orcp_ros2/LICENSE).
