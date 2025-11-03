# mapping

Purpose
-------
Maintain a global map of cones by fusing successive 3D detections using a
Kalman filter. Performs data association, state update and map publishing.

Key files
- `src/cone_mapper.cpp`, `src/cone_mapper_node.cpp` — Kalman filter logic and
  ROS node.
- `include/cone_mapper/cone_mapper.hpp` — mapper interface and types.

Topics
- Subscribes: Marker/point arrays from `localisation/`.
- Publishes: persistent map (e.g., `visualization_msgs/MarkerArray`) of cone
  states (position + uncertainty) for visualization and planner consumption.

Run (example)
--------------
```bash
ros2 launch mapping launch.py
```

Notes
- Data association is critical: adjust thresholds and gating parameters in
  `config/params.yaml`.
- The mapper expects reasonably accurate depth inputs — large depth noise will
  increase mapping uncertainty.
# cone_mapper