# Autonomous Drone Exploration and Mapping (Crazyflie 2.1)

Autonomous exploration and mapping of an unknown indoor space with a **Crazyflie 2.1** and the **Multi-ranger deck**, in two stages:

| Folder | Localization | Decks |
|---|---|---|
| [`lighthouse/`](lighthouse) | Lighthouse (absolute position, millimetre-level accuracy) | Multi-ranger + Lighthouse |
| [`flowdeck/`](flowdeck) | Onboard sensors only (optical flow + height) | Multi-ranger + Flow deck v2 |

The drone:
1. takes off autonomously,
2. explores the space by **physically visiting** its areas,
3. builds an **occupancy map** (obstacle / free / unknown) and a **map of visited areas**,
4. chooses its next goals on its own and follows **collision-free paths**,
5. returns to the starting point and lands,
6. uses the saved map to **navigate to positions given by the user during flight**.


---

## Requirements

- Crazyflie 2.1 with the Multi-ranger deck and the Lighthouse deck (+ base stations) **or** the Flow deck v2
- Crazyradio (2.0 or PA)
- Python 3.8+

```bash
cd Project/lighthouse                # or Project/flowdeck
pip install -r requirements.txt      # cflib, numpy, matplotlib
```
> On Windows, if `pip` is not recognized: `py -m pip install -r requirements.txt`

## Configuration

All main settings are at the top of `common.py`:

```python
URI  = "radio://0/90/2M/E7E7E7E7A1"   # your drone's address
MODE = "lighthouse"                    # "lighthouse" or "flow"
```


---

## Running: step by step

The code was built incrementally:

```bash
cd Project/lighthouse        # or Project/flowdeck
py step1_sensors.py
```

| Step | File | What it does | Output |
|---|---|---|---|
| 1 | `step1_sensors.py` | Connects, checks the decks, prints position and distances live. **No flight.** | – |
| 2 | `step2_hover.py` | Pre-flight checks (battery ≥ 3.8 V, sensors), take-off, hover, landing | – |
| 3 | `step3_goto.py` | Flies to waypoints with real-time obstacle avoidance | – |
| 4 | `step4_map.py` | Mapping along a predefined route with 360° scans | `results/map.png`, `map.npz`, `flight_log.csv` |
| 5 | `step5_explore.py` | **Autonomous exploration**, return and landing | `results/explore_map.png`, `explore_map.npz`, `explore_log.csv` |
| 6 | `step6_navigate.py` | **Navigation** to positions you type (`x y`) while it flies. A new goal cancels the current one; an empty Enter ends the flight. | `results/navigation.png`, `navigation.csv` |

Helper tools (no flight):

| File | Use |
|---|---|
| `make_map.py` | Rebuilds the map from a flight log, e.g. `py make_map.py results/explore_log.csv` |
| `plot_map.py` | Redraws the image from a saved `.npz` |
| `flowdeck/step5_diagnostic.py` | Take-off only, printing the height continuously (to diagnose height problems with the Flow deck) |

**Stopping:** `Ctrl+C` always triggers a controlled landing.

> **Flow deck, step 6:** coordinates are measured from the take-off point. Place the drone **at the same spot and with the same heading** as during exploration, otherwise the map will not match the room.

---

## Architecture

```
common.py        connection, decks, sensors (log blocks), obstacle avoidance,
                 take-off / landing, safety checks
mapping.py       occupancy grid (log-odds) + visited grid
explore_core.py  safety zones, Dijkstra, A*, goal selection,
                 path smoothing, pure pursuit, map-based avoidance
mission.py       20 Hz control loop: explore() and go_to()
step1..6_*.py    the steps of the project
```

In every cycle (50 ms) the loop reads the position and distances, computes the velocity from the path, passes it through obstacle avoidance, sends it to the drone and updates the map.

### Algorithms

- **Mapping:** occupancy grid with 5 cm cells and log-odds updates (+0.9 for an obstacle, −0.4 for free space). Each ToF sensor is modelled as a cone with 3 rays. The map is only updated when the drone is at flight height, so the floor is not recorded as an obstacle.
- **Visited areas:** a separate grid. Coverage is measured on it, because the project requires physically visiting areas, not just observing them.
- **Exploration:** frontiers plus unvisited free cells. The score combines path cost, information gain and change of heading. Unreachable goals are blacklisted.
- **Path planning:** Dijkstra on a cost map with three zones (forbidden / caution / free) and line-of-sight smoothing. If a goal lies on an obstacle or cannot be reached, the drone goes to the **nearest safe point** instead. If a new obstacle appears on the path, the path is replanned.
- **Three layers of obstacle avoidance:** (1) safety zones during planning, (2) braking in front of mapped obstacles in 8 directions, (3) braking and repulsion from the live Multi-ranger measurements.

### Safety mechanisms
Maximum height 1 m · virtual fence around the starting point · detection of "frozen" sensors · detection of a hand above the drone (the check is disabled automatically when the Lighthouse deck covers the sensor) · height-sensor monitoring with the Flow deck · `Ctrl+C` → landing.

---

## Differences between the two folders

| Parameter | `lighthouse/` | `flowdeck/` |
|---|---|---|
| Forbidden zone (`LETHAL`) | 0.20 m | 0.25 m |
| Caution zone (`INFLATION`) | 0.27 m | 0.40 m |
| Zone cost (`INFLATION_COST`) | 2 | 6 |
| Minimum goal clearance (`TARGET_CLEARANCE`) | 0.15 m | 0.27 m |
| Exploration height / speed | 0.3 m / 0.3 m/s | 0.4 m / 0.25 m/s |
| Coverage criterion | 95% | 90% |
| Synchronized logging of position + distances | – | ✓ |
| Map-based avoidance, mapping only at flight height, `explore_log.csv` | – | ✓ |

With Lighthouse the position is accurate, so the zones are smaller and the drone fits through narrower passages. With the Flow deck the position comes from integrating velocity, so the error (drift) accumulates. That is why that version keeps larger margins, a lower speed and extra mechanisms.

---

## Results

Exploration with Lighthouse: **82 goals, ~92% coverage** of the mapped free space, with no collisions. The outer wall and the five interior obstacles are clearly visible.

<!-- To show the image here: upload it as Project/figures/lighthouse_explore_map.png
![Exploration with Lighthouse](figures/lighthouse_explore_map.png)
-->

In the map image: **black** = obstacle, **white** = free, **grey** = unknown, **green** = visited, **blue** = trajectory, **star** = starting point.

The full analysis (theory, equations, parameter tuning, problems, simulation, Lighthouse vs Flow deck comparison) is in the report.

---
