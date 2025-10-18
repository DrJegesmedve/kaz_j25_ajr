# `kaz_j25_ajr` package

ROS 2 C++ package. [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)

A package két node-ból áll:

* A `/world_node` egy **Qt-alapú szimulációs környezetet** valósít meg, amely megjeleníti egy robotporszívó mozgását, a falakat, a koszt és a takarítás folyamatát.
* A `/vacuum_node` egy **autonóm porszívó logikát** valósít meg, amely a ROS-topicokon keresztül kap információt a világról, és ennek alapján vezérli saját mozgását.

A robot 10×10 m-es szobában mozog, port keres és felszívja azt. A világ szimulálja az érzékelést, a mozgást, és minden komponens ROS 2 Humble alatt futtatható.

---

## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages

```bash
cd ~/ros2_ws/src
```
```bash
git clone https://github.com/DrJegesmedve/kaz_j25_ajr
```

### Build ROS 2 packages

```bash
cd ~/ros2_ws
```
```bash
colcon build --packages-select kaz_j25_ajr --symlink-install
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

```bash
source ~/ros2_ws/install/setup.bash
```

</details>

---

## Run

### Launch both nodes

```bash
ros2 launch kaz_j25_ajr vacuum_world.launch.py
```

### Or run manually

```bash
ros2 run kaz_j25_ajr world_node
```

```bash
ros2 run kaz_j25_ajr vacuum_node
```

---

## Graph

```mermaid
graph LR;

subgraph World
world([ /world_node]):::red
end

subgraph Vacuum
vac([ /vacuum_node]):::red
end

vac --> cmd[ /cmd_vel<br/>geometry_msgs/Twist]:::light
cmd --> world

world --> pose[ /pose<br/>geometry_msgs/Pose2D]:::light
pose --> vac

world --> target[ /target_dust<br/>geometry_msgs/Point]:::light
target --> vac

world --> wall[ /wall<br/>std_msgs/Bool]:::light
wall --> vac

world --> done[ /done<br/>std_msgs/Bool]:::light
done --> vac

classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#152742
classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff
```

---

## Description

* **`world_node`**

  * Qt-based GUI simulation (10×10 m map)
  * Randomly generated 30 dust spots
  * Publishes robot pose, wall proximity, dust targets and completion status
  * Visualizes sensing radius (green), cleaned areas (purple), sensed cells (yellow), and remaining dust (gray)

* **`vacuum_node`**

  * Receives robot position and dust targets
  * Publishes movement commands (`/cmd_vel`)
  * Avoids walls and navigates towards dust
  * Moves randomly when no dust is known

---

## Example visualization

![](img/image.png)
