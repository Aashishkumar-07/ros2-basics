## ROS2 Basics
#### This repository contains learning &amp; implementation of various ROS2 (Robot Operating System 2) features such as:
- `📦 Packages` – Independent, reusable unit comprising multiple nodes (e.g., a camera package).
- `🤖 Nodes` – Individual subprograms, each performing a specific function (e.g., camera driver, image processor).
- `🗣️ Topics` – Nodes publish messages to a topic which are received by subscribers in real time through DDS’s publish–subscribe mechanism.
- `🧩 Services` – It follows a client-server model with a request & response message where only 1 server can exist per service but multiple clients can connect to it.
- `⚙️ Parameters` – named configuration values declared in code whose value are passed at node runtime.
- `💾 Bags` – useful for recording data from topics for any amount of time &amp; then replaying that data later
- `🐢 Turtlesim` – The “Hello World 🌍” of robotics used for simulating &amp; controlling turtles for learning &amp; visualization
- `🧰 RQT` – A GUI toolkit for visualizing and debugging ROS2 nodes.

<br>
<br>

## Turtlesim project
### 🎯 Goal
This project uses the turtlesim package in ROS 2 to demonstrate *object motion* and *target tracking* in a 2D plane. The default `turtlesim_node` provides a turtle in a GUI and `teleop_key_node` captures keyboard input. The goal is to build custom nodes that allow the turtle to move autonomously and reach dynamically spawned targets at different locations.

<br>

### 🧭 Logic behind moving towards the target in a 2D plane
### a. distance calculation
<p align="center">
  <img src="https://images.prismic.io/turing/65a53be67a5e8b1120d58808_image1_11zon_fa4497e473.webp?auto=format,compress" 
       alt="Euclidean distance calculation" 
       width="400">
</p>

The distance between two points in a 2D plane is calculated using the `Euclidean distance formula`, which is a generalized version of the `Pythagorean theorem`. This gives the magnitude of how far the turtle is from the target.

To understand the concept of euclidean distance, you can refer to this short video:
👉 [Euclidean Distance Explained](https://youtu.be/3rPwfmrCwVw?si=Fj8WuYipXjXTlkV8)

<br>

### b. angle calculation
The direction the turtle must move depends on the angle difference between:
- the angle to the target
- the turtle’s current heading
  
This angle to the target is calculated using the `atan2` function.

#### 📐 Understanding `atan` and `atan2`
#### 🔸 `atan` (Arc Tangent)
The tangent function is `periodic` after every `π radians` 
<p align="center">
     <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\tan\theta=\frac{\text{opposite}}{\text{adjacent}}" alt="tan theta formula"/>
     &nbsp;&nbsp;&nbsp;&nbsp;
     <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\tan\theta=\frac{\sin\theta}{\cos\theta}" alt="tan theta sine cosine"/>
</p>
Observation for angles in below range :
<p align="center">
   <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\tan(0)=\frac{\sin(0)}{\cos(0)}=\frac{0}{1}=0" alt="tan(0) formula" />
   &nbsp;&nbsp;&nbsp;&nbsp;
   <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\tan(90^\circ)=\frac{\sin(90^\circ)}{\cos(90^\circ)}=\frac{1}{0}=\text{undefined}" alt="tan(90) formula" />
</p>

<p align="center">
  <img src="https://github.com/Aashishkumar-07/ros2-basics/blob/main/assets/images/tan_graph.png?raw=true" 
       alt="Tangent Function Graph" width="500"/>
</p>

- `tan(−π/2->0)` : increases from −∞ to 0
- `tan(0->π/2)` :  increases from 0 to +∞
  
It is observed for the domain `(−π/2, π/2)` the range has a unique value for every θ. After that the values keep repeating for intervals  `(π/2, 3π/2)`, `(3π/2, 5π/2).` So the principal domain  `(−π/2, π/2)`  was chosen.

<div align="center">

| **Domain and Range for** | **tan(θ)** | **tan⁻¹(z)** |
|:--:|:--:|:--:|
| **Domain** | (−π/2, π/2) | (−∞, +∞) |
| **Range**  | (−∞, +∞) | (−π/2, π/2) |

</div>

<br>
<br>

>  **🤔 Why this domain?:**  
> Given that the tangent function is periodic with a period of π, what makes (−π/2, π/2) the preferred principal domain rather than (0, π) or another equivalent interval?

- The chosen domain has to be `continuous`, `monotonically increasing` with `one-to-one unique mapping`. This makes the above mentioned intervals valid.
- From the above valid intervals, **`(−π/2, π/2)`** was chosen because of its `symmetry about 0`, which gives it the property of an `odd function`:  
<p align="center">
  <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\arctan(-x)=-\arctan(x)" alt="arctan odd function property" />
</p>

- Because of this symmetry about the origin, evaluating the tangent function for positive values of θ is sufficient to infer its behavior for negative values as well.
- Calculations were simpler when functions had this property and analysis was more intuitive since many natural phenomena have symmetry (pendulums, springs, waves)
 
<p align="left">
  <img src="https://github.com/Aashishkumar-07/ros2-basics/blob/main/assets/images/domain_selection_rule.png?raw=true" 
       alt="Domain Selection Rule for Tangent Function" 
       width="600">
</p>


<br>

#### 🔸 The Limitation of `atan(y/x)`

<p align="center">
  <img src="https://github.com/Aashishkumar-07/ros2-basics/blob/main/assets/images/atan_values.png" 
       alt="atan_values"
       width="800"/>
</p>

`atan(y/x)`  only returns the `slope angle` and ignores which quadrant the point (x, y) lies in. Because its range is limited to (−π/2, π/2), it represents directions only in the 1st and 4th quadrants, making it insufficient when the actual target could lie in any of the four quadrants

<br>

#### 🔸 `atan2(y, x)` — the Quadrant-Aware Version ✅

`atan2(y, x)` solves this by considering both `x and y signs` ,  allowing it to determine the `true angle` from the origin to the target point (x, y).

**Domain and range for `tan⁻¹(z)`**

- Domain: (−∞, +∞)
- Range: (−π, π) — covering `all 4 quadrants`
  
Thus, `atan2` gives the `correct angular direction` of the target relative to the turtle’s current position which is essential when moving a robot in a 2D plane.

<br>

### c. computing velocities for motion control

```
def linear_vel(self, constant=1):
     return constant * self.euclidean_distance()

def steering_angle(self):
     return atan2(self.target_pose.y - self.parent_pose.y, self.target_pose.x - self.parent_pose.x)

def angular_vel(self, constant=2):
    angle_diff = self.steering_angle() - self.parent_pose.theta
    while angle_diff > pi:
        angle_diff -= 2*pi
    while angle_diff < -pi:
        angle_diff += 2*pi
    angular_vel = constant * angle_diff
    return angular_vel
```

**🌀 Angular adjustment**

<p align="center">
  <img src="https://github.com/Aashishkumar-07/ros2-basics/blob/main/assets/images/path_computation_to_target.png" 
       alt="atan_values"
       width="500"/>
</p>

```atan2(y2-y1, x2-x1) = atan2(0.55,0.06) = 83.77 degree```

- *steering angle*: angle to the `target turtle(green)` from `current turtle(blue)` 
- *self.parent_pose.theta*: current heading angle of `turtle(blue)` = 178 degree
- *angle_diff*: `83.77 - 178 = -94 degree`. The `current turtle(blue)` must rotate `94 degrees clockwise` from its current facing angle to reach the `target turtle(green)`.

<br>

<p align="center">
  <img src="https://raw.githubusercontent.com/Aashishkumar-07/ros2-basics/main/assets/images/longer_shorter_path_to_target.png" 
       alt="atan_values"
       width="500">
  <br>
  <em>Source: (-5, -5)  Target: (-5, 5)</em>
</p>


`atan2(y2-y1, x2-x1) = atan2(10,0) = 90 degree`

- *steering angle*: angle to the `target turtle(green)` from `current turtle(yellow)`
- *self.parent_pose.theta*: current heading angle of `turtle(yellow)`  = -178 degree
- *angle_diff*: `90 - (-178) = 268 degree`  The `current turtle(yellow)` must rotate `268 degrees anti clockwise` from its current facing angle to reach the `target turtle(green)`. It is taking the `longer path rather than the shorter path` of -82 degree (i.e moving `82 degree clockwise`). To achieve this whenever the `angle_diff becomes greater than 180 / -180 degree (pi /-pi radians) we adjust it`

<br>

### d. velocity control

Both `linear` &amp; `angular velocities` are multiplied by constants:

- **`linear_vel = constant × euclidean_distance`**
- **`angular_vel = constant × angle_diff`**

These *constants* act as `gain factors`, ensuring smooth motion:

- When the `distance` or `angle` to the target is `large`, the velocity is `higher` enabling faster movement.
- As the turtle gets `closer` to the target (distance or angle difference decreases), the velocity `slows down` — ensuring `smooth deceleration` and `precise alignment`, rather than overshooting or oscillating

<br>

### 📡 Node communication diagram 

<p align="center">
  <img src="https://github.com/Aashishkumar-07/ros2-basics/blob/main/assets/images/turtlesim_node_architecture.png" 
       alt="atan_values"
  >
  <br>
</p>
