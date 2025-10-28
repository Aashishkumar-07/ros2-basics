## ros2-basics

#### This repository contains learning &amp; implementation of various ROS2 (Robot Operating System 2) features such as:

- 📦 Packages – Creating and managing ROS2 packages
- 🤖 Nodes – Writing and launching publisher/subscriber nodes
- 🗣️ Topics – Exchanging data between nodes through publishers and subscribers
- 🧩 Services – Implementing client-server interactions
- ⚙️ Parameters – Configuring nodes dynamically
- 💾 Bags – Recording and replaying ROS2 topic data
- 🐢 Turtlesim – Simulating and controlling turtles for practice and visualization
- 🧰 RQT – Visualizing and debugging nodes using RQT tools


### Turtlesim project

#### Goal

This project uses the turtlesim package in ROS 2 to illustrate object-motion and target-tracking in a 2D plane. The default turtlesim_node provides a simple turtle in a GUI, and teleop_key_node captures keyboard input. My goal is to build custom nodes that allow the turtle to move autonomously and reach dynamically spawned targets at different locations. 

### **🧭 Logic Behind Moving Towards the Target in a 2D Plane**

The **distance** between two points in a 2D plane is calculated using the **Euclidean distance formula**, which is a generalized version of the **Pythagorean theorem**. This gives the *magnitude* of how far the turtle is from the target.
![image.png](attachment:b67edbd1-0574-46a8-b057-8d8c79553745:image.png)

To understand the concept of **euclidean** **distance**, you can refer to this short video:
👉 [Euclidean Distance Explained](https://youtu.be/3rPwfmrCwVw?si=Fj8WuYipXjXTlkV8)
---

However, the **direction** the turtle must move in depends on the **angle difference** between:

- the **angle to the target**
- the **turtle’s current heading**, and

This angular difference is calculated using the **`atan2`** function.

### **📐 Understanding `atan` and `atan2`**

#### 🔸 `atan` (Arc Tangent)

We know:

\tan \theta = \frac{\text{opposite side}}{\text{adjacent side}}

The tangent function is **periodic** every **π radians**, because:

\tan \theta = \frac{\sin \theta}{\cos \theta}

where sine and cosine repeat every 2π

At specific angles:

- **θ = 0:** sin(0) = 0, cos(0) = 1 → tan(0) = 0
- **θ = π/2:** sin(π/2) = 1 cos(π/2) = 0 → tan(π/2) → ∞ (undefined)

Between:

- **−π/2 and 0:** tan(θ) increases from −∞ to 0
- **0 and π/2:** tan(θ) increases from 0 to +∞

Hence, the interval **(−π/2, π/2)** gives a *unique* tan(θ) value for every θ. After that the values keep repeating for intervals  **(π/2, 3π/2), (3π/2, 5π/2).** The principal domain chosen was **(−π/2, π/2)**

**Domain:** (−π/2, π/2) 

**Range:** **(**−∞, +∞**)**

For inverse/arc tan, each domain value must give a unique value, so the principal domain  **(−π/2, π/2)**  was chosen.

**Domain:** (−∞, +∞) 

**Range:** **(**−π/2, π/2**)**

![image.png](attachment:e3c71f52-ac4e-45a9-bb84-baba47652f4f:image.png)

#### 🔸 The Limitation of `atan(y/x)`

`atan(y/x)` only gives you the **slope angle**. It doesn’t know which **quadrant** the point (x, y) lies in.

For example:

- (1, 1) → π/4
- (−1, −1) → π/4 ❌ (should be −3π/4)

 **`atan`**, has a range of **(−π/2, π/2)** for any given domain so it is covering **only the 1st & 4th quadrants** 

![ `atan` cannot distinguish between points diagonally opposite to each other.](attachment:c7da76c6-3d09-4815-b711-df7e59fc4e05:image.png)


#### 🔸 `atan2(y, x)` — the Quadrant-Aware Version ✅

`atan2(y, x)` solves this by considering both **x** and **y signs**, allowing it to determine the **true angle** from the origin to the point (x, y).

arc tan

- **Domain:** (−∞, +∞)
- **Range:** **(−π, π)** — covering *all 4 quadrants*
- *Here for each inverse/arc tan  domain (-pi,pi) there is a unique range*

Thus, `atan2` gives the **correct direction** of the target relative to the turtle’s current position — essential when moving a robot or turtle in a 2D plane.

**⚙️ Computing Velocities for Motion Control**

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

**🌀 Angular Adjustment**

- *steering angle* - Angle to the target turtle(*green turtle*) from current turtle(*blue* *turtle*).
`atan2`(y2-y1, x2-x1) = `atan2`(0.55,0.06) = 83.77 degree
- *self.parent_pose.theta* - turtle’s(*blue*) current heading angle  = 178 degree
- *angle_diff* = 83.77 - 178 = -94 degree  The turtle (*blue*) must rotate 94 degrees clockwise from its current facing angle to reach the target(*green*).

![image.png](attachment:e630347d-a6ea-4ddc-b94b-02def00e240a:image.png)

- *steering angle* - Angle to the target turtle(*green turtle*) from current turtle(*blue* *turtle*).
`atan2`(y2-y1, x2-x1) = `atan2`(10,0) = 90 degree
- *self.parent_pose.theta* - turtle’s(*blue*) current heading angle  = -178 degree
- *angle_diff* = 90 - (-178) = 268 degree  The turtle (*blue*) must rotate 268 degrees anti clockwise from its current facing angle to reach the target(*green*). It is taking the longer path rather than the shorter path of -82 degree (i.e moving 82 degree clockwise). To achieve this whenever the angle_diff becomes greater than 180 / -180 degree (pi /-pi radians) we adjust it

target : (-5,5) , source : (-5,-5)

![image.png](attachment:9cbc0ceb-f7c8-464f-a368-4db036500e5f:image.png)

The angular constant and linear constant are multiplied with angular velocity and linear velocity. These constants ensure that when the distance to cover is large or angle to move is large, these velocity remains high and when they reach closer to target the velocity remains slow ensuring what ?

### ⚙️ Velocity Control

Both **linear** and **angular velocities** are multiplied by constants:

- **`linear_vel = constant × euclidean_distance`**
- **`angular_vel = constant × angle_diff`**

These *constants* act as **gain factors**, ensuring smooth motion:

- When the **distance** or **angle** to the target is **large**, the velocity is **higher** — enabling faster movement.
- As the turtle gets **closer** to the target (distance or angle difference decreases), the velocity **slows down** — ensuring **smooth deceleration** and **precise alignment**, rather than overshooting or oscillating
