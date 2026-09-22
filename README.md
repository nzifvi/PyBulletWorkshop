# PAL PyBullet Workshop
## Introduction
The agent is a model of a robot within an environment. The robot comes with the ability to:
* Sense the environment using 2 angled infrared sensors.
* Navigate through the environment using it's own locomotion and sensors.
  
Each level has a particular objective to achieve. Good luck!
## Agent
As discussed, the agent is a model of a robot. It comes with the following functions which can be used to perform various actions:
* getPose()
* setVelocity(bearing:float, mag:float)
* readSensors()

These are just functions that I have made for you which you can read about in the following section. Feel free to make your own if you know how!

<img width="622" height="650" alt="Untitled" src="https://github.com/user-attachments/assets/d331187f-8149-4cfe-975f-d7b7d92a3d57" />

The agent is equipped with 2 forward facing infrared sensors sensors: enabling the agent to measure the distances between itself and an object. In robotics, infrared sensors can be used for many tasks such as obstacle avoidance (with a bit of additional coding)

### Agent Pre-implemented Functions
#### getPose Function
the getPose function returns a tuple which consists of the (x, y, z) position and orientation of the agent in the environment. Some example usages are:
* Locate the agent in the environment to decide where to move next.
* Choose the next value to set the agent's velocity to based on the position (or potentially velocity if you want to do some calculus.)

#### setVelocity Function
The setVelocity function updates the velocity vector of the agent by taking in 2 parameters on call:
* bearing: the angle which you want the robot to turn to before movement.
* mag: the speed you want the robot to move forward at.

#### readSensors Function
The readSensors function is used to take measurements from both infrared sensors. The function returns a tuple in the following form:

$$\bigg(IR_{\text{left}}, IR_{\text{right}}\bigg)$$
## Level 1
### Objective
The objective of level 1 is to figure out how to use the infrared sensors (IR) to enable the robot to avoid collision and perform a full lap of the arena. Once you are done with level one, change the parameter of the Environment constructor to "level2" to load the next level.
### Hint
Consider all cases which may occur when a robot is moving: encountered wall on left, right, a wall which is equidistant to both sensors, or no wall at all. Using the measurements from the infrared sensors, you must devise a way to handle each case.
## Level 2
### Objective
The objective of level 2 is to navigate towards the green cube within the arena using methods from level 1 as well as new ones that you come up with. As task 2 is more complex, additional functions members of the Agent object have been created for you! These functions are:
* getBearing
* castRay
* seeObjective
* atObjective
  
To make things easier, the parameters of these functions have been assigned default values. However, if you want, you are able to override them as you see fit.
### Hint
Please note that there are multiple solutions to this problem. This solution is one that I came up with : )


The robot has 2 possible states whilst exploring the maze: explore and seek. If the agent cannot see the green cube (determined by the return values of seeObjective) then you know that the agent must keep exploring. If the agent can see the green cube then there is no need to continue exploring and it should navigate towards the green cube. Furthermore, using the solution to level1 that you devised, the agent already has a partial exploration algorithim. With a bit of modification, it can be turned into a proper navigation algorithim such as the left (or right) hand rule.
## Solutions
For solutions, please look at the text file with the name of the level you desire to see the solution of. Otherwise, ask a member of PAL in the lab : )
