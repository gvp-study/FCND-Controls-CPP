

# FCND Drone Control in 3D #

This is the readme for the C++ project.

For easy navigation throughout this document, here is an outline:

 - [Simulator walkthrough](#simulator-walkthrough)
 - [The tasks](#the-tasks)
 - [Evaluation](#evaluation)



## Simulator Walkthrough ##

Now that you have all the code on your computer and the simulator running, let's walk through some of the elements of the code and the simulator itself.

### The Code ###

For the project, the majority of your code will be written in `src/QuadControl.cpp`.  This file contains all of the code for the controller that you will be developing.

All the configuration files for your controller and the vehicle are in the `config` directory.  For example, for all your control gains and other desired tuning parameters, there is a config file called `QuadControlParams.txt` set up for you.  An import note is that while the simulator is running, you can edit this file in real time and see the affects your changes have on the quad!

The syntax of the config files is as follows:

 - `[Quad]` begins a parameter namespace.  Any variable written afterwards becomes `Quad.<variablename>` in the source code.
 - If not in a namespace, you can also write `Quad.<variablename>` directly.
 - `[Quad1 : Quad]` means that the `Quad1` namespace is created with a copy of all the variables of `Quad`.  You can then overwrite those variables by specifying new values (e.g. `Quad1.Mass` to override the copied `Quad.Mass`).  This is convenient for having default values.

You will also be using the simulator to fly some difference trajectories to test out the performance of your C++ implementation of your controller. These trajectories, along with supporting code, are found in the `traj` directory of the repo.


### The Simulator ###

In the simulator window itself, you can right click the window to select between a set of different scenarios that are designed to test the different parts of your controller.

The simulation (including visualization) is implemented in a single thread.  This is so that you can safely breakpoint code at any point and debug, without affecting any part of the simulation.

Due to deterministic timing and careful control over how the pseudo-random number generators are initialized and used, the simulation should be exactly repeatable. This means that any simulation with the same configuration should be exactly identical when run repeatedly or on different machines.

Vehicles are created and graphs are reset whenever a scenario is loaded. When a scenario is reset (due to an end condition such as time or user pressing the ‘R’ key), the config files are all re-read and state of the simulation/vehicles/graphs is reset -- however the number/name of vehicles and displayed graphs are left untouched.

When the simulation is running, you can use the arrow keys on your keyboard to impact forces on your drone to see how your controller reacts to outside forces being applied.

#### Keyboard / Mouse Controls ####

There are a handful of keyboard / mouse commands to help with the simulator itself, including applying external forces on your drone to see how your controllers reacts!

 - Left drag - rotate
 - X + left drag - pan
 - Z + left drag - zoom
 - arrow keys - apply external force
 - C - clear all graphs
 - R - reset simulation
 - Space - pause simulation




### Scenario_1 ###

When you run the simulator, you'll notice your quad is falling straight down.  This is due to the fact that the thrusts are simply being set to:

```
QuadControlParams.Mass * 9.81 / 4
```

Changing the `Mass` parameter in `QuadControlParams.txt` to 0.5 kg made the vehicle more or less stay in the same spot. The simulator run looks like this:

<p align="center">
<img src="docs/Scenario_1.mov" width="500"/>
</p>



## The Tasks ##

For this project, you will be building a controller in C++.  You will be implementing and tuning this controller in several steps.

You may find it helpful to consult the [Python controller code](https://github.com/udacity/FCND-Controls/blob/solution/controller.py) as a reference when you build out this controller in C++.

#### Notes on Parameter Tuning
1. **Comparison to Python**: Note that the vehicle you'll be controlling in this portion of the project has different parameters than the vehicle that's controlled by the Python code linked to above. **The tuning parameters that work for the Python controller will not work for this controller**

2. **Parameter Ranges**: You can find the vehicle's control parameters in a file called `QuadControlParams.txt`. The default values for these parameters are all too small by a factor of somewhere between about 2X and 4X. So if a parameter has a starting value of 12, it will likely have a value somewhere between 24 and 48 once it's properly tuned.

3. **Parameter Ratios**: In this [one-page document](https://www.overleaf.com/read/bgrkghpggnyc#/61023787/) you can find a derivation of the ratio of velocity proportional gain to position proportional gain for a critically damped double integrator system. The ratio of `kpV / kpP` should be 4.

### Body rate and roll/pitch control (scenario 2) ###

First, you will implement the body rate and roll / pitch control.  For the simulation, you will use `Scenario 2`.  In this scenario, you will see a quad above the origin.  It is created with a small initial rotation speed about its roll axis.  Your controller will need to stabilize the rotational motion and bring the vehicle back to level attitude.

I converted the python code in  the [Python controller code](https://github.com/udacity/FCND-Controls/blob/solution/controller.py)   specifically these functions and setting these Kpqr parameters.

1. Implement body rate control

 - implement the code in the function `GenerateMotorCommands()`
 - implement the code in the function `BodyRateControl()`
 - Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot

```
QkpPQR = 75, 75, 7
```

```c++
VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float l = L / sqrt(2.f);
  float F = collThrustCmd;
  float tx = momentCmd.x / l;
  float ty = momentCmd.y / l;
  float tz = momentCmd.z / kappa;

  float F0 = (F + tx + ty - tz) / 4.f; // front left
  float F1 = (F - tx + ty + tz) / 4.f; // front right
  float F2 = (F + tx - ty + tz) / 4.f; // rear left
  float F3 = (F - tx - ty - tz) / 4.f; // rear right

  cmd.desiredThrustsN[0] = CONSTRAIN(F0, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[1] = CONSTRAIN(F1, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[2] = CONSTRAIN(F2, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[3] = CONSTRAIN(F3, minMotorThrust, maxMotorThrust);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

```

```c++

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)
 
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F M = V3F(Ixx, Iyy, Izz);
  V3F pqrErr = pqrCmd - pqr;
  V3F pqrU_bar = kpPQR * pqrErr;
  V3F momentCmd = M * pqrU_bar;
 
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}
```

I then Implemented the  roll / pitch control as shown below. I experimented with the setting `QuadControlParams.kpBank` by changing it from 0.0 to 15 in steps of 5 to figure out a reasonable settling time on the roll of 0.240 seconds.

```c++
// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd(0.0, 0.0, 0.0);
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float cmdByM = collThrustCmd / mass;

  if ( collThrustCmd > 0 ) {  
    float b_x = R(0,2);
    float b_x_c = -CONSTRAIN(accelCmd.x / cmdByM, -maxTiltAngle, maxTiltAngle);
    float b_x_err = b_x_c - b_x;
    float b_x_p_term = kpBank * b_x_err;

    float b_y = R(1,2);
    float b_y_c = -CONSTRAIN(accelCmd.y / cmdByM, -maxTiltAngle, maxTiltAngle);
    float b_y_err = b_y_c - b_y; 
    float b_y_p_term = kpBank * b_y_err;

    float p_c = (R(1,0) * b_x_p_term - R(0,0) * b_y_p_term) / R(2,2);
    float q_c = (R(1,1) * b_x_p_term - R(0,1) * b_y_p_term) / R(2,2);

    pqrCmd.x = p_c;
    pqrCmd.y = q_c;
    pqrCmd.z = 0.0;
  }
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

```

The result of the code and the parameter tuning is shown below.

<p align="center">
<img src="docs/Scenario_2.mov" width="500"/>
</p>



### Position/velocity and yaw angle control (scenario 3) ###

For Scenario_3, I implemented teh LateralPositionControl and AltitudeControl as shown below.

```c++
// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F pos_err = posCmd - pos;
  V3F vel_cmd = kpPosXY * pos_err;
  
  float vel_mag = vel_cmd.mag();
  if(vel_mag > maxSpeedXY)
    vel_cmd = vel_cmd * (maxSpeedXY / vel_mag);
  
  V3F vel_err = vel_cmd - vel;
  V3F acc_cmd = accelCmdFF + kpVelXY * vel_err;
  
  float acc_mag = acc_cmd.mag();
  if(acc_mag > maxAccelXY)
    acc_cmd = acc_cmd * (maxAccelXY / acc_mag);
  
  accelCmd = acc_cmd;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

```



```c++
float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float z_err = posZCmd - posZ;
  float z_err_dot = velZCmd - velZ;
  float b_z = R(2,2);

  float p_term = kpPosZ * z_err;
  float d_term = kpVelZ * z_err_dot;
  float i_term = KiPosZ * z_err * dt;
  
  float u_1_bar = p_term + d_term + i_term + accelZCmd;
  
  float z_acc = (u_1_bar - CONST_GRAVITY) / b_z;
  thrust = -mass * z_acc;
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}
```

I experimented with the parameters kpPosXY, KpPosZ, kpVelXY, kpVelZ till I found these values by trial and error.

```json
# Position control gains
kpPosXY = 4
kpPosZ = 4
KiPosZ = 80

# Velocity control gains
kpVelXY = 16
kpVelZ = 16

# Angle control gains
kpBank = 15
kpYaw = 4
```



The video below shows the first quad1 starting from 45 degree away from the target yaw and reach the target yaw angle gently. The gains also do not affect quad2 which starts with the target yaw angle.

<p align="center">
<img src="docs/Scenario_3.mov" width="500"/>
</p>

### Non-idealities and robustness (scenario 4) ###

In this part, we will explore some of the non-idealities and robustness of a controller.  For this simulation, we will use `Scenario 4`.  This is a configuration with 3 quads that are all are trying to move one meter forward.  However, this time, these quads are all a bit different:
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual

1. Edited `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.

   ```c++
   float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
   {
     // Calculate desired quad thrust based on altitude setpoint, actual altitude,
     //   vertical velocity setpoint, actual vertical velocity, and a vertical 
     //   acceleration feed-forward command
     // INPUTS: 
     //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
     //   posZ, velZ: current vertical position and velocity in NED [m]
     //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
     //   dt: the time step of the measurements [seconds]
     // OUTPUT:
     //   return a collective thrust command in [N]
   
     // HINTS: 
     //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
     //  - you'll need the gain parameters kpPosZ and kpVelZ
     //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
     //  - make sure to return a force, not an acceleration
     //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER
   
     Mat3x3F R = attitude.RotationMatrix_IwrtB();
     float thrust = 0;
   
     ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
   
     float z_err = posZCmd - posZ;
     float z_term = kpPosZ * z_err;
   
     integratedAltitudeError += z_err * dt;
     float i_term = KiPosZ * integratedAltitudeError;
   
     float z_dot_cmd = z_term + velZCmd;
     z_dot_cmd = CONSTRAIN(z_dot_cmd, -maxAscentRate, maxDescentRate);
   
     float z_dot_err = z_dot_cmd - velZ;
     float z_dot_term = kpVelZ * z_dot_err;
     float z_dot_dot_cmd = z_dot_term + i_term +  accelZCmd;
   
     thrust = -(z_dot_dot_cmd - CONST_GRAVITY) * mass / R(2,2);
   
     /////////////////////////////// END STUDENT CODE ////////////////////////////
     
     return thrust;
   }
   ```

   

2. Changed the the kpPosXY = 3, kpPosZ = 8 and KiPosZ = 2 to make it work for all three cases as shown in the video below.

<p align="center">
<img src="docs/Scenario_4.mov" width="500"/>
</p>

### Tracking trajectories ###

Now that we have all the working parts of a controller, you will put it all together and test it's performance once again on a trajectory.  For this simulation, you will use `Scenario 5`.  This scenario has two quadcopters:
 - the orange one is following `traj/FigureEight.txt`
 - the other one is following `traj/FigureEightFF.txt` - for now this is the same trajectory.  For those interested in seeing how you might be able to improve the performance of your drone by adjusting how the trajectory is defined, check out **Extra Challenge 1** below!

How well is your drone able to follow the trajectory?  It is able to hold to the path fairly well?


### Extra Challenge 1 (Optional) ###

You will notice that initially these two trajectories are the same. Let's work on improving some performance of the trajectory itself.

1. Inspect the python script `traj/MakePeriodicTrajectory.py`.  Can you figure out a way to generate a trajectory that has velocity (not just position) information?

2. Generate a new `FigureEightFF.txt` that has velocity terms
Did the velocity-specified trajectory make a difference? Why?

With the two different trajectories, your drones' motions should look like this:

<p align="center">
<img src="animations/scenario5.gif" width="500"/>
</p>


### Extra Challenge 2 (Optional) ###

For flying a trajectory, is there a way to provide even more information for even better tracking?

How about trying to fly this trajectory as quickly as possible (but within following threshold)!


## Evaluation ##

To assist with tuning of your controller, the simulator contains real time performance evaluation.  We have defined a set of performance metrics for each of the scenarios that your controllers must meet for a successful submission.

There are two ways to view the output of the evaluation:

 - in the command line, at the end of each simulation loop, a **PASS** or a **FAIL** for each metric being evaluated in that simulation
 - on the plots, once your quad meets the metrics, you will see a green box appear on the plot notifying you of a **PASS**


### Performance Metrics ###

The specific performance metrics are as follows:

 - scenario 2
   - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
   - roll rate should less than 2.5 radian/sec for 0.75 seconds

 - scenario 3
   - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
   - Quad2 yaw should be within 0.1 of the target for at least 1 second


 - scenario 4
   - position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds

 - scenario 5
   - position error of the quad should be less than 0.25 meters for at least 3 seconds

## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.