# Locomotion Experiment Package

## What's inside:

### 1. The 'getup' node
From *any* configuration of the robot, you can call 

    ros2 run locomotion_experiments getup
  
and the node will interpolate linearly in 5 seconds from the current position to the default 'stand' position.

### 2. The experiment launch
This is a general-purpose launch file for locomotion experiments. It's tailored for RL (it launches the Inference controller) but with little effort can be used for any type of locomotion experiment.

Here is the syntax:

        ros2 launch locomotion_experiments experiment.launch.py exp:='experiment_number_1' vel:='0.8' duration:='3.0' use_joy:='False' csv:='Data.csv'

Arguments:
 - `exp`: the experiment name. This will be used to save the bag file and to save the informations in the csv
 - `use_joy`: if true, the launch will start the joystick node and the cmd_vel_node will not start
 - `csv`: the name of the csv file where the data will be saved. If the file already exists, the data will be appended to the file. If the file does not exist, it will be created.
 - `vel` and `duration` are the parameters of the cmd_vel_node. They are the velocity and the duration of the movement. Depending on the configuration (you can switch them in the code) the cmd_vel_node can output:
   - A constant velocity `vel` for a fixed duration `duration`
   - A linearly increasing velocity from 0 to `vel` in `duration` seconds
   - A "trapedoidal" wave with a peak velocity of `vel` and a total duration of `duration` with a linear ramp-up and ramp-down of 10% of `duration` each.
   - Any custom function you want to implement, you can use the "elapsed time" `te` inside the code.

When executed, the launch will record a bag file with the name specified in the `exp` argument. 
> **PLEASE NOTE: If a bag already exists with that name the launch will throw an error and will shut down.** 
The launch will also save all the arguments and the timestamp inside the csv file specified in the `csv` argument. This is often helpful to quickly associate the bag file with the experiment. At the moment, the csv file is created with space-separated values and will not contain headers. The syntax is:

    exp_name, use_joy, vel, duration



## TODO:
- [ ] Add headers to the csv file
- [ ] Add timestamp to bag name
- [ ] Add "video number" to the csv file

