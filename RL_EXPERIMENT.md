## HOW TO experiments

Here is a (work in progress) list of step-by-step info to perform RL experiments. With little effort, this workflow can be applied to Model-based controllers and experiments.

 - log into the robot and launch the interface
 - Launch the controllers with

        ros2 launch locomotion_experiments controller_start.launch.py

 - Getup. This works from any configuration, it takes always 5 seconds. You **HAVE TO** terminate the node with `CTRL-C` once the 'index' max out to 700

       ros2 run locomotion_experiments getup

 - Launch the experiment. Look up at the launch arguments an the explanations in the [readme](README.md). Here is an example with the `cmd_vel_node`

       ros2 launch locomotion_experiments experiment.launch.py exp:='experiment_number_1' vel:='0.8' duration:='3.0' csv:='Data.csv'

 - Once the experiment is finished, the policy **WILL BE STILL ACTIVE** you have to turn it off with `CTRL-C` in the terminal of the launch above.
 - You can now run the `getup` node again, the robot will slowly go back in the default position.
 - Take a look at the temperatures before going again!
 - To change the reference shape take a look in `cmd_vel_node.py`
