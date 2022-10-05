---------------- joint_control.py ------------------------------

python code run method

1. open the terminal.
2. roslauch abbrobot abb_simulator.launch in terminal.
3. open the new terminal.
4. move in abbrobot/src in terminal.
5. python3 joint_control.py



Note!!
You have to change code in robot_hand.launch file. 


Python code control
<!-- send fake joint values -->
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"/>

<!-- Gui for joint contol --> 
<!-- <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui"/> -->


GUI control 
<!-- send fake joint values -->
<!-- <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"/> -->

<!-- Gui for joint contol --> 
<node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui"/>