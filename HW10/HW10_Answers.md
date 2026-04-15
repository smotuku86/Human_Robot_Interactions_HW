1. Identify the elements of the state vector. Explain why each element is relevant for the given task. Why does closing the gripper act like a trigger to change the robot's motion?

The state vector has the robot position, its gripper state, and the object location
 We need the robot postion so we know where the robot is
 the gripper state is useful for knowing if we are reaching ot the cylinder or if we have grabbed it and are moving it elsewhere
 the object location is usefull for us to know where it is and that it is moving with us once grabbed

2. Complete the model for the autoencoder. Train your resulting autoencoder, and use the decoder to control the robot's behavior in `test_policy.py`
 -trained with inclass model

3. Explore how user inputs affect the robot's behavior at run time. Try pressing `w`, `s`, and `x`. You can use `.` to terminate the simulation. Are we controlling this robot in end-effector space, or in joint space?

    It is hard to tell, but it seems to be closer to controlling joint space, moving faster or slower. It crashes into the table before doing anything useful and gets stuck. 

4. Modify the training parameters or dataset as neccesary to reach reasonable performance.


5. Imagine you wanted the user to be able to control whether the robot pushed the cylinder right or left. What changes would you need to make in the training data? How would these changes be reflected in the latent actions?


6. Modify the training data so that users can control whether the robot arm carries the cylinder right or left (or forward or back, your choice).