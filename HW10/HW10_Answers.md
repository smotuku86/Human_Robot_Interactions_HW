1. Identify the elements of the state vector. Explain why each element is relevant for the given task. Why does closing the gripper act like a trigger to change the robot's motion?

The state vector includes the robot’s position, the gripper state, and the object’s location. We need the robot’s position to know where the robot is. The gripper state is useful for determining whether we are still reaching for the cylinder or have already grabbed it and are moving it elsewhere. The object’s location is useful for tracking where it is and confirming that it moves with the robot once it has been grasped.

2. Complete the model for the autoencoder. Train your resulting autoencoder, and use the decoder to control the robot's behavior in `test_policy.py`
 -trained with inclass model

3. Explore how user inputs affect the robot's behavior at run time. Try pressing `w`, `s`, and `x`. You can use `.` to terminate the simulation. Are we controlling this robot in end-effector space, or in joint space?

It is hard to tell, but it seems closer to controlling the robot in joint space (e.g., moving faster or slower). It often crashes into the table before doing anything useful and then gets stuck. One more thing I noticed is that w and s move it in opposite directions, and opening or closing the gripper seemed to switch the w/s behavior around.

4. Modify the training parameters or dataset as neccesary to reach reasonable performance.

    I could not find a set of parameters or datasets that had reasonable preformace  

5. Imagine you wanted the user to be able to control whether the robot pushed the cylinder right or left. What changes would you need to make in the training data? How would these changes be reflected in the latent actions?

    We would have to add those actions to the dataset and the latent actions would be our keyboard commands. 

6. Modify the training data so that users can control whether the robot arm carries the cylinder right or left (or forward or back, your choice).

    I wasn't able to grab the cylinder, but the video "BestAttempt" shows how pressing the keys resulted in the robot moving different directions. Now, it was not in the world's xy plane, but it did might've followed some sort of axis about the robot. Basically, there were discont actions, but i was hard to gauge what movements would result in. 


Data Results:

1000_100_0.001_128_1_weights - a mess, random failing around
10000_200_0.001_128_1_weights - straightened up and W/S seemed to control the first joint
10000_200_0.001_128_2_weights - increased latent dim -> 2 and it got closer but still failed
10000_200_0.01_128_2_weights - increased LR to see if that would help it explore, still no good

50 dataset examples
1000_10_0.0001_128_2_dataset_50_weights - no good
1000_100_0.001_128_2_dataset_50_weights  - no good
10000_200_0.0001_128_2_dataset_50_weights - no good

HW10/model weights/1000_100_0.001_128_5_dataset_50_weights - got close but then spun off
HW10/model weights/1000_100_0.001_128_5_dataset_50L_weights - got close but then spun off - this one had 49800 SA pairs too!

Move forward, backward, left, or right 
1000_100_0.001_128_6_dataset_50L_movingXY_weights - showed some responce to wasd command
1000_500_0.001_128_3_dataset_50L_movingXY_weights - lowered hiddendim to see how it would perform without being given cylinder location - not well
1000_100_0.001_128_4_dataset_50L_movingXY_weights - swung around and hit tables - added in displacement to cylinder as a latent dim
1000_1000_0.001_1024_3_dataset_50L_movingXY_weights - random try on gpu very slow and unresponsive
1000_200_0.001_64_3_dataset_50L_movingXY_weights - also really slow??

