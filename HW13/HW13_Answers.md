1. Describe how an agent modifies their actions. What changes in the code will cause the agent to make "larger" or "smaller" changes?
    - a text document that explains how learning rates affect the ways that agents co-adapt.

    the lower the human's LR, the less they were able to run away from the robot. A larger LR made the human go further away, but it gets capped by the lmit set by np.clip in human actions

2. What happens if the two agents take turns improving their strategies? Does the behavior converge? What happens if they have different "rates" for updating their actions? Modify the code to explore these questions.

    The human seems to just run to the corner while the robot changes how close it can get based on its learning rate. The robot seems to be quite sensitive to the learning rate, as it's reseults over .2 are not inline with expected behavior of getting closer to the human.

3. What happens if they both adapt at the same time? As before, consider whether the team converges to a joint strategy, and how the interaction changes if they update at different rates. Modify the code to implement this simultaneous co-adaptation.
    Now, if they are the same, they move more coherently together. The best results was when both learning rates were .1, and there seems to deviations when values are set higher or lower. Ussually the human is able to have some distance on the robot if both learning rates are not .1.

4. Imagine that the robot knows how the human will adapt. Progam the robot so that it is aware of the human's learning rule. How can the robot leverage this information?
    - a text document that explains how agents can leverage their knowledge of adaptation to stay "one step ahead" of their opponents.

    The robot can use this information to plan its next move. If it already knows the humans next move, all it has to do is go there, so it makes chasing the human very easy. The only way the human can run, is if they just move more than what the robot is capapble of. 

6. the main.py file for your final version
7. png images of co-adaptation when the robot is aware of the human's learning rule - check anything name env3_(human learning rate)_(robot learning rate).png