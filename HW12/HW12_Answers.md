 What is the code doing
Collection - 
     happens in main.py
    main.py 
        dynamics - moves the robot based on its action
        get_action - figures out what to do to go to the goal based on current location
        add2dataset - adds SA pairs to dataset
        plot_SA - in the name plots the SA pair
        get_dataset - combines all the above to output a dataset we can use to train - our goal is to use less than 20 examples and get within 1 unit radius of goal
    
    models - file with model params 

    test_policy - 
        does x num of tests where the goal gets randomized and then the robot has 20 tries to get as close to the goal as possible
        record error at end

    train_policy -
        load dataset and model, runs data through the model and optimizes for a certain amount of epochs at a .001 LR.

 
    -    I think the most intuitive way to start, given the constraint, is to add more data to the demonstrations. This allows the model to gain more context about what it should be trying to do. So, I let the robot observe more state-action (SA) pairs in each demo, since that has worked well in previous assignments. Looking at the results, that was exactly the case. The default program provided one SA pair per demo, and it performed poorly in guiding the robot. When we increased this to 10 SA pairs per demo, the robot performed much better, getting within 1 unit of the goal.

    One strategy other I tried was constructing the training set so that the robot single-stepped directly to the goal (SingleStraightShot). This essentially made each training demo consist of the robot moving straight to the goal. Since the dynamics of the testing robot were not significantly different, I expected the model to continue outputting actions directed toward the goal and eventually reach it. After all, the actions are normalized (i.e., their magnitude is scaled down). However, the results were quite surprising—the path taken by the trained model was curved rather than straight. After observing this, I figured that it was not an effective strategy and did not attempt to refine it further.

    From an information-gathering perspective, this demonstrates that having more data is extremely beneficial for the robot’s learning process. In more complex tasks, maintaining a rolling history can significantly improve performance without necessarily increasing the amount of training data. This is similar to how humans approach tasks; we rely on past context to avoid repeating mistakes.

Results:
strategy - #demos - #SA pairs #avg error
default - 20 demos - 20 - 7.67
AddInfo - 20 demos - 100 - 2.01
AddInfo - 20 demos - 200 - .4
AddInfo - 20 demos - 400 - .1
SingleStraightShot - 20 demos - 20 - 3.45

check these pics for best results SS
HW12/visualizations/state_action2020.png
HW12/visualizations/xi2020.png