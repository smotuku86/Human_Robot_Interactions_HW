import numpy as np
import json

# load a demonstration
# there are 10 demonstrations (numbered 1 - 10) saved to the demos folder
def get_demo(demo_number):
    demo_name = "demos/demo" + str(demo_number) + ".json"
    with open(demo_name, 'r') as f:
        data = json.load(f)
    return data["features"]

# score a demonstration for given parameters theta
def score_demo(demo_number, theta):
    theta = np.array(theta)
    features = get_demo(demo_number)
    score = 0.0
    # sum score for each recorded feature vector
    for feature in features:
        score += feature @ theta

    return score

# human model 
# gives you the chance of human choosing demo_number1 over demo_number2
def human_model(demo_number1, demo_number2, theta, beta=1.0):
    P_demo1 = boltsman_model(demo_number1, theta, beta)
    P_demo2 = boltsman_model(demo_number2, theta, beta)
    #print("P_demo1:", P_demo1)
    #print("P_demo2:", P_demo2)
    Human_Preferance = P_demo1 / (P_demo1 + P_demo2)
    return Human_Preferance
# Boltsman model
# Tells us the probability of human choosing a demonstration given theta
def boltsman_model(demo_number, theta, beta=1.0):
    #print(score_demo(demo_number, theta))
    P_demo = np.exp(beta * score_demo(demo_number, theta))
    return P_demo

def Metropolis_Hastings(demo_num, theta_guess, beta=1.0):
    theta_list = []
    for idx in range(1000):
        theta_list.append(theta_guess)
        #Find a nearby Theta - theta prime
        theta_prime = theta_guess + .5 * (np.random.rand(4) - .5)
        theta_prime = np.clip(theta_prime, -1.0, 1.0)

        #Comapare theta and theta prime using the boltsman model
        P_theta = boltsman_model(demo_num, theta, beta)
        P_theta_prime = boltsman_model(demo_num, theta_prime, beta)

        #if you want to compare mulitple demos, you can extend this part
        # P_theta = likelihood_function
        
        alpha = P_theta_prime / (P_theta + 1e-7) # avoid divide by zero
        if alpha > np.random.rand(): # noisy gradeint ascent step
            theta_guess = np.copy(theta_prime)

    print("theta list[last 10]:", np.array(theta_list[-10:]))
    return theta_guess

def likelihood_function(demos_liked, demos_disliked, theta, beta=1.0):
    P_demo_liked = []
    for idx in range(len(demos_liked)):
        P_demo_liked[idx] = human_model(demos_liked[idx],demos_disliked[idx], theta, beta)

    
    return likelihood
# example code to get a demo features and score them
theta = [-1.0, 0, 0, 1.0]
demo1 = get_demo(1)
score1 = score_demo(1, theta)
#print("demonstration features:\n", np.round(demo1, 3))
#print("score:", np.round(score1, 3))

demo4 = get_demo(4)
score4 = score_demo(4, theta)
human_pref = human_model(1, 4, theta)

theta_guess = [0, 0, 0, 0]
optimized_theta = Metropolis_Hastings(1, theta_guess, beta=1)
print("optimized theta:", optimized_theta)