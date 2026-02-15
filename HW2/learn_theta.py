from random import random
import numpy as np
import json
from setup_world import get_score

# load a demonstration
# there are 10 demonstrations (numbered 1 - 10) saved to the demos folder
def get_demo(demo_number):
    demo_name = "demos/collect_demos/demos/demo" + str(demo_number) + ".json"
    with open(demo_name, 'r') as f:
        data = json.load(f)
    return data["features"]

# score a demonstration for given parameters theta
def score_demo(demo_number, theta):
    theta = np.array(theta)
    features = get_demo(demo_number)
    score = get_score(features[-1], theta)
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

# Q1
# Boltsman model
# Tells us the probability of human choosing a demonstration given theta
def boltsman_model(demo_number, theta, beta=1.0):
    #print(score_demo(demo_number, theta))
    P_demo = np.exp(beta * score_demo(demo_number, theta))
    return P_demo

def Metropolis_Hastings(demo_pairs, theta_guess, beta=1.0):
    theta_list = []
    for idx in range(100):
        theta_list.append(theta_guess)
        #Find a nearby Theta - theta prime
        theta_prime = theta_guess + .2 * (np.random.rand(3) - .5)
        theta_prime = np.clip(theta_prime, -1.0, 1.0)

        #Comapare theta and theta prime using the boltsman model
        P_theta = likelihood_function(demo_pairs, theta_guess, beta)
        P_theta_prime = likelihood_function(demo_pairs, theta_prime, beta)
        
        alpha = P_theta_prime / (P_theta + 1e-7) # avoid divide by zero
        if alpha > np.random.rand(): # noisy gradeint ascent step
            theta_guess = np.copy(theta_prime)

    print("theta list[last 10]:", np.array(theta_list[-10:]))
    return theta_guess

def likelihood_function(demo_pairs, theta, beta=1.0):
    P_demo_liked = []
    for demo1, demo2 in demo_pairs:
        human_pref = human_model(demo1, demo2, theta, 1) #hardcode 1 to choose besdt demo - always

        if human_pref == 1: # demo 1 is prefered
            P_demo_liked.append(boltsman_model(demo1, theta, beta))
        else:
            P_demo_liked.append(boltsman_model(demo2, theta, beta))

    likelihood = np.prod(P_demo_liked) #multiply all our chances together 
    
    return likelihood


#score all demos with a theta
theta = [.5,0,1]
for demo_num in range(1, 11): #10 demos
    score = score_demo(demo_num, theta)
    print("score for demo", demo_num, "is:", np.round(score, 3))

#Q2 - found .9 to be good
human_pref = human_model(1, 5, theta, beta=.9)
print("human preferance for demo 1 over demo 5 chance:", np.round(human_pref, 3))
theta_guess = [0, 0, 0]
optimized_theta = Metropolis_Hastings([(1,5)], theta_guess, beta=.9)
print("optimized theta:", optimized_theta)

#now randomly sample pairs of demos (1-10) and get a theta from Metropolis Hastings

pairs = [] # keep track of what we compare
preferd_demos = [] # keep track of what the human prefers in each pair
no_likey_demos = [] # keep track of what the human does not prefer in each pair
for pair in range(10): #get 10 pairs
    demo1 = np.random.randint(1,11)
    demo2 = np.random.randint(1,11)
    while demo1 == demo2:
        demo2 = np.random.randint(1,11)
    pairs.append((demo1, demo2))

    human_pref = human_model(demo1, demo2, theta, beta=1) # choose highest score - always
    if human_pref == 1: # demo 1 is prefered
        preferd_demos.append(demo1)
        no_likey_demos.append(demo2)
    else:
        preferd_demos.append(demo2)
        no_likey_demos.append(demo1)

print ("pairs compared:", pairs)
print("prefered demos:", preferd_demos)

# ok, well I put the above in a function and now we will run Metropolis Hastings on the pairs we randomly sampled
optimized_theta = Metropolis_Hastings(pairs, theta_guess, beta=.9)
print("optimized theta:", optimized_theta)
#I got - optimized theta: [0.99049298 0.14766348 0.31938759]
#                         [1, 0.46424706, 0.0973374]
#     ran this 5x         [ 0.94865027  0.88740711 -0.39036824]
#                         [1.         0.27080109 0.16573933]
#                         [ 1.         -0.07991503  0.47366972]

# Q5 - put it in main.py and it worked as expected - the score increased as I grouped the cubes together more and stacked them. But, the theta I gave it is different from the one I got from Metropolis Hastings.
# Maybe it was due to teh demos themselves, I had only 2 where I stacked blocks, and I think I needed more examples of stacking for the algorithm to learn that feature and give it more importance.
#  

#Chat Log - https://chatgpt.com/c/6990d0c9-8818-832c-825e-c12add10cf7e 


learned_thetas = np.array([[0.99049298, 0.14766348, 0.31938759],
                           [1, 0.46424706, 0.0973374],
                           [ 0.94865027,  0.88740711, -0.39036824],
                           [1.         , 0.27080109, 0.16573933],
                           [ 1.         , -0.07991503, 0.47366972]])

Avg_theta = np.mean(learned_thetas, axis=0)
print("Chosen Theta:", theta)
print("Average Theta from Metropolis Hastings:", Avg_theta)
#get the error of each feature compared to the chosne theta of [.5, 0, 1]
Error =  learned_thetas - np.array(theta)
AvgError = np.mean(np.abs(Error), axis=0)
print("Average error of the learned theta compared to the chosen theta:", AvgError)