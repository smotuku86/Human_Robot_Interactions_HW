"""
Multi-armed bandits starter code

"""

import numpy as np
import matplotlib as mpl

class MAB:

     # initialization
    def __init__(self, theta):

        # probability for each arm        
        self.theta = theta
        
    # pull the a-th arm
    def draw(self, a):

        # get reward 1 with probability theta[a]
        reward = np.random.binomial(1, self.theta[a])

        # expected regret of pulling arm a
        regret = np.max(self.theta) - self.theta[a]

        return reward, regret

def epsilon_greedy(rewards, epsilon):

    if np.random.rand() < epsilon:
        a = np.random.choice(len(rewards))
    # otherwise pull the best performing arm
    else:
        a = np.argmax(rewards)
    return a


def main():

    # setup the bandit
    theta = [0.5, 0.55, 0.4, 0.2]
    bandit = MAB(theta)
    n_interaction = 100

    # initialize
    times_pulled = [0] * len(theta)
    total_reward = [0] * len(theta)
    avg_reward = [0] * len(theta)

    for idx in range(n_interaction):

        # use chosen strategy to select arm
        a = epsilon_greedy(avg_reward, 0.2)
        reward, regret = bandit.draw(a)
        # update the average reward for the chosen arm
        times_pulled[a] += 1.0
        total_reward[a] += reward
        avg_reward[a] = total_reward[a] / times_pulled[a]
        # print how many times we pulled each arm
        print("round", idx, "times_pulled", times_pulled)


main()