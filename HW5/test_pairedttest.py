from scipy.stats import ttest_rel
import numpy as np

# --------- Raw data  ---------
person1_unassisted_scores = [4]
person2_unassisted_scores = []
person3_unassisted_scores = []
person4_unassisted_scores = []

person1_assisted_scores = [2]
person2_assisted_scores = []
person3_assisted_scores = []
person4_assisted_scores = []

person1_unassisted_time = [221]
person2_unassisted_time = []
person3_unassisted_time = []
person4_unassisted_time = []

person1_assisted_time = [371]
person2_assisted_time = []
person3_assisted_time = []
person4_assisted_time = []

# --------- Averages per person ---------
unassisted_scores = person1_unassisted_scores + person2_unassisted_scores + person3_unassisted_scores + person4_unassisted_scores
assisted_scores   = person1_assisted_scores   + person2_assisted_scores   + person3_assisted_scores   + person4_assisted_scores

unassisted_time = person1_unassisted_time + person2_unassisted_time + person3_unassisted_time + person4_unassisted_time
assisted_time   = person1_assisted_time   + person2_assisted_time   + person3_assisted_time   + person4_assisted_time

# --------- Tests ---------
stat, p = ttest_rel(assisted_scores, unassisted_scores, alternative='greater')
print(f"Score p-value: {p:.4f}")

stat, p = ttest_rel(unassisted_time, assisted_time, alternative='lesser')
print(f"Time p-value: {p:.4f}")