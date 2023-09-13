import math
import random
import matplotlib.pyplot as plt
import numpy as np

def N(x, u, o):
    result = (1 / (math.sqrt(2 * math.pi) * o)) * (math.exp(-0.5 * ((x - u) ** 2 / o ** 2)))
    return result

def p(x):
    result = 0.3 * N(x, 2.0, 1.0) + 0.4 * N(x, 5.0, 2.0) + 0.3 * N(x, 9.0, 1.0)
    return result

def q(x):
    if x < 0 or x > 15:
        return 0
    else:
        return 1 / 15

""" def q2(x):
    if x < 0 or x > 15:
        return 0
    for i in range(x):
        lst.append(np.random.normal(0, 15))
    return lst
"""

""" def pq(x):
    probabilities = []
    for i in range(len(q(x))):
        probabilities.append(p(q(x)[i]))
    return probabilities

"""

def samplegenerator(k):
    lst = []
    for i in range(k):
        lst.append(np.random.uniform(0, 15))
    return lst

# Generate a range of x values
x1_values = samplegenerator(1000)  # Adjust the range as needed

# Calculate the corresponding p(x) values
p_values = [p(x)/q(x) for x in x1_values]

# normalize p_values so that the area under the curve sums to 1
norm_p_values = [x / sum(p_values) for x in p_values]

# draw 1000 random samples from x1_values, but weighted by p_values
x1_samples = np.random.choice(x1_values, size=1000, replace=True, p=norm_p_values)

# Create a histogram plot
plt.figure(0)
plt.figure(figsize=(10, 6))
plt.hist(x1_values, bins=50, density=True, alpha=0.6, color='blue', label='p(x) Histogram')
#plt.plot(x_values, p_values, color='red', linewidth=2, label='p(x) Curve')

plt.xlabel('x')
plt.ylabel('Probability Density')
plt.title('Histogram of x1_values')
plt.legend()
plt.grid(True)




plt.figure(1)
plt.figure(figsize=(10, 6))
plt.hist(x1_samples, bins=50, density=True, alpha=0.6, color='blue', label='p(x) Histogram')
plt.plot(x1_samplnorm_p_values, color='red', linewidth=2, label='p(x) Curve')

plt.xlabel('x')
plt.ylabel('Probability Density')
plt.title('Histogram of x1_samples(resampled)')
plt.legend()
plt.grid(True)




plt.show()

