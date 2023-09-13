import math


def N(x, u, o):
    result = (1/(math.sqrt(2 * math.pi) * o)) * (math.exp(-0.5 * ((x - u)**2/o**2)))
    return result


def p(x):
    result = 0.3 * N(x, 2.0, 1.0) + 0.4 * N(x, 5.0, 2.0) + 0.3 * N(x, 9.0, 1.0)
    return result

print(p(5))