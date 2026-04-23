import math
import numpy as np
import matplotlib.pyplot as plt

def taylor_expansion_exponential(x, n_terms):
    exp = 0
    for i in range (0, n_terms+1):
        exp += ((x**i)/(math.factorial(i)))
    return exp

taylor_exp = taylor_expansion_exponential(5, 15)
print(taylor_exp)
print(math.exp(5))