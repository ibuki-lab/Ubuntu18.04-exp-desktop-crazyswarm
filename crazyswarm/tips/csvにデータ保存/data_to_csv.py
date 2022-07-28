import numpy as np
import pandas as pd
import time

data = {}
x = [[] for _ in range(2)]
y = [[] for _ in range(2)]
t = 0
t_start = time.time()
while True:
    t += time.time() - t_start
    for i in range(2):
        x[i].append(np.sin(2 * np.pi * t/10))
        y[i].append(np.cos(2 * np.pi * t/10))

    if t > 5:
        break
for i in range(2):
    data['x{}'.format(i+1)] = x[i]
    data['y{}'.format(i+1)] = y[i]

df = pd.DataFrame(data)
df.to_csv("data")


    