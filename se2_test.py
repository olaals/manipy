from manipy import SE2
import numpy as np



vec = np.array([0.1,0.2,0.05])
a = SE2.from_vec(vec)
v = a.as_vec()
print(a)
print(v)
