import numpy as np

p_0 = np.array([-17.9, 1.645, 84.201])
p_1 = np.array([-17.856, 2.144, 98.601])
p_2 = np.array([-17.251, 9.063, 137.993])
p_3 = np.array([-16.498, 17.667, 164.647])

print(np.around(np.linalg.norm(p_0 - p_1), 3))
print(np.around(np.linalg.norm(p_1 - p_2), 3))
print(np.around(np.linalg.norm(p_2 - p_3), 3))