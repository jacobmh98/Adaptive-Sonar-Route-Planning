import numpy as np

a1 = np.array([1, 1])
a2 = np.array([4, 4])
b1 = np.array([2, 2])
b2 = np.array([6, 6])

v1 = a2 - a1
v2 = b2 - b1

print(f'det = {np.cross(v1, v2)}')

t1 = (a1 - a1) @ v1 / (v1 @ v1)
t2 = (a2 - a1) @ v1 / (v1 @ v1)
s1 = (b1 - a1) @ v1 / (v1 @ v1)
s2 = (b2 - a1) @ v1 / (v1 @ v1)

print(f'{t1=}')
print(f'{t2=}')
print(f'{s1=}')
print(f'{s2=}')
print(5/3)