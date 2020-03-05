import numpy as np

def test():
    b=np.add(a,np.array([[1],[2]]))
    return b, 0, 1
a = np.array([[1],
         [4]])
print(a)
a, b, c = test()
print(a)