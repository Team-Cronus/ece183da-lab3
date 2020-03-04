import numpy as np

def test():
    b=np.add(a,np.array([[1],[2]]))
    return b
a = np.array([[1],
         [4]])
print(a)
a = test()
print(a)