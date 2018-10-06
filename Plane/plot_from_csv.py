import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def gaussian_function(_x, _sigma):
  y = np.exp( - ( (_x**2) / (2*(_sigma**2)) ) )

  return y

df = pd.read_csv("noise_distribution.csv", names= ['Vol', 'chars'])
noises = df.values
print(noises.shape)
#print(noises[:,1])

plt.scatter(noises[:,1], gaussian_function(noises[:,1], 0.131607))
#plt.scatter(noises[:,1], function(noises[:,1], 1.0))
plt.title('Gaussian noise distribution')
plt.xlabel('x')
plt.ylabel('y')
plt.xlim([-0.3, 0.3])

plt.show()