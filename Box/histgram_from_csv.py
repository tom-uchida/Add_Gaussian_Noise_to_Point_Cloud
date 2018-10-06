import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("noise_distribution.csv", names= ['Vol', 'chars'])
print(df.describe())
plt.hist(df['chars'], bins=100)
plt.title('Gaussian noise distribution')
plt.ylabel('Frequency')
plt.show()