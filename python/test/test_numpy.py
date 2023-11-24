import numpy as np

val = 12*207*128*128*3
#data = np.random.rand(45612928)
data = np.random.rand(val)

new_shape = (12,207,128,128,3)

try:
  reshaped_data = data.reshape(new_shape)
except ValueError as e:
  print(f"Error: {e}")