import torch

tensor = torch.tensor([1,2,3,4,5,6,7,8,9,10,
                       11,12,13,14,15,16,17,18,19,20,
                       21,22,23,24,25,26,27,28,29,30,
                       31,32,33])
tensor = torch.randn(5, 33)
sizes = [6,7,10,10]
print(tensor)

split_tensors = torch.split(tensor, [6,7,10,10], dim=1)

for i, split_tensors in enumerate(split_tensors):
  print(split_tensors.shape)