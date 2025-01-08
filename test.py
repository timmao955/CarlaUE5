import torch

print("Allocated memory:", torch.cuda.memory_allocated())
print("Reserved memory:", torch.cuda.memory_reserved())
 