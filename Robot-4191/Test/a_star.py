import numpy as np
import pyastar2d
# The minimum cost must be 1 for the heuristic to be valid.
# The weights array must have np.float32 dtype to be compatible with the C++ code.
weights = np.array([[1, 3, 3, 3, 3],
                    [2, 1, 3, 3, 3],
                    [2, 2, 1, 3, 3],
                    [2, 2, 2, 1, 3],
                    [2, 2, 2, 2, 1]], dtype=np.float32)
# The start and goal coordinates are in matrix coordinates (i, j).
path = pyastar2d.astar_path(weights, (0, 0), (4, 4), allow_diagonal=True)
print(path)
# The path is returned as a numpy array of (i, j) coordinates.
