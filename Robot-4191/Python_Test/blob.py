from scipy.ndimage import label
import numpy as np



def pad_map(arr, pad_val=80,  null_value = 1, min_blob=2):
    size = arr.shape[0]
    heuristic = [[1,0], [0,1], [-1,0], [0, -1]]
    structure = np.array([
                        [0, 1, 0],
                        [1, 1, 1],
                        [0, 1, 0]])
    labeled, n_blob = label(arr, structure)
    indices = np.indices(arr.shape).T[:,:,[1, 0]]
    print(arr)
    # only pad large blobs
    for i in range(1, n_blob+1):
        mask = np.ma.where(i == labeled)
        mask = list(zip(mask[0], mask[1]))
        if len(mask) > min_blob:
            for point in mask:
                print('point: ', point)
                # pad out using heuristic
                for j in heuristic:
                    new_index = np.add(point,j)
                    if (new_index[0] < size and new_index[1] < size) and (arr[new_index[0], new_index[1]] != null_value):
                        # pad
                        
                        print(new_index, arr[new_index[0], new_index[1]])
                        arr[new_index[0], new_index[1]] = pad_val
    return arr

if __name__ == "__main__":
    
    arr = np.array([[1, 0, 0, 0, 0],
                [0, 0, 0, 1, 1],
                [0, 0, 0, 1, 1],
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0]])
    print(pad_map(arr))
