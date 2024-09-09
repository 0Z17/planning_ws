import rospkg
import os
import numpy as np
from scipy.io import savemat, loadmat

curve_name = 'curve_simple'
data_path = rospkg.RosPack().get_path('planning_utils') + '/data/' + curve_name + '/'

file = 'path_refine.mat'
mat_value_name = 'path_refine'

# identify the data extension
file_full_name = os.path.basename(file)
file_name = os.path.splitext(file_full_name)[0]
file_extension = os.path.splitext(file_full_name)[1]

# check if the file exists
if os.path.isfile(data_path + file_full_name):
    if file_extension == '.mat':
        # convert the file to a npy file
        data = loadmat(data_path + file_full_name)
        # save the data as a npy file
        np.save(data_path + file_name + '.npy', data[mat_value_name])
        print(f'File converted: {data_path + file_name}.npy ')
    elif file_extension == '.npy':
        # convert the file to a mat file
        data = np.load(data_path + file_full_name)
        # save the data as a mat file  
        savemat(data_path + file_name + '.mat', {"data": data})
        print(f'File converted: {data_path + file_name}.mat ')
else:
    print(f'File does not exist:{data_path + file_full_name}')