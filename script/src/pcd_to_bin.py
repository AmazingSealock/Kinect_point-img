import os
import numpy as np
import fire
#python pcd2bin.py convert pcdfolder binfolder

def check_float32_conversion(lst):
    for item in lst:
        try:
            np.float32(item)
        except  ( ValueError, TypeError):
            return False
    return True


def read_pcd(filepath):
    lidar = []
    with open(filepath,'r') as f:
        line = f.readline().strip()
        while line:
            linestr = line.split(" ")
            if len(linestr) == 8 and check_float32_conversion(linestr): #for my pcd format, the length of each data line is 8
                linestr_convert = list(map(float, linestr))
                lidar.append(linestr_convert)
            line = f.readline().strip()
    return np.array(lidar)


def convert(pcdfolder, binfolder):
    current_path = os.getcwd()
    ori_path = os.path.join(current_path, pcdfolder)
    file_list = os.listdir(ori_path)
    des_path = os.path.join(current_path, binfolder)
    if os.path.exists(des_path):
        pass
    else:
        os.makedirs(des_path)
    from tqdm import tqdm
    for file in tqdm(file_list): 
        (filename,extension) = os.path.splitext(file)
        velodyne_file = os.path.join(ori_path, filename) + '.pcd'
        pl = read_pcd(velodyne_file)
        pl = pl.reshape(-1, 4).astype(np.float32)
        velodyne_file_new = os.path.join(des_path, filename) + '.bin'
        pl.tofile(velodyne_file_new)
    
if __name__ == "__main__":
    pcdfolder=r'C:\Users\maser\Documents\GitHub\GndNet-master\pcd'
    binfolder=r'C:\Users\maser\Documents\GitHub\GndNet-master\BIN'
    convert(pcdfolder,binfolder)