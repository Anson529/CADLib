import os
import pickle
import numpy as np

import open3d as o3d

catID = 11
catName = 'building'

infoDir = 'E:\work\kitti360\kitti360Scripts\kitti360scripts\custom/all_bboxes'
outDir = f'E:\CARLA\CAD\library/{catName}'
dataDirs = os.listdir(infoDir)

info = []

meshAll = None

for dataDir in dataDirs:
    dataPath = f'{infoDir}/{dataDir}/{catID}'

    if os.path.exists(dataPath) == False:
        continue

    gridDirs = os.listdir(dataPath)

    for gridDir in gridDirs:
        gridPath = f'{dataPath}/{gridDir}'
        meshDirs = os.listdir(gridPath)

        for meshDir in meshDirs:
            if meshDir.endswith('ply'):
                meshPath = f'{dataDir}/{catID}/{gridDir}/{meshDir}'

                gloID = meshDir.split('.')[0]

                R = np.load(f'{gridPath}/TFmats/{gloID}_R.npy')
                T = np.load(f'{gridPath}/TFmats/{gloID}_T.npy')

                info.append({'pcd_path': meshPath, 'R': R, 'T': T})

                # print (info[-1])
                # quit()
                

with open(f'{outDir}/info.pkl', 'wb') as f:
    pickle.dump(info, f)