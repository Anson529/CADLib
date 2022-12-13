import open3d as o3d
import numpy as np
import pickle
import os

from scipy.spatial.transform import Rotation as sc_R

SCALE = 0.01

def decomposition(mat):
    scales = np.array([np.linalg.norm(mat.T[0]), np.linalg.norm(mat.T[1]), np.linalg.norm(mat.T[2])])
    rot = mat / scales
    
    R = sc_R.from_matrix(rot)
    angle = R.as_euler('xyz')[2]
    out = np.append(scales, angle)

    return out

def composition(out):
    R = sc_R.from_euler('xyz', [0, 0, out[3]])
    rot = R.as_matrix()

    rot = rot * out[:3]

    return rot

def transMesh(mesh, R, T):
    points = np.array(mesh.vertices) @ R.T + T
    mesh.vertices = o3d.utility.Vector3dVector(points)

def getLoss(X, Y):
    return ((X - Y) ** 2).sum()

catName = 'building'
dataID = 9

# dataDir = f'E:\work\kitti360\code\processed/{catName}_new\data'
CADDir = f'E:\CARLA\CAD\library\{catName}'
dataDir = CADDir
gtCADDir = 'E:\work\kitti360\kitti360Scripts\kitti360scripts\custom/all_bboxes'

with open(f'{dataDir}/info.pkl', 'rb') as f:
    gtInfo = pickle.load(f)
# 
with open(f'{CADDir}/CADInfo.pkl', 'rb') as f:
    CADInfo = pickle.load(f)

cnt = 0
CADSizes = []

nCADs = len(CADInfo)

for CAD in CADInfo:
    CADSize = CAD['size'] * SCALE
    
    if catName == 'building':
        CADSize[0], CADSize[1] = CADSize[1], CADSize[0]
        
    CADSizes.append(CADSize)

result = []
ID = str(dataID)

meshAll = None

for info in gtInfo:
    path = info['pcd_path']
    if path.split('/')[0] != ID:
        continue

    gID = path.split('/')[-1].split('.')[0]
    
    R, T = info['R'], info['T']

    R = decomposition(R)
    T[2] -= R[2] / 2  
    R[3] += np.pi / 2

    losses = []
    for CADSize in CADSizes:
        losses.append(getLoss(CADSize, R[:3]))
        print (CADSize, R[:3])

    
    p_cad = np.argmin(losses)

    R[:3] = [1, 1, 1]

    result.append({'id': p_cad, 'gID': int(gID), 'R': composition(R), 'T': T})
    mesh = o3d.io.read_triangle_mesh(f'{gtCADDir}/{path}')

    if meshAll is None:
        meshAll = mesh
    else:
        meshAll += mesh

os.makedirs(f'{CADDir}/result9', exist_ok=True)
o3d.io.write_triangle_mesh(f'{CADDir}/result9/bbox.ply', meshAll)

with open(f'{CADDir}/result{ID}.pkl', 'wb') as f:
    pickle.dump(result, f)

print (cnt)