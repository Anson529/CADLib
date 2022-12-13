import pickle

catName = 'pole'
catID = 9

resultDir = f'E:\CARLA\CAD\library\{catName}/result{catID}.pkl'

with open(resultDir, 'rb') as f:
    info = pickle.load(f)

print (len(info))

print (info[0])
