import json

import pandas as pd

from Polygon import *

path = 'C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/antwerpen.xlsx'
filename = './antwerpen_full.json'

df = pd.read_excel(path, header=None, index_col=0)
vertices = []

x = {
    "area": {
        "coordinates": [],
        "hard_edges": []
    },
    "obstacles": {
        "num_of_obstacles": 0
    }
}

for i, r in df.iterrows():
    if r[3] == 'P':
        point = np.array(r[0:2])
        x['area']['coordinates'].append([point[0], point[1]])

x['area']['coordinates'].reverse()
#x['obstacles']['obstacle_1'].reverse()
print(x)

json_object = json.dumps(x)

with open(filename, 'w') as outfile:
    outfile.write(json_object)