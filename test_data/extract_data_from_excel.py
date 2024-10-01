import json

import pandas as pd

from Polygon import *

path = 'C:/Users/jacob/Documents/GitHub/Adaptive-Sonar-Route-Planning/test_data/antwerpen.xlsx'
filename = '/test_data/antwerpen_full2.json'

df = pd.read_excel(path, header=None, index_col=0)
vertices = []

x = {
    "area": {
        "coordinates": []
    }
}

for i, r in df.iterrows():
    if r[5] == '#':
        point = np.array(r[2:4])
        x["area"]["coordinates"].append([point[0], point[1]])
print(x)

json_object = json.dumps(x)

with open(filename, 'w') as outfile:
    outfile.write(json_object)