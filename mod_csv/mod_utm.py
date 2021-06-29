from pyproj import Proj, transform
import pandas as pd
import csv
import numpy as np

data = pd.DataFrame(columns=['lat','lon'])

characters = "\""
##count = 0
file = open("./11.csv","r")
while True:
    #print(count)
    line = file.readline().rstrip("\n")
    for i in range(len(characters)):
        line = line.replace(characters[i],"")
        line = line.replace(" ","")
        line = line.replace("\n","") 
    ind = line.find(',')
    x = str(line[0:ind])
    y = str(line[ind+1:])
    #print(x+', '+y)
    if bool(line):
        lat, lon = transform(Proj('epsg:32652'),Proj('epsg:4326'),float(x),float(y))
        print (str(lat) + ', ' + str(lon))
        data = data.append({'lat':str(lat),'lon':str(lon)}, ignore_index=True)
    else:
        break
    #count = count + 1

data.to_csv('test1.text', index=False)
