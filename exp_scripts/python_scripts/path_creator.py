import os
import numpy as np
import pandas as pd

path = "/home/akshay/parameters.csv"

def create_csv(data):
   
    d = {"Samples": data[0],"Horizon": data[1],"Q": data[2],"Qf": data[3],"R": data[4]}
    
    print(os.path.isfile(path))
    if os.path.isfile(path):
        df = pd.read_csv(path)
        df.loc[len(df)] = data
    else:
        df = pd.DataFrame(pd.Series(d)).T
    
    df.to_csv(path, mode= 'w', index= False, header= True)

if __name__ == "__main__":
    data = [0.0, 0.1, 0.2, 0.3, 0.4]
    create_csv(data)
