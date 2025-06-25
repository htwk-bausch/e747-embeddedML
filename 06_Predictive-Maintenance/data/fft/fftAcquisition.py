import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import serial
import sys


def acquisition(file, numDatasets=30, device=''):

  dataset = 0
  ser = serial.Serial(device, 115200, timeout=10)
  
  ydata = np.empty((numDatasets, 128))

  index = 0

  # dummy read lines until new dataset starts
  while(index != 127):
    line = ser.readline().decode("utf-8")
    line = line.rstrip().split(',')
    index = int(line[0])

  while (dataset < numDatasets):

    line = ser.readline().decode("utf-8")
    line = line.rstrip().split(',')

    ydata[dataset][int(line[0])] = float(line[2])

    if (int(line[0]) == 127):
      dataset = dataset + 1
    
  #file.write(ydata)
  #print(ydata)
  df = pd.DataFrame(ydata)
  df.to_csv(file, header=False, index=False)
  #print(df)

  ser.close()


if __name__ == '__main__':

  if (len(sys.argv) > 3):
    device      = str(sys.argv[1])
    numDatasets = int(sys.argv[2])
    fileName    = str(sys.argv[3])  
    
    acquisition(fileName, numDatasets, device)

  else:
    print('\r\nMissing parameters!')
    print('Usage: %s <DEVICE> <NUM_DATASETS> <FILENAME>' % sys.argv[0])