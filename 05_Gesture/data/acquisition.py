import serial
import sys

def acquisition(file, device, numSamples=104, numDatasets=1):

  datasets = 0
  samples  = 0

  ser = serial.Serial(device, 115200, timeout=10)

  data = 'aX,aY,aZ,gX,gY,gZ\r\n'

  while(datasets < numDatasets):

    #print("%d: %s" % (samples, ser.readline().decode("utf-8").rstrip('\r\n')))      

    line = ser.readline().decode("utf-8")
    data = data + line
    

    if (samples == numSamples):

      print("Received %d samples for dataset %d/%d" % (samples, datasets+1, numDatasets))
      datasets = datasets + 1
      samples = 0

      file.write(data)
      data = ''
    
    samples = samples + 1

  ser.close()

if __name__ == '__main__':

  if (len(sys.argv) > 4):
    fileName    = str(sys.argv[1])
    numSamples  = int(sys.argv[2])
    numDatasets = int(sys.argv[3])
    device      = str(sys.argv[4])

    with open(fileName, "w") as f:
      
      #f.write("aX,aY,aZ,gX,gY,gZ\r\n")
      acquisition(f, device, numSamples, numDatasets)
      f.close()
  
  else:
    print('\r\nUsage: %s <FILENAME> <NUM_SAMPLES> <NUM_DATASETS> <DEVICE>' % sys.argv[0])