import serial
import sys

def acquisition(file, numSamples=200, device=''):

  samples  = 0
  
  ser = serial.Serial(device, 115200, timeout=10)
  
  data = 'aX,aY,aZ\r\n'

  # read dummy line
  ser.readline().decode("utf-8")

  while(samples < numSamples):
    line = ser.readline().decode("utf-8")
    data = data + line
    samples = samples + 1

  print("Done receiving %d samples!" % samples)
  file.write(data)
  ser.close()

if __name__ == '__main__':

  if (len(sys.argv) > 3):
    fileName   = str(sys.argv[1])
    numSamples = int(sys.argv[2])
    device     = str(sys.argv[3])

    with open(fileName, "w") as f:
      
      acquisition(f, numSamples, device)
      f.close()

  else:
    print('\r\nMissing parameters!')
    print('Usage: %s <FILENAME> <NUM_SAMPLES> <DEVICE>' % sys.argv[0])
  