import matplotlib.pyplot as plt
import numpy as np
import serial
import sys

plt.ion() # Stop matplotlib windows from blocking

# Setup figure, axis and initiate plot
def plotFFT(device):

  ser = serial.Serial(device, 115200, timeout=10)
  ser.readline().decode("utf-8")  # read dummy line

  fig, ax = plt.subplots(figsize=(10,8))

  xdata = np.empty(128)
  ydata = np.empty(128)
  
  ax.set_title('Frequenzspektrum')
  ax.set_xlabel('Frequenz (Hz)')
  ax.set_ylabel('Betrag')

  ln, = ax.plot([], [])
  ax.grid()

  while True:
    
    line = ser.readline().decode("utf-8")
    line = line.rstrip().split(',')

    if (int(line[0]) != 127):
      # store data in array
      xdata[int(line[0])] = float(line[1])
      ydata[int(line[0])] = float(line[2])
    
    else:
      # store last sample in array
      xdata[int(line[0])] = float(line[1])
      ydata[int(line[0])] = float(line[2])

      # Reset the data in the plot
      ln.set_xdata(xdata)
      ln.set_ydata(ydata)

      # Rescale the axis so that the data can be seen in the plot
      # if you know the bounds of your data you could just set this once
      # so that the axis don't keep changing
      ax.relim()
      ax.autoscale_view()
      
      # Update the window
      fig.canvas.draw()
      fig.canvas.flush_events()

if __name__ == '__main__':

  if (len(sys.argv) > 1):
    device = str(sys.argv[1])
    plotFFT(device)

  else:
    print('\r\nMissing parameters!')
    print('Usage: python %s <DEVICE>' % sys.argv[0])