# Predictive Maintenance

Dieses Projekt demonstriert Predictive Maintenance. Dazu werden Daten eines Beschleuigungssensors aufgenommen, der an einem Tischventilator befestigt ist. Mit diesen Daten wird ein neuronales Netz trainiert, welches unterschiedliche Fehlerfälle erkennt.

```
├── data
│   ├── fft                                  # Trainingsdatensätze (Frequenzspektrum)
│   └── time                                 # Trainingsdatensätze (Zeitreihen)
├── data_selected
├── model
│   └── predMaintenanceFFT.tflite            # Trainiertes Modell
├── notebook
│   ├── fft_vs_cmsis.ipynb                   # Vergleich zwischen SciPy.FFT und CMSIS-DSP-FFT
│   ├── PredictiveMaintenanceFFT.ipynb       # Training eines Netzes auf Basis der FFT-Daten
│   └── PredictiveMaintenanceTime.ipynb      # Training eines Netzes auf Basis der Zeitreihen
├── README.md
├── src
│   ├── nucleo-l432-pm-capture               # STM32L432-Projekt mit Anbindung des LIS3DH IMU-Sensors
│   ├── nucleo-l432-pm-capture_fft           # STM32L432-Projekt inkl. Datenvorverarbeitung
│   └── nucleo-l432-pm-inference             # STM32L432-Projekt mit neuronalem Netz für Inferenz
└── tools
    └── fftPlot.py                           # Python-Script zur grafischen Darstellung der FFT-Daten
```