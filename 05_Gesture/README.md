# Gestenerkennung

Dieses Projekt demonstriert die Gestenerkennung mittels neuronalem Netz für zwei unterschiedliche Klassen auf Basis von Beschleunigungssensordaten. Im ersten Schritt werden mittels Beschleunigungssensor Trainingsdaten für zwei Klassen aufgezeichnet. Anschließend werden die Daten zum Training eines neuronalen Netzes eingesetzt. Im letzten Schritt wird das neuronale Netz auf einem STM32L432-Nucleo implementiert.

```
├── data
│   ├── acquisition.py                          # Python-Script zur Datenaufnahme
│   ├── flex.csv                                # Beispieldatensatz 'Flex'
│   └── punch.csv                               # Beispieldatensatz 'Punch'
├── model
│   └── gesture_model.tflite                    # Trainiertes Modell
├── notebook
│   └── Gesture.ipynb                           # Jupyter Notebook für die Erstellung und Training des neuronalen Netzes
├── README.md
└── src
    ├── nucleo-l432-lsm6dsox                    # STM32L432-Projekt mit Anbindung des LSM6DSOX-Beschleunigungssensors
    ├── nucleo-l432-lsm6dsox-capture            # STM32L432-Projekt für die Erhebung der Trainingsdaten
    ├── nucleo-l432-lsm6dsox-inference          # STM32L432-Projekt mit neuronalem Netz für Inferenz
    └── nucleo-l432-template                    # STM32L432 Template (RCC, SWD, Clock, UART)
```