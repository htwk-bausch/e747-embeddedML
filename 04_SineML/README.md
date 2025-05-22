# SinusML

Dieses Verzeichnis enthält die Projektdateien zum Training eines neuronalen Netzes zur Erzeugung eines Sinus-Signals auf Basis eines Eingangswertes zwischen 0 und $2 \pi$.

```
├── model
│   ├── sine_model.h5              # Trainiertes Modell im Keras-Format (nicht optimiert)
│   └── sine_model.tflite          # Trainiertes Modell im TensorFlow light-Format (optimiert)
├── notebook
│   └── SineML.ipynb               # Jupyter Notebook zur Erstellung von Trainingsdaten sowie Training des neuronalen Netzes
├── README.md
└── src
    ├── nucleo-l432-sine_ml        # STM32-Projekt inkl. neuronalem Netz (statische Berechnung eines Wertes)
    ├── nucleo-l432-sine_ml2       # Erweiterung mit Timer zur Messung der Inferenz
    └── nucleo-l432-sine_ml3       # Erweiterung (2) mit Berechnung aller Werte zwischen 0 ... 2*PI
```