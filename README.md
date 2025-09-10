# Embedded Machine Learning (E747)

Projektdateien für das Modul E747: Embedded Machine Learning


### Benötigte Werkzeuge

- [Python](https://www.python.org/) (am besten über [Conda](http://anaconda.com/))
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

### Verwendung einer virtuellen Python-Umgebung

- Installation von [Miniconda](https://www.anaconda.com/docs/getting-started/miniconda/main) unter Windows/Linux/MacOS
- Workflow zur Erstellung der conda-Umgebung:
```bash
# 1. Erstellung einer virtuellen Umgebung mit dem Namen 'embeddedml'
conda create --name embeddedml -c conda-forge python=3.11

# 2. Virtuelle Python-Umgebung aktivieren
conda activate embeddedml

# 3. Notwendige Python-Pakete in der virtuellen Umgebung installieren
pip install -r requirements.txt
```
