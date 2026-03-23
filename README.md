## Installation
* Create a new virtual environment.
    ```bash
    conda create -n inspire_hands_unitree python=3.10
    conda activate  inspire_hands_unitree
    ```
* Install unitree sdk 
    ```bash
    cd ../
    git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
    cd unitree_sdk2_python
    pip install -e .
    ```
* Install inspire hand sdk
    ```bash
    pip install -e .
    ```