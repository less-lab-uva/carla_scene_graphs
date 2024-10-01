Steps to install CARLA on fresh Ubuntu 20.04

# Install GPU Drivers
1. Install driver
```bash
sudo apt install nvidia-driver-530 -y
```


# Install CARLA
0. Setup dirs
```bash
mkdir $HOME/sim
cd $HOME/sim
```

1. Setup [conda](https://conda.io/projects/conda/en/latest/user-guide/install/index.html)
Follow the above link to install conda as appropriate for your system. 

2. Setup the conda environment:
The environment will be called `carla_sgg` and is built from the `environment.yml` file:
```bash
conda env create -f environment.yml --prefix .carlasg
conda activate .carlasg/
```

3. Download CARLA from GitHub [here](https://github.com/carla-simulator/carla/blob/master/Docs/download.md) using [0.9.14](https://github.com/carla-simulator/carla/releases/tag/0.9.14/) as the current latest. This will take several minutes
```bash
cd $HOME/sim
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.14.tar.gz
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.14.tar.gz
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.14_RSS.tar.gz
```

4. Unzip and cleanup. This will take a minute or two
```bash
cd $HOME/sim
tar -xvzf CARLA_0.9.14.tar.gz -C $HOME/sim
tar -xvzf AdditionalMaps_0.9.14.tar.gz -C $HOME/sim/Import
rm -rf CARLA_0.9.14.tar.gz
rm -rf AdditionalMaps_0.9.14.tar.gz
```

5. Install the additional maps
```bash
cd $HOME/sim
./ImportAssets.sh
```

6. Setup CARLA Python Client Library. 
Optional: This will use the local CARLA library.
```bash
pip3 install $HOME/sim/PythonAPI/carla/dist/carla-0.9.14-cp37-cp37m-manylinux_2_27_x86_64.whl
```

7. Install misc. libs
```bash
sudo apt-get install libomp5 -y
sudo apt install vulkan-utils -y
sudo apt-get install graphviz -y
```

8. Launch CARLA
```bash
cd $HOME/sim
./CarlaUE4.sh
```
