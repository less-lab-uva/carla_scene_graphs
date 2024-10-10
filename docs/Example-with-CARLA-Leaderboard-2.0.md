A full guide how to set everything up can be found [here](https://leaderboard.carla.org/get_started). However, here is a quick guide to get you started.

1. Download [CARLA leaderboard release](https://leaderboard-public-contents.s3.us-west-2.amazonaws.com/CARLA_Leaderboard_2.0.tar.xz), and [ScenarioLogs](https://leaderboard-logs.s3.us-west-2.amazonaws.com/ScenarioLogs.zip), you can do it by executing:

```bash
mkdir leaderboard2
wget https://leaderboard-public-contents.s3.us-west-2.amazonaws.com/CARLA_Leaderboard_2.0.tar.xz -P leaderboard2/
wget https://leaderboard-logs.s3.us-west-2.amazonaws.com/ScenarioLogs.zip -P leaderboard2/
cd leaderboard2/
tar -xvf CARLA_Leaderboard_2.0.tar.xz
unzip ScenarioLogs.zip
mv ScenarioLogs ScenarioLogs_rm
mv ScenarioLogs_rm/ScenarioLogs/ ScenarioLogs
rm -rf ScenarioLogs_rm/
```

2. Create a conda environment by executing:
```bash
conda env create -f environment.yml --prefix .carlasg
conda activate .carlasg/
```

3. Create an environment file `.env` and Update the following environment variable paths: 
```
CARLA_SGG_DIR={Absolute_path}/carla_scene_graphs/carla_sgg/
CARLA_9_14_DIR={Absolute_path}/carla_scene_graphs/leaderboard2/CARLA_Leaderboard_20/
LEADERBOARD2_DIR={Absolute_path}/carla_scene_graphs/leaderboard2/
```
After that, source the environment variables by executing:
```bash
source .env
```

4. Launch CARLA on port 2000:
```bash
export CARLA_SERVER=$CARLA_9_14_DIR/CarlaUE4.sh
${CARLA_SERVER} --world-port=2000 -opengl -RenderOffscreen
```
Alternatively you can simply execute:
```bash
. launch_carla.sh
```

5. Launch capture_sensor_data.py to collect scene graphs by executing:
```bash
export PYTHONPATH=$PYTHONPATH:$CARLA_9_14_DIR/PythonAPI
export PYTHONPATH=$PYTHONPATH:$CARLA_9_14_DIR/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_9_14_DIR/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:$CARLA_SGG_DIR
python capture_sensor_data.py --record {Path to ScenarioLog record}
```
Alternatively you can simply execute:
```bash
. launch_leaderboard2.sh
```