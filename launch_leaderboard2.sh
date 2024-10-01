export PYTHONPATH=$PYTHONPATH:$CARLA_9_14_DIR/PythonAPI
export PYTHONPATH=$PYTHONPATH:$CARLA_9_14_DIR/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_9_14_DIR/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:$CARLA_SGG_DIR

python capture_sensor_data.py --record leaderboard2/ScenarioLogs/Accident