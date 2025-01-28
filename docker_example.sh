sudo -u carla /bin/bash ./CarlaUE4.sh 2>/dev/null &
echo "Waiting for CARLA to load..."
sleep 10
cd /carla_sgg/
conda activate carla_sgg
python3 mwe.py
