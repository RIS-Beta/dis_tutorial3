# Navodila za zagon vsega

## Zaženi zenoh router
V terminalu 1:

```bash
pkill -9 -f ros && ros2 daemon stop
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## Zaženi simulacijo
V terminalu 2:


Za že zgrajeno mapo:
```bash
ros2 launch dis_tutorial3 sim_turtlebot_nav.launch.py
```

## Zaženi detect_people.py` skripto
V terminalu 3:

```bash
source /opt/ultralytics/bin/activate
ros2 run dis_tutorial3 detect_people.py
```


## Za voice command serivce

Potrebujes .env z kitty:
[text](https://github.com/KittenML/KittenTTS)
in pa simpleaudio ter soundfile.

```bash
python3 -m venv kitten_env && source kitten_env/bin/activate
pip install numpy soundfile simpleaudio kittentts
```

Moraš vžgati service da lahko posiljas na topic.
```bash
ros2 run dis_tutorial3 voice_commander.py 
```
Za testiranje 
```bash
ros2 service call /speech dis_tutorial3/srv/Speech "{text: 'Hello'}"
```
Drugace samo v nodih publisas na topic /speech z custom servicom ki ga imamo 


# Mission controler

All of the following commands must run in seperate terminals
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```
```bash
ros2 launch dis_tutorial3 sim_turtlebot_nav.launch.py
```
```bash
ros2 run dis_tutorial3 detect_people.py 
```
```bash
ros2 run dis_tutorial3 detect_rings.py 
```
```bash
ros2 ros2 run dis_tutorial3 voice_commander.py
```
Then we turn on the mission_controler.py with predefined points and hope it works:
```bash
ros2 run dis_tutorial3 mission_controler.py
```

