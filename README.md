# ARL_Project

## Śledzenie dronem DJI TELLO taga Aruco umieszczonego na innym ruchomemym przedmiocie

## Instalacja 


1. Sklonuj katalog repozytorium i otwórz w nim terminal

2. Zbuduj obraz Docker za pomocą komendy
```bash
./build_image.sh
```
3. W przypadku pierwszego uruchomienia stwórz kontener:
```bash
./create_container.sh
```
4. Podłączenie terminala do kontenera:
```bash
docker exec -it arl_foxy bash
```

## Symulacja

1. Uruchom kontener:
```bash
docker start arl_foxy
```
2. Podłącz się do terminala:
```bash
docker exec -it arl_foxy bash
```
3. Zbuduj pakiet ros2:
```bash
colcon build 
```

4. Wczytaj instalacje:
```bash
source install/setup.bash
```

5A. Symulacja z jednym dronem:
```bash
colcon build && ros2 launch tello_arl simulation_1drone.launch.py
```

5B. Symulacja z dwoma dronami:
```bash
colcon build && ros2 launch tello_arl simulation_2drones.launch.py
```

## Rzeczywisty dron

1. Uruchom kontener:
```bash
docker start arl_foxy
```
2. Podłącz się do terminala:
```bash
docker exec -it arl_foxy bash
```
3. Zbuduj pakiet ros2:
```bash
colcon build 
```
4. Wczytaj instalacje:
```bash
source install/setup.bash
```
5. Podłącz się do WiFi drona.

6. Uruchom teleop_node
```bash
ros2 launch tello_driver teleop_launch.py
```

7. Uruchom konteroler drona:
```bash
colcon build && ros2 launch tello_arl control_drone.launch.py
```




