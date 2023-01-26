# Śledzenie dronem DJI TELLO taga Aruco umieszczonego na innym ruchomemym przedmiocie

### Opracowany pakiet oprogramowania pozwala na autnomiczne podążanie drona za znacznikiem wizyjnym Aruco zarówno w symulacji jak i w rzeczywistym świecie.

## Założenia projektu 
### Plan
1. Testowanie symulacji
2. Detekcja i przemieszczenie do aruco
3. Podążanie za ruchomym aruco
4. Rejestracja przelotu drona z aruco
5. Odtworzenie lotu

### Finalny rezultat
Ostatecznie udało nam się zrealizować aplikacje umożliwiającą podążanie jednego drona za drugim przy pomocy taga aruco. Znaczną część projektu stanowiła optymalizacja i dobór nastaw prędkości dla podążającego drona. Mimo ograniczonych możliwości testowania programu na dronach w laboratorium udało się nam uzyskać oczekiwany efekt. Od początku trwania projektu zakładaliśmy, że ostatni punkt planu jest opcjonalny, ponieważ realizacja takiej funkcjonalności wymagałaby dodania osobnej logiki programu. Efekt odtworzenia lotu, byłby jednak efektowną funkcją, więc może stanowić punt wyjścia do dalszego rozwoju projektu. Główne założenie podążania dronem za drugą jednostką zostało w pełni wykonane.

## Instalacja

### 1. Sklonuj katalog repozytorium i otwórz w nim terminal

### 2. Zbuduj obraz Docker za pomocą komendy

```bash
./build_image.sh
```

### 3. W przypadku pierwszego uruchomienia stwórz kontener:

```bash
./create_container.sh
```

### 4. Podłączenie terminala do kontenera:

```bash
docker exec -it arl_foxy bash
```

## Symulacja

### 1. Uruchom kontener:

```bash
docker start arl_foxy
```

### 2. Podłącz się do terminala:

```bash
docker exec -it arl_foxy bash
```

### 3. Zbuduj pakiet ros2:

```bash
colcon build
```

### 4. Wczytaj instalacje:

```bash
source install/setup.bash
```

### 5A. Symulacja z jednym dronem:

```bash
ros2 launch tello_arl simulation_1drone.launch.py
```

### 5B. Symulacja z dwoma dronami:

```bash
ros2 launch tello_arl simulation_2drones.launch.py
```

### 6. W przypadku edycji kodu kontrolera należy przebudować pakiet:

```bash
colcon build --packages-select tello_arl && source install/setup.bash
```

## Rzeczywisty dron

### 1. Uruchom kontener:

```bash
docker start arl_foxy
```

### 2. Podłącz się do terminala:

```bash
docker exec -it arl_foxy bash
```

### 3. Zbuduj pakiet ros2:

```bash
colcon build
```

### 4. Wczytaj instalacje:

```bash
source install/setup.bash
```

### 5. Podłącz się do WiFi drona.

### 6. Uruchom teleop_node

```bash
ros2 launch tello_driver teleop_launch.py
```

### 7. Uruchom konteroler drona:

```bash
ros2 launch tello_arl control_drone.launch.py
```

### 8. Uruchomienie konterolera po zmianach kodu źródłowego:

```bash
colcon build --packages-select tello_arl && ros2 launch tello_arl control_drone.launch.py
```

## Inne polecenia

### Wszystkie istotne polecenia zapisane są do historii poleceń w tworzonym kontenerze, zamiast je wpisywać, możesz znaleźć je używając strzałki w góre

## Symulacja

### Lądowanie:

```bash
ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
```

### Startowania:

```bash
ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
```

### Reset symulacji

```bash
ros2 service call /reset_simulation std_srvs/srv/Empty
```

## Rzeczywisty dron

### Lądowanie:

```bash
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"
```

### Startowania:

```bash
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
```

# Używane biblioteki

```bash
transformations                      2022.9.26
scipy                                1.10.0
opencv-contrib-python                4.7.0.68
```

# Modyfikacje

### Wszystkie kluczowe parametry sterowania dronem oraz typ i rozmiar znacznika wizyjnego można edytować w plikach `launch.py` znajdujących się w katalogu `src_files\src\tello_arl`, są tam pliki startowe dla symulacji jak i kontrolera rzeczywistego drona.
