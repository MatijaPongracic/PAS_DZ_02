# PAS_DZ_02

## Opis
Matija Pongračić: Projektiranje autonomnih sustava - DZ 02

## Zahtjevi
- ROS 2 instaliran na sustavu
- colcon build alat za izgradnju ROS 2 workspace-a
- Git za preuzimanje repozitorija
- Stage
- SLAM toolbox

## Instalacija i pokretanje

### Kreiranje ROS 2 workspace-a
U terminalu pokrenuti sljedeće naredbe za kreiranje ROS2 workspace-a.
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
```

### Preuzimanje repozitorija
Klonirati repozitorij s GitHub-a u `src` direktorij.
```bash
git clone https://github.com/MatijaPongracic/PAS_DZ_02.git
```

### Buildanje projekta i postavljanje okruženja
Vratiti se u root direktorij workspace-a, izgraditi workspace i source-ati postavke.
```bash
cd ..
colcon build
source install/setup.bash
```

## (1) OPCIONALNO: Pokretanje launch datoteke za mapiranje (ako mapa je već ne postoji)
U prvom terminalu pokrenuti:
```bash
ros2 launch stage_ros2 create_map.launch.py
```
Učitati mapu pomoću ADD -> By topic -> iz topic-a /map dodati Map, nakon čega bi mapa trebala postati vidljiva.

Otvoriti drugi terminal u istoj mapi kao i prvi te pokrenuti:
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Pomoću tipki I, J, K i L voziti robota po mapi sve dok u RVIZ-u nije vidljivo kompletno područje interesa.

Kada je mapa kompletno kreirana, otvoriti još jedan terminal na istoj lokaciji i pokrenuti:
```bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/PAS_DZ_02/src/path_planning/config/moja_karta
```
nakon čega će se mapa spremiti u config folderu paketa za planiranje putanje.

Na kraju se mogu zatvoriti svi terminali.


## (2) Pokretanje launch datoteke za pretraživanje prostora i planiranje putanje - D* Lite algoritam
Otvoriti novi terminal i ponovno build-ati projekt:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Zatim pokrenuti launch datoteku:
```bash
ros2 launch path_planning d_star_lite.launch.py
```

Otvorit će se RVIZ s učitanom mapom te je potrebno odrediti točke START i GOAL.
Za određivanje točke START kliknuti `2D Pose Estimate` i klinuti na neku točku na mapi.
ZA određivanje točke GOAL kliknuti `2D Goal Pose` i klinuti na neku točku na mapi.

Nakont toga započinje vizualizacija pretraživanja te se na kraju prikuje pronađena putanja.
U slučaju da putanja nije pronađena, vizualizacija se ne prikazuje postepeno, već se trenutno prikažu svi pretraženi čvorovi.

