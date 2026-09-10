# PAS-DUAL-ARM (Simulacijski Model)

Ovaj repozitorij sadrži cjelokupni simulacijski model i okruženje za robota s dvije ruke (Dual Arm) i omnidirekcijskom bazom. Model je razvijen u okviru seminara iz kolegija, a cilj je realistična ROS2 Humble i Gazebo Fortress simulacija.

## Arhitektura i Alati

Projekt koristi sljedeće ključne komponente:
- **Baza**: PAL Robotics `omni_base_simulation`
- **Torzo**: Custom dizajn s dva vertikalna linearna klizača (vodilice) pokretana `prismatic` zglobovima. Mase su aproksimirane prema standardnim aluminijskim profilima.
- **Ruke**: Dva komada Kinova Gen3 (7-DOF) sa `robotiq_2f_85` hvataljkama, preuzete iz `ros2_kortex`.
- **Senzorika**: Pan-tilt mehanizam (`pan_tilt_ros`) s montiranom Intel RealSense D435 kamerom (`realsense-ros`).
- **Gazebo Simulator**: Gazebo Fortress LTS, korištenjem `ign_ros2_control` plugina za hardversku apstrakciju upravljanja zglobovima.

## Odabir Aruco Markera i Parametara
Za detekciju i manipulaciju ciljanom kutijom (0.3x0.3x0.3 m, 1 kg) odabran je **Aruco Marker iz DICT_4X4_50 rječnika (ID 0)**.
*Opravdanje:* Manji rječnik (50 markera) smanjuje false positive detekcije i ubrzava obradu slike na robotu u stvarnom vremenu. Dimenzija od 4x4 pixela (unutarnja matrica) pruža dovoljnu robusnost i savršeno je prikladna za identifikaciju većih objekata poput kutija u logistici (gdje nema mnogo različitih unikatnih oznaka koje su simultano vidljive).

## Pokretanje i Korištenje (Vodič)

> Napomena: Slijedite upute iz `HUMAN.md` datoteke kada je potrebna vaša intervencija za vizualnu i funkcionalnu provjeru.

### 1. Preduvjeti i Instalacija (Setup)
Sustav se oslanja na Ubuntu 22.04 i ROS2 Humble. Automatska instalacija rješava `rosdep` i sistemske alate. Za ručnu instalaciju (već provedenu skriptom):
```bash
sudo apt update
sudo apt install ignition-fortress ros-humble-ros-gz ...
rosdep install --from-paths src --ignore-src -y -r
```

### 2. Kompilacija
Nakon kloniranja svih repozitorija (`omni_base`, `ros2_kortex`, itd.) unutar `src/`, prevedite sve
iz eksplicitnog projektnog okruženja:
```bash
cd ~/FSB/PAS-DUAL-ARM
./scripts/run_native.sh colcon build --symlink-install
```

Za interaktivni rad otvorite projektni shell:

```bash
./scripts/run_native.sh
```

Shell učitava samo `/opt/ros/humble` i lokalni PAS-DUAL-ARM overlay, koristi Fast DDS, ROS domenu 5
i lokalno otkrivanje čvorova. Prije prelaska u drugi ROS projekt izađite naredbom `exit`; nemojte
učitavati njegov `setup.bash` u isti shell. Globalni `~/.bashrc` ne smije učitavati ROS distribuciju,
workspace, middleware, domenu ni adresu robota.

### 3. Vizualna Verifikacija (RViz2)
Kako biste pregledali statični model (bez Gazeba, služi za provjeru URDF-a i spajanja ruku):
```bash
ros2 launch pas_dual_arm_bringup display.launch.py
```
Ovdje možete pomicati klizače i promatrati kinematic-stablo robota.

### 4. Pokretanje Gazebo Simulacije
Za pokretanje punog okruženja (svijet + robot):
```bash
ros2 launch pas_dual_arm_bringup sim.launch.py
```
Ova skripta učitava `seminar_world.sdf` koji sadrži zid s 0.8m prolazom, startnu kutiju s Aruco markerom i ciljani stol na fiksnoj lokaciji, a zatim instancira kontrolere za aktuatore robota.

## Očekivani Rad (Future Tasks)
1. Detekcija Aruco markera putem kamere (aruco_ros).
2. Autonomna navigacija (Nav2) pomoću Lidar senzora na bazi.
3. Rješavanje inverzne kinematike (MoveIt2) za zahvat objema rukama simultano.
4. Prenošenje kroz vrata i odlaganje na stol.
