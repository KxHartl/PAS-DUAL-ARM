# Stanje Projekta (State)

**Trenutna faza**: Početno postavljanje radnog okruženja dovršeno, prelazak na Fazu 2 (URDF).
**Datum zadnje izmjene**: Automatsko ažuriranje

## Preuzeti Repozitoriji (src/)
Svi su klonirani na `humble` / `humble-devel` / ispravne grane za ROS2:
- `omni_base_simulation`
- `ros2_kortex`
- `pan_tilt_ros`
- `realsense-ros`
- `aruco_ros`

## Kreirani Paketi (src/)
- `dual_arm_torso` (ament_cmake) - služi za spremanje mesh datoteka linearnih vodilica
- `pas_dual_arm_bringup` (ament_cmake) - služi za launch skripte, rviz, moveit i centralni URDF
- `pas_dual_arm_scripts` (ament_python) - kreiran prema zahtjevu zadatka, služi za glavnu Python logiku zadatka

## Nedostajuće Ovisnosti (Sistemski paketi)
Zbog nedostatka sudo ovlasti kod automatske agent skripte, korisnik mora ručno pokrenuti sljedeće naredbe kako bi instalirao potrebne alate:
```bash
sudo apt-get update
sudo apt-get install -y \
  ignition-fortress \
  ros-humble-ros-gz \
  ros-humble-ign-ros2-control \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-ros2-controllers
```
Nakon toga je potrebno riješiti ovisnosti repozitorija:
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Sljedeći Koraci
Pročitati `README.md` i slijediti smjernice iz **Faze 2**, tj. započeti s pisanjem integracijskog URDF-a u paketu `pas_dual_arm_bringup`.
