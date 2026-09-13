# Kako pokrenuti simulaciju i zadatak

## 0. Projektno okruženje (jednom po terminalu)
```bash
cd ~/FSB/PAS-DUAL-ARM
./scripts/run_native.sh
```
Ova naredba otvara izolirani PAS-DUAL-ARM shell. Svaki terminal koji sudjeluje u istom ROS grafu
otvorite na isti način. Za prijelaz u drugi ROS projekt prvo izađite naredbom `exit`.

Ako si mijenjao kod (Python čvorove) ili xacro/URDF/world/config fajlove, prvo rebuild:
```bash
./scripts/run_native.sh colcon build --symlink-install
```
(Za brzi rebuild samo jednog paketa:
`./scripts/run_native.sh colcon build --packages-select <ime_paketa>`.)

## 1. Pokreni simulaciju (Gazebo + ros2_control + senzori)
```bash
ros2 launch pas_dual_arm_bringup sim.launch.py
```
- Otvara GUI Gazebo Ignition/Fortress prozor s robotom u `seminar_world.sdf`.
- Pričekaj dok se svih ~8 kontrolera ne aktivira (u logu vidiš `activate` poruke)
  i dok se Gazebo prozor potpuno ne učita prije sljedećeg koraka.
- Headless varijanta (bez GUI-ja, koristi se ako GUI izgladnjuje
  `gz_ros2_control` petlju na opterećenom stroju): `sim.launch.py headless:=true`.

## 2. Pokreni zadatak (autonomni find→approach→grasp→lift→place)
U novom terminalu:
```bash
cd ~/FSB/PAS-DUAL-ARM
./scripts/run_native.sh
ros2 launch pas_dual_arm_bringup task.launch.py
```
Ovo pokreće `main_task.py` state machine: SCAN → vizualni prilaz → mjerenje
dubinom → dvoručni top-down hvat → contact-check → attach → podizanje →
spuštanje na stol.

## 3. Provjera contact senzora (ako testiraš novi kontaktom-verificirani hvat)
U trećem terminalu, dok task radi, provjeri javljaju li se topici prije nego
robot dođe do kutije:
```bash
ros2 topic echo /contact/left_left_tip --once
```
Ako topic šuti (timeout), ne prekidaj test — hvat ima fallback na depth+stall
provjeru. Ali zapiši to jer znači da treba ispraviti ime kolizije u `tip_contact`
makrou (`robot.urdf.xacro`) — trenutno pretpostavlja `<link>_collision` kao
zadano sdformat ime.

## 4. Što pratiti u logu main_task.py
- **Pozitivan tijek**: izmjereni centar+yaw kutije (ne izmišljen x=0.55), zatim
  `EE ... REACHED` za obje ruke, `Grasp evidence: placement=True physical=...`,
  pa `Box ATTACHED`, i kutija se stvarno digne u Gazebo prozoru.
- **Negativan test** (makni kutiju izvan dohvata u SDF-u prije pokretanja):
  task mora ABORTIRATI s porukom tipa "did not reach" / "no real contact" —
  NE smije fake-attachati kutiju izdaleka.

## Poznati problemi / gotchas
- **colcon upozorenje `realsense2_description` override**: `colcon build` javlja da paket
  `realsense2_description` iz `src/realsense-ros/` nadjačava istoimeni paket iz `/opt/ros/humble`.
  Ovo je **namjerno**: cijeli `realsense-ros` je dohvaćen iz izvora (v4.57.6, noviji od apt verzije u
  Humbleu) radi usklađenog drivera i opisa kamere, pa override mora ostati. Upozorenje je benigno i ne
  treba ga uklanjati; ako želiš tišu konzolu, ne briši paket iz source seta (izgubila bi se verzija
  usklađena s driverom).
- **`gazebo_version` xacro greška** (`name 'gazebo_version' is not defined`):
  ispravljeno u `robot.urdf.xacro` — dodano svojstvo `gazebo_version=gazebo`
  jer namjerno preskačemo `omni_base_description/robots/omni_base.urdf.xacro`
  (taj bi vukao Gazebo Classic ros2_control koji ign_ros2_control ne učita).
  Ako se ponovno pojavi nakon `apt upgrade` paketa `ros-humble-pal-urdf-utils`,
  provjeri je li svojstvo i dalje definirano prije `base_sensors.urdf.xacro`
  include-a.
- **Middleware i domena**: `scripts/run_native.sh` postavlja Fast DDS, ROS domenu 5 i lokalno
  otkrivanje čvorova. Globalni `~/.bashrc` namjerno ne postavlja ROS varijable. Ako je potreban
  mrežni ili hardverski profil, definiraj ga zasebno; ne postavljaj router ili adresu robota
  globalno.
- **Transport do zasebnog stola ne radi**: DART DetachableJoint + gibanje baze
  izbaci kutiju iz hvataljki. Trenutno testirano samo pick+lift+place na ISTI
  stol. Transport-proba (`probe_transport`) je napisana, ali `task.launch.py` taj
  parametar još ne prosljeđuje → `notes/03_problemi/P-18_transport_drops_box.md`.
- **Kontaktom-verificirani hvat je provjeren u GUI-ju** (16. 7., 3 puna ciklusa
  kocke); zadnje izmjene gatea/odlaganja (`73617e8`) još čekaju run →
  `notes/03_problemi/P-28_gate_too_strict.md`. Stanje svih zahtjeva: `notes/00_MAPA.md`.
