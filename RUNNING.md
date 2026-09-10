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
  stol (vidi STATE.md).
- **Kontaktom-verificirani hvat (redizajn 30.6.) još nije testiran u simu** —
  ovo je prva sesija koja ga stvarno pokreće. Prati STATE.md sekciju
  "ZA TESTIRATI" za točan checklist.
