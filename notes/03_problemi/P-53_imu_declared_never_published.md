---
id: P-53
type: problem
status: rijeseno
verified: "Izmjereno 18. 9. 2026., `ign topic -l` i `validation/evaluate_imu_yaw.py`."
updated: 2026-09-18
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
solutions: ["[[S-06_navigation]]"]
decisions: ["[[D-25_laser_odometry_like_pal]]", "[[D-18_measure_do_not_argue]]"]
---
# P-53: IMU je cijelo vrijeme bio u robotu, ali nije objavljivao ništa

## Simptom
Zakret pri dolasku na cilj drži se **1,3–1,7°** kroz sve serije, neovisno o tome što se
popravlja. Popravci koji su pomogli položaju (`FollowPathAlign`, laserska odometrija) na taj
broj nisu utjecali — medijan `amcl_yaw_max_deg` ostaje 1,3–1,45° u četiri uzastopne serije.

To nije bezopasno. Greška smjera nošena kroz prolaz daje bočni pomak na izlazu:

```
duljina prolaza × tan(greška smjera) = 2,3 m × tan(1,7°) = 68 mm
```

Izmjereni bočni promašaji u vratima: **57, 68 i 72 mm**. Red veličine i predznak se slažu.

## Uzrok
**Potvrđeno.** Baza ima IMU koji nitko nikad nije uključio.

| korak | stanje prije 18. 9. |
|---|---|
| deklaracija senzora | **postoji** — `urdf/base/base_sensors.urdf.xacro:20`, `update_rate="100.0"` |
| PAL-ov makro | `pal_urdf_utils/urdf/interaction_sensors/imu.urdf.xacro`, `<topic>base_imu</topic>` |
| Ignition sustav koji ga vrti | **nedostaje** — `libignition-gazebo-imu-system.so` nije bio u svijetu |
| most prema ROS-u | **nedostaje** — `/base_imu` nije bio u `config/bridge.yaml` |
| pretplatnik | nema ga |

Svijet je učitavao `sensors`, `contact` i `forcetorque` sustav, ali ne i `imu`. Senzor je zato
postojao u opisu robota i nije postojao u simulaciji: `ign topic -l` ga nije prikazivao.

Zašto to nije prije uočeno: `gz_ros2_control` je uredno javljao
`IMU sensor 'base_imu_sensor' not found in hardware_info`, što je pročitano kao bezazlena
poruka o nekorištenom senzoru — a bila je točan opis stanja.

## Zašto je baš zakret važan
Ni jedan izvor koji smo imali **ne mjeri** smjer, nego ga zaključuje:

| izvor | kako dobiva zakret | čemu vjeruje |
|---|---|---|
| kotači | iz četiri brzine kotača | da nema proklizavanja — a mecanum baza kliže bočno (P-52: 19,5 mm) |
| laser | iz geometrije dvaju skenova | da je scena kruta i da je podudaranje konvergiralo |
| AMCL | iz čestica na karti | da karta odgovara stvarnosti |
| **IMU** | **žiroskop mjeri brzinu zakreta izravno** | vlastitom pomaku nule |

## Lijek
1. `worlds/seminar_world.sdf` — dodan `ignition::gazebo::systems::Imu`.
   `gen_world.py` uzima taj svijet kao predložak, pa generirani svjetovi to nasljeđuju.
2. `config/bridge.yaml` — dodan `/base_imu`, `ignition.msgs.IMU` → `sensor_msgs/msg/Imu`.
3. `validation/run_batch.py` — `/base_imu` i `/laser_odom` se snimaju u bag svakog runa.

Provjera da radi, uživo: `ign topic -l | grep imu` → `/base_imu`.

## Kako je izmjeren
Misija nije mjerilo za smjer — traje deset minuta, pada u trećini slučajeva i većinu vremena
stoji. Umjesto toga `scripts/yaw_drive.py` vozi otvorenom petljom kratak niz koji sadrži točno
one pokrete iz kojih greška smjera nastaje (ravno 2 m, zaokret, bočno 0,8 m, sve odjednom), a
`validation/evaluate_imu_yaw.py` sve procjenitelje uspoređuje s Gazebovom istinom iz istog baga.

Svaka procjena se nulira na vlastitom prvom uzorku — odometriju se nikad ne pita za apsolutni
smjer, pa bi je bilo nepošteno tako i ocjenjivati.

Rezultat, tri čista runa protiv Gazebove istine — srednja apsolutna greška zakreta:

| izvor | sred. | na kraju |
|---|---|---|
| **IMU (žiroskop)** | **0,02–0,03°** | 0,00–0,03° |
| laser/fuzija | 0,06° | 0,01–0,03° |
| kotači | 7,3–7,5° | **34,4–34,7°** |

Kotači nisu malo lošiji nego IMU — lošiji su za dva reda veličine, i to je greška koja
**raste s prijeđenim putem**, jer se svaki sljedeći pomak zakrene za nju.

Što s tim brojem činimo i zašto dobitak u misiji **nije** izmjeren: [[D-26_imu_yaw_in_the_fusion]].

## Mjerenje je dvaput bilo krivo prije nego je bilo točno
Zapisano jer su obje greške tihe i obje bi prošle kao nalaz:

1. **Integracija po vremenu baga.** Bag pamti kad je poruka primljena na zidnom satu, a
   simulacija ide na pola realnog vremena. Brzina mjerena u simuliranim sekundama, integrirana
   po zidnim, naraste za 1/RTF: žiroskop je ispao 42° promašen dok je isti senzor svojom
   orijentacijom davao 0,02°. Neslaganje dvaju puteva do istog broja je i otkrilo grešku.
2. **Sedam živih `laser_odometry` čvorova.** `clean_ros.sh` ga nije gasio, pa se gomilao kroz
   pokušaje i svi su objavljivali na `/laser_odom` — 247 Hz umjesto 50. Mjerenje je davalo 34°
   i 3,1 m ondje gdje čist okoliš daje 0,06° i 1,8 mm. Uzorak je dodan u `clean_ros.sh`.
