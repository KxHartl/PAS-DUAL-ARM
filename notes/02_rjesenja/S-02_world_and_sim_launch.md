---
id: S-02
type: rjesenje
status: djelomicno
requirements: ["[[R-07_ros2_humble_fortress_control]]", "[[R-10_mappable_world]]", "[[R-11_door_80cm]]", "[[R-12_box_with_aruco]]", "[[R-13_destination_place]]"]
problems: ["[[P-02_robot_description_yaml_parse]]", "[[P-04_mesh_uri_not_found]]", "[[P-08_marker_not_detected_texture]]", "[[P-12_door_too_narrow]]", "[[P-32_gui_starves_controllers]]"]
decisions: ["[[D-13_three_room_world]]", "[[D-14_light_box_free_size]]", "[[D-10_headless_vs_gui]]"]
files: ["src/pas_dual_arm_bringup/worlds/seminar_world.sdf", "src/pas_dual_arm_bringup/launch/sim.launch.py", "src/pas_dual_arm_bringup/config/bridge.yaml"]
updated: 2026-09-13
---
# S-02: Svijet + pokretanje simulacije

## Što radi
`sim.launch.py` digne Ignition Fortress sa `seminar_world.sdf`, spawna robota na ishodište, pokrene
`ros_gz_bridge` i spawnere za svih 8 kontrolera.

## Svijet (`seminar_world.sdf`, od 13. 9.: tri sobe u L, [[D-13_three_room_world]])
```
        y
        ^   +-----------+-----------+
      2 |   |   HOME    |   CRVENA  |
        |   |  (siva)   |  (odred.) |
      0 |   |  robot→   D   [stol]  |      D = vrata 0.9 m
        |   |           |           |
     -2 |   +-----D-----+-----------+
        |   |  PLAVA    |
        |   | (kutija)  |
     -4 |   |  [stol+K] |
        |   |           |
     -6 |   +-----------+
        +---|-----|-----|-----|-----|--> x
           -2     0     2     4     6
```
| Model | Poza | Bitno |
|---|---|---|
| `rooms` | — | zidovi 0.1 m × **3.0 m** (viši od kamere na 1.39 m, [[P-36_walls_lower_than_camera]]). Sobe **6 × 6 m**. HOME siva, PLAVA plava, CRVENA crvena. Vrata **1.0 m**: zid y = -3 (x ∈ ±0.5) i zid x = 3 (y ∈ ±0.5) |
| `room_floors` | — | obojeni podovi soba, samo vizual (bez kolizije) |
| `pick_table` | (0, -6.5), yaw -π/2 | plava soba; ploča 0.4 × 0.5 na z = 0.10 |
| `aruco_box` | (0, -6.38, 0.25), yaw -π/2 | kocka 0.30 m, **0.3 kg**, μ 5.0, žuta; marker (0.22 / 0.165 m) gleda prema vratima (+y) |
| `place_table` | (6.5, 0) | crvena soba, **odredište**; ploča 0.6 × 0.6 na z = 0.10 |

Ime svijeta mora ostati `seminar_world`, jer ga sadrže bridge topici kontaktnih senzora.

## GUI kamera (`<gui>` u svijetu)
Svijet nosi vlastitu `<gui>` sekciju s **cijelim standardnim setom Fortress plugina** (18 komada,
prepisani iz `/usr/share/ignition/ignition-gazebo6/gui/gui.config`), uz izmjene:
- `MinimalScene.horizontal_fov` **1.05 rad (60°)** — normalni objektiv. Zadanih 90° daje „riblje
  oko“; ispod ~50° slika postaje plosnata i neprirodna (probano 46°, korisnik odbio).
- `MinimalScene.camera_pose` `-6 -6 7 0 0.55 0.55` — pogled na sve tri sobe.
- dodan plugin **`ViewAngle`** („Pogled“): gumbi za gotove poglede (odozgo, sprijeda, sa strane) i
  `Home` koji vraća na gornju pozu. Time korisnik mijenja pogled sam, bez diranja SDF-a.

Ako se `<gui>` doda, gubi se zadana konfiguracija, pa popis plugina mora biti potpun (inače nestanu
alatne trake, Component Inspector, Entity Tree…).

**Upravljanje pogledom mišem u Gazebu:** lijevi gumb + pomak = rotacija oko scene; srednji gumb ili
`Shift` + lijevi = pomak (pan); kotačić ili desni gumb + pomak = zumiranje; dvoklik na objekt =
centriraj na njega.

## Launch
- `IGN_GAZEBO_RESOURCE_PATH` iz svih `AMENT_PREFIX_PATH/share` ([[P-04_mesh_uri_not_found]]).
- `robot_description` omotan u `ParameterValue` ([[P-02_robot_description_yaml_parse]]), a
  negativne skale uklonjene.
- `headless:=true` → samo server (`-s -r`) ([[P-32_gui_starves_controllers]], [[D-10_headless_vs_gui]]).
- Spawneri s `--controller-manager-timeout 120`; robot se spawna na (0, 0, 0).
- RMW forsiran na Fast DDS ([[P-01_shell_zenoh_contamination]]).

## Bridge (`bridge.yaml`)
`/clock`, `/scan`, `/camera/{image,camera_info,depth_image,points}` i 4 kontaktna topica. Duge
gz putanje kontakata mapiraju se na `/contact/<ruka>_<prst>_tip`
([[P-27_contact_sensor_topic_ignored]]).

## Provjereno
| datum | način | što | commit |
|---|---|---|---|
| 12. 6. | headless | 7 kontrolera aktivno, senzori | `7aaab94`, `ae60d9b` |
| 16. 7. | GUI | cijela misija do lifta (stari svijet) | `c504720` |
| 13. 9. | GUI + korisnik | novi svijet s tri sobe potvrđen; `ign sdf -k` valjan; 8/8 kontrolera aktivno | (commit svijeta) |

## Otvoreno
- Prolaz kroz vrata od 0.9 m ([[R-18_door_pass_empty]], [[R-19_door_pass_with_box]]).
