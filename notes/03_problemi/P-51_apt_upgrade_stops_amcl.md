---
id: P-51
type: problem
status: rijeseno
verified: "A/B 18. 9.: stablo od 16. 9. (d594fd8) pada isto kao main; nakon vraćanja stacka main vozi `MISSION COMPLETE`"
updated: 2026-09-18
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]", "[[R-20_place_at_destination]]"]
solutions: ["[[S-10_build_run_environment]]", "[[S-06_navigation]]"]
decisions: ["[[D-11_project_scoped_ros_env]]"]
---
# P-51: `apt` nadogradnja od 17. 9. zaustavlja AMCL usred vožnje

> [!important] Ovo je uzrok, a [[P-47_headless_batch_map_odom_stale]] je bio simptom
> Sve što je od 17. 9. navečer izgledalo kao regresija našeg koda — pad ispitnog sklopa, pad
> automatskog mapiranja, pad misije na trećoj dionici — ima jedan uzrok izvan repozitorija.

## Simptom
Misija pada na dionici do stola, uvijek isto:

```
[controller_server] [tf_help]: Transform data too old when converting from map to odom
                               Data time: 55s 827ms, Transform time: 55s 380ms
[planner_server]  GridBased failed to generate a valid path to (0.01, -5.15)   (14×)
[bt_navigator]    Goal failed → room_navigator: Nav2 status 6
```

`map → odom` se zamrzne na jednom žigu i više se ne osvježava, a Nav2 zatim odbija svaki cilj.

## Uzrok
**Potvrđeno.** `/var/log/apt/history.log`, **17. 9. 2026. u 18:09**, `aptdaemon` (neinteraktivno,
bez pitanja): zamijenjeno je **439 `ros-humble-*` paketa** — `rclcpp`, `tf2`, cijeli `nav2`,
`ros2_control`/`controller_manager`, `ros_gz_bridge`/`ros_gz_sim`, `slam_toolbox`, MoveIt, RViz,
`rmw_fastrtps`. To je jedini događaj između „radilo je 16. 9. do ~14 h" i „pada od 17. 9. navečer".

**Što točno stane (izmjereno uživo dok je robot stajao u vratima):**

| tema | stanje |
|---|---|
| `/scan` | 13.6 Hz — živ |
| `/scan_filtered` | 13.3 Hz — živ |
| `/base_controller/odom` | 56 Hz — živ |
| `/clock` | 459 Hz — živ |
| **`/amcl_pose`** | **tišina** |

AMCL je živ kao proces (11.5 % jezgre, pretplaćen, tema ima 5 pretplatnika) ali mu se **callback
skena prestaje okidati**. `nav2_amcl` objavljenu transformaciju žigoše s
`last_laser_received_ts_ + transform_tolerance`, pa kad ta oznaka stane, stane i `map → odom` —
iako tajmer objave i dalje radi. Zato se u logu vidi zamrznut `Transform time` uz `Data time` koji
raste.

## A/B koji je to dokazao
Isti stroj, ista karta, isti svijet, ista naredba — mijenjano je **samo** stablo koda i stack:

| mjera | `main` (noćni kod) | **stablo 16. 9. (`d594fd8`)** | `main` nakon vraćanja stacka |
|---|---|---|---|
| `Transform data too old` | 23 | **94** | **0** |
| padova planera | 14 | **14** | **0** |
| `missed its desired rate` | 307 | 312 | 734 |
| ishod | `PREKID [3/8]` | **`PREKID [3/8]`, ista poruka** | **`MISSION COMPLETE`, 5 mm od markera** |

Stablo od 16. 9. je ono s kojim je snimljen video i koje je radilo i kolegi. **Pada identično** →
krivac nije kod. Logovi: `log/AB-run-A-16-09.log`, `log/AB-run-B-main-vraceni-stack.log`.

## Rješenje
Vraćanje cijelog ROS stacka na dated snapshot **`2026-08-07`**, koji sadrži **točno** verzije koje
su bile instalirane prije 17. 9. (provjereno: 0 neslaganja na svih 543 paketa).

```bash
# 1. snimi stanje prije zahvata
dpkg -l 'ros-humble-*' | awk '/^[hi]i/{print $2"  "$3}' > log/dpkg-ros-prije.txt

# 2. snapshot izvor (ključ je "ROS Snapshot builder <rosbuild@ros.org>", AD19BAB3CBF125EA,
#    NIJE isti kao ključ packages.ros.org; instalira se samo za taj izvor)
sudo tee /etc/apt/sources.list.d/ros2-snapshot.sources <<'SRC'
Types: deb
URIs: http://snapshots.ros.org/humble/2026-08-07/ubuntu
Suites: jammy
Components: main
Signed-By: /usr/share/keyrings/ros-snapshot.gpg
SRC
sudo mv /etc/apt/sources.list.d/ros2.sources /root/ros-rollback-backup-2026-09-18/
sudo apt update

# 3. vrati točno te verzije (0 uklanjanja; kod nas 439 paketa)
sudo apt-get install -y --allow-downgrades $(cat popis-paket=verzija)

# 4. da se ne ponovi
sudo apt-mark hold $(dpkg -l 'ros-humble-*' | awk '/^[hi]i/{print $2}')

# 5. OBAVEZNO: čist rebuild, jer je workspace u međuvremenu gradjen na novom stacku
rm -rf build install && ./scripts/run_native.sh colcon build --symlink-install
```

**Stanje nakon zahvata (18. 9.):** 543 paketa na `hold`, živi ROS repo isključen, sve staro u
`/root/ros-rollback-backup-2026-09-18/`, build 25/25, `verify_environment` 20/20.

## Pokušaji
| # | datum | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 17. 9. | traženje uzroka u našem kodu (watchdog, `cmd_vel_relay` na sim vrijeme, lagani most) | kvar ostaje | vidi [[P-47_headless_batch_map_odom_stale]] #2–4 |
| 2 | 18. 9. | čist rebuild cijelog workspacea (25/25) i `verify_environment` (20/20) | kvar ostaje | nije zastarjeli build |
| 3 | 18. 9. | **A/B: stablo od 16. 9. u istom okolišu** | **pada identično** (tablica gore) | **kod nije krivac** |
| 4 | 18. 9. | mjerenje tema uživo tijekom kvara | skenovi teku, `/amcl_pose` šuti | stane **AMCL**, ne dotok skenova |
| 5 | 18. 9. | vraćanje 439 paketa na snapshot 2026-08-07 + `hold` + čist rebuild | ✅ **`MISSION COMPLETE`, 5 mm od centra markera** | **rješenje** |

## Ne ponavljati
- **Ne tražiti uzrok u kodu prije nego se provjeri `/var/log/apt/history.log`.** Noć 17→18. 9.
  potrošena je na popravke koda za kvar koji kod nije uzrokovao.
- Ne zaključivati iz „tema ima pretplatnike i publisher objavljuje" da potrošač prima. Ovdje je
  `/scan_filtered` tekao 13 Hz, a AMCL ga nije obrađivao.
- Ne dirati stack bez `dpkg -l` snimke prije zahvata — bez nje se ne zna na što se vraća.

## Sljedeći korak
Otvoreno pitanje za poslije predaje: **koja** od 439 promjena zaustavlja AMCL (`nav2-amcl` sam,
`rclcpp` izvršitelj ili `rmw_fastrtps`). Za predaju nije potrebno — stack je zaključan i misija
vozi. Ako se bude tražilo, bisekcija po grupama paketa na snapshotima između 2026-08-07 i
2026-09-08.
