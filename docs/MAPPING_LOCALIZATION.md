# Mapiranje, lokalizacija i značajke

## Izvedbeni plan

1. **Mehanika prije SLAM-a.** Pokrenuti simulaciju, složiti ruke u `ARM_CARRY_V2`, pa iz
   `/joint_states` i stvarnih kolizijskih meshova potvrditi da je širina tijekom vožnje
   manja od otvora 1.0 m. MoveIt-ov `SUCCEEDED` sam po sebi nije dokaz. Držati
   kutiju i stolove na postojećoj visini 0.10 m dok se doseg hvata ne izmjeri.
2. **Čist 2D sken.** `/scan` je sirovi Gazebo lidar. `scan_filter` izbacuje povrate
   unutar ovojnice robota i objavljuje `/scan_filtered`; taj topic koriste
   slam_toolbox, Nav2 i sigurnosna provjera ture. Usporediti skenove u RViz-u
   tijekom gibanja, posebno šake i dovratke. Samo filtriranje ne opravdava
   prolaz ako šake fizički strše izvan 1.0 m.
3. **SLAM.** `mapping.launch.py` pokreće `slam_toolbox` s odometrijom i filtriranim
   skenom. `mapping_tour` vozi uz odmjerene okrete i ravne dionice, provjerava
   sken, stanje ruku te promjenu `map→odom`. Odmah prekida na pogrešci; tura nije
   zamjena za test stvarne geometrije u GUI-ju. Tek nakon potpune i konzistentne
   karte sve tri sobe spremiti `.yaml/.pgm` i `.posegraph`. `save_map.sh` provjerava
   slobodnu površinu i raspon karte te odbija očito nepotpune snimke.
4. **Registracija.** `feature_registry` iz `/map` traži praznine između kolinearnih
   zidova širine 0.7–1.3 m; niski stolovi dolaze iz RGB-D horizontalnih klastera
   na z≈0.10 m; kutija je opaženi ArUco ID 0 na prednjoj plohi. JSON topic
   `/semantic_features` daje koordinate u `map` i izvor svakog mjerenja.
   Opažanje treba potvrditi iz više pogleda prije uporabe kao navigacijski cilj.
5. **Lokalizacija i navigacija.** Nakon spremanja karte koristiti AMCL preko
   `nav2.launch.py mode:=localization`; `map_server` čita `.yaml/.pgm`, a AMCL
   objavljuje `map→odom` nakon zadavanja početne poze u RViz-u. Za novu mapu može se koristiti `mode:=mapping`, ali
   ne istodobno s `mapping.launch.py`. Nav2 sada ima statički globalni sloj i
   footprint 1.04×0.854 m. Korisnik zadaje približnu regiju u `task.launch.py`
   (`region_x`, `region_y`, `region_yaw`, `navigate_region:=true`) ili u RViz-u.
   Prije automatskog prolaza provjeriti može li planner uopće naći put kroz
   otvor i je li stvarna širina ruku stabilna. Završni prilaz kutiji ostaje
   postojeći vizualni servo.

## Naredbe

U svakom terminalu koristiti `./scripts/run_native.sh` ili njegov shell.

```bash
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_scripts pas_dual_arm_bringup
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mapping.launch.py
./scripts/run_native.sh ros2 launch pas_dual_arm_moveit_config move_group.launch.py
./scripts/run_native.sh ros2 run pas_dual_arm_scripts mapping_tour
./scripts/save_map.sh seminar_map
```

U sljedećem, čistom startu simulacije, bez istodobnog SLAM čvora:

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup nav2.launch.py mode:=localization
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup task.launch.py navigate_region:=true region_x:=0.0 region_y:=-4.5 region_yaw:=-1.57079632679
```

## Stanje mjerenja 14. 9. 2026.

**Karta je prihvaćena.** Run 44 (teleop, popravljeni `mecanum_drive_controller`, 100 Nm,
`mu1=0.80`, `mu2=0.20`, `ARM_CARRY_V2`) dao je kartu 11.9 × 11.8 m, 102.3 m² slobodno, oba
prolaza od 1.0 m čista, sve četiri noge po stolu razlučene; `check_map.py` prošao. Spremljena
je u `src/pas_dual_arm_bringup/maps/seminar_map.*`.

**Registar značajki je provjeren na toj karti, ne na sintetičkoj.** Karta ima točno devet
povezanih komponenti: osam nogu stola i zidove — bez šuma. Detekcija nalazi oboja vrata s
promašajem centra 1.5 cm, a širinu očitava kao **0.95 m** (stvarnih 1.0 m; SLAM zadeblja zid
~2.5 cm po strani). Zone se zato grade na izmjerenih 0.95 m.

**Navigacija je prebačena na zone iz detektiranih značajki**
([[D-16_zones_from_detected_features]], razlog [[P-39_nav2_enters_doorway_at_an_angle]]).
Headless je provjereno: 8/8 kontrolera, AMCL lokaliziran, `KeepoutFilter` aktivan na globalnom
**i** lokalnom costmapu, 14/14 sondi costmapa točno, a planirane putanje sijeku prag vrata pod
0.00–1.07° i 2.5 cm od osi.

**Nijedan metar još nije odvožen.** Sljedeće je GUI vožnja praznog robota u `ARM_CARRY_V2`:
ručni cilj iz RViz-a, pa tipke u `scripts/nav_gui.py`. Otvoreno ostaje
[[P-37_arm_position_gain_sag]] — ruke su nakon gibanja bile izmjerene na 1.109 m, šire od
otvora, pa `room_navigator` prije svakog prolaza provjerava pozu i odbija voziti ako ne drži.
Ne podizati stolove samo radi lidara: to mijenja visinu kutije i neprovjereni hvat.
