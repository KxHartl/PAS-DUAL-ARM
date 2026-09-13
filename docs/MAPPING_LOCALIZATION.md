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

## Stanje mjerenja 13. 9. 2026.

Kod i launch datoteke se grade. Sirovi `/scan` i `/scan_filtered` objavljuju, a
registar značajki radi na sintetičkoj karti s vratima od 1.0 m. **Nema prihvaćene
karte ni potvrđene navigacije.** Pokus s čistom simulacijom postigao je odometrijski
okret −89.9°, ali ravna dionica zadana kao 4.5 m ostvarila je samo 1.89 m, uz
promjenu `map→odom` 1.04 m. Izmjerena stvarna širina ruku nakon gibanja bila je
**1.109 m**, dakle robot fizički ne može proći kroz vrata od 1.0 m u sadašnjem
stanju. Tura je zato zaustavljena, a nepotpuna karta spremljena samo kao
dijagnostička kopija u `/tmp/pas-map-diagnostic/`.

Sljedeći tehnički preduvjet je dinamički stabilna putna poza ili fizički ispravan
upravljani model ruku, pa zatim provjera stvarnog pogona ravno naprijed. Ne
podizati stolove samo radi lidara: to mijenja visinu kutije i neprovjereni hvat.
