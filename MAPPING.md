# Mapiranje od nule (SLAM)

Za demo ovo **ne treba** — karta `src/pas_dual_arm_bringup/maps/seminar_map.yaml` je u repou i
misija je vozi ([`README.md`](README.md) §4). Ovaj dokument je za slučaj da kartu želiš snimiti
sam: izmijenio si svijet, ili želiš proći cijeli SLAM postupak.

Mapiranje ima **vlastiti launch i namjerno ne diže Nav2**: dok se mapira nema se što planirati, a
puni stack se otima oko CPU-a s `gz_ros2_control` petljom (to je izgladnjivalo kontrolere,
`notes/03_problemi/P-32…`).

---

## 1. Prije početka

```bash
cd ~/FSB/PAS-DUAL-ARM
bash scripts/clean_ros.sh        # nikad dvije simulacije odjednom
./scripts/run_native.sh colcon build --symlink-install
```

Svaki terminal ide kroz `./scripts/run_native.sh`. Ne učitavaj drugi ROS workspace u isti shell.

## 2. Terminal 1 — simulacija

```bash
PAS_SIM_CARRY_ARMS=true ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
```

`PAS_SIM_CARRY_ARMS=true` stvara robota **odmah u uskoj pozi `ARM_CARRY_V2`** (0.854 m). To je
bitno: u raširenoj spawn pozi robot je preširok za otvor od 0.98 m i zapinje na dovratku.
Pričekaj da se aktivira **svih osam kontrolera**.

Ako se ruke tijekom vožnje rašire (`P-37`, popuštanje pozicijskog pojačanja), vrati ih:

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_moveit_config move_group.launch.py   # terminal X
./scripts/run_native.sh ros2 run pas_dual_arm_scripts set_posture ARM_CARRY_V2
```

## 3. Terminal 2 — SLAM

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mapping.launch.py
```

Diže `slam_toolbox` (rezolucija **0.02 m**), `feature_registry` i RViz s pogledom za mapiranje.
Za headless: `rviz:=false`. Simulacija već diže `scan_filter` i relay za `/cmd_vel`.

**Prije vožnje provjeri:** `/scan_filtered` dolazi, `/map` postoji, TF lanac
`map → odom → base_link` je cijel.

## 4. Terminal 3 — vožnja

```bash
./scripts/run_native.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Pravila vožnje** (invarijanta trenja kotača `mu1=0.80`, `mu2=0.20`):

- **Zaustavi robota prije skretanja.** Rotiraj u mjestu, pa vozi ravno — nikad u luku.
- Smanji brzine: `x` → linearna na ≈ 0.2 m/s, `c` → kutna na ≈ 0.3 rad/s.
- Kroz vrata idi **okomito** i polako; rezerva je ~7 cm po strani.

**Ruta** (loop closure je ono što drži kartu ravnom):

```
HOME → PLAVA soba (krug po sobi) → natrag u HOME → CRVENA soba (krug) → natrag u HOME
```

U RViz-u prati kako se zidovi zatvaraju. Ako se karta „razdvoji" (duplirani zidovi), prekini i
vozi ponovno sporije — spremanje loše karte samo pomiče problem na Nav2.

### Alternativa: automatska tura

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_moveit_config move_group.launch.py   # terminal 4
./scripts/run_native.sh ros2 run pas_dual_arm_scripts mapping_tour                    # terminal 5
```

Tura provjerava ruke i **namjerno prekida** ako odstupanje pređe 0.10 rad. Zbog uske rezerve u
vratima otvorena petlja odometrije zna zapeti na štoku — **ručna vožnja je pouzdanija**.

## 5. Spremanje karte

Tek nakon potpune i provjerene ture:

```bash
./scripts/save_map.sh moja_tura
```

Skripta:

1. sprema arhivsku kopiju `maps/map_YYYYMMDD_HHMMSS_moja_tura.{yaml,pgm,posegraph,data}`;
2. ažurira **aktivnu** kartu `maps/seminar_map.{yaml,pgm,posegraph,data}` (to je ona koju vozi misija);
3. pokreće `check_map.py` (pokrivenost) i `check_map_geometry.py` (geometrija vrata).

Sve ide u `src/pas_dual_arm_bringup/maps/`, neovisno o tome odakle pokreneš skriptu.

> U git idu samo `.yaml` i `.pgm`. `.posegraph` i `.data` (≈ 44 MB po karti) ostaju lokalni —
> trebaju samo za **nastavak** SLAM-a, ne za vožnju.

## 6. Gate: kad se karta smije prihvatiti

Pokrivenost kaže da su sve tri sobe na karti, ali **ne** kaže jesu li vrata na njoj i dalje
prohodna. Uz ~7 cm rezerve po strani, geometrija je ta koja odlučuje:

| Mjera | Prolazi ako |
|---|---|
| širina oboja vrata | ≥ **0.97 m** (stvarno 1.00 m) |
| os prolaza (uzduž zida) | ≤ **1 cm** od stvarne |
| debljina zida | ≤ **0.14 m** (stvarno 0.10 m) |
| lice zida: RMS / nagib | ≤ **10 mm** / ≤ **1.0°** |
| stepenica između lica s obje strane otvora | ≤ **20 mm** |
| razmak dvaju vrata (mjerilo karte) | ≤ **20 mm** od stvarnih 4.243 m |

Gate se može pustiti i naknadno, bez simulatora:

```bash
./scripts/run_native.sh python3 scripts/check_map_geometry.py
./scripts/run_native.sh python3 scripts/check_map_geometry.py src/pas_dual_arm_bringup/maps/map_2026...yaml
```

**Ako gate padne, karta se ne prihvaća — tura se ponavlja.** Stara karta ostaje u arhivi.
Referentna karta iz repoa: 102.3 m² slobodnog prostora, raspon 11.9 × 11.8 m, vrata 0.980 m.

## 7. Vožnja po vlastitoj karti

```bash
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_bringup
bash scripts/clean_ros.sh
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mission.launch.py
```

Rebuild je obavezan — Nav2 čita kartu iz `install/…/share/pas_dual_arm_bringup/maps/`, ne iz `src/`.

Prije prve vožnje pusti offline provjere zona na novoj karti:

```bash
./scripts/run_native.sh python3 scripts/check_doors.py
./scripts/run_native.sh python3 scripts/check_zones.py
```

Zone (vrata, stolovi, dock poze) izvode se **iz karte**, pa nova karta znači i nove zone. U RViz-u
dodaj `MarkerArray` na `/nav_zones_markers`: zelena traka kroz svaki prolaz, narančasti halo oko
stolova, plave strelice = portalne poze. Ako toga nema, `nav_zones` nije našao vrata — ne voziti.

Detalji i povijest odluka: [`notes/00_run/01_pokretanje.md`](notes/00_run/01_pokretanje.md),
`notes/03_problemi/P-11_nav2_slam_drift.md`.
