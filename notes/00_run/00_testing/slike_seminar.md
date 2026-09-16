---
id: TEST_SLIKE_SEMINAR
type: upute
updated: 2026-09-16
---
# Slike za seminar — što snimiti, kada i kako

> Popis snimaka za pisani seminar ([[R-21_deliverables]]). Sve osim S3 i S12 dolazi iz **dva
> runa** `mission.launch.py`: run A s navigacijskim pogledom, run B s pogledom na kocku.
> Razlog za dva: `mission.launch.py` prima **jedan** `rviz_config`, a S4/S5 traže `nav2.rviz`
> dok S6 traži `cube.rviz`. Otvaranje drugog RViza usred runa radi, ali otima CPU
> `gz_ros2_control` petlji ([[P-32_gui_starves_control]]) — ne isplati se.
>
> **Nikad headless** — za slike treba GUI.

---

## Prije snimanja

```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
bash scripts/clean_ros.sh
./scripts/run_native.sh colcon build --symlink-install
./scripts/run_native.sh python3 scripts/check_doors.py
./scripts/run_native.sh python3 scripts/check_zones.py
```

Oba `check_*` moraju dati `PASS`. Snimke padaju u `~/Pictures/Screenshots/` s vremenskom
oznakom u imenu — **zapiši vrijeme uz oznaku snimke** (S1, S2 …) dok snimaš, inače se poslije
ne razaznaje što je što (119 datoteka u tom direktoriju već sad).

### Postavke koje se isplati napraviti jednom

| Gdje | Što | Zašto |
|---|---|---|
| RViz | ugasi **Grid** | mreža na ispisu u sivim tonovima zbunjuje |
| RViz | `Path` → `Line Width` **0.05** | zadana linija nestane pri smanjenju na širinu stupca |
| RViz | `Map` → `Color Scheme` = `map` | `costmap` shema je šarena i nečitljiva crno-bijelo |
| Gazebo | zatvori lijevu i desnu ploču (`Entity tree`, `Component inspector`) | robot dobiva cijeli prozor |
| Gazebo | `View` → `Orthographic` za S3 | tlocrt bez perspektivnog izobličenja |
| oboje | prozor na **cijeli zaslon** prije snimanja | slike moraju biti čitljive na ~15 cm širine |

---

## Run A — navigacija (S1, S2, S3, S4, S5, S8, S9, S10)

```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
bash scripts/clean_ros.sh
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mission.launch.py |& tee log/slike-A.log
```

| # | Slika | Trenutak (znak u logu / Gazebu) | Kadar |
|---|---|---|---|
| **S1** | Robot s raširenim rukama | odmah po spawnu, **prije** nego ruke krenu | Gazebo, robot s prednje strane, cijeli u kadru — pokazuje `ARM_ZERO` spawn pozu |
| **S2** | Poza vožnje `DRIVE_V4` | log: `mission: drive posture ... verified` / `WAITING for the user` | Gazebo, robot **bočno**, da se vidi kako su ruke skupljene uz torzo (0.821 m) |
| **S3** | Tlocrt svijeta | prije pritiska na gumb, dok robot stoji | Gazebo odozgo, ortografski, **sve tri sobe u kadru** — siva/plava/crvena, oba prolaza, oba stola, kocka |
| **S4** | Karta + čestice + polje + putanja | tijekom vožnje kroz home sobu prema vratima 0 | RViz `nav2.rviz`: uključi `Map`, `AMCL Pose`, `/keepout_filter_mask_planner`, `Path`, `LaserScan`. Kadar: cijela karta |
| **S5** | Prolaz kroz vrata | u trenutku kad je robot **u** otvoru | RViz, **zumirano na vrata**: vidi se `/robot_footprint` (živi obris) i laserske točke uz oba dovratka. Ovo je slika koja dokazuje 7 cm rezerve |
| **S8** | Nošenje kroz vrata | log: `NAV: leg …` nakon `TASK COMPLETE` | Gazebo, robot s kockom u rukama **u otvoru vrata**, bočno-koso |
| **S9** | Kocka na markeru | log: `PLACE VERIFIED: … mm` | Gazebo **izbliza**, odozgo-koso na ploču stola: kocka na markeru, vidljiv jednak rub markera (~3 cm) sa sve četiri strane |
| **S10** | Upravljački panel | bilo kad dok misija traje | Prozor `nav_gui` sam za sebe (ne cijeli zaslon), s vidljivim živim prikazom koraka |

**Tijek runa A:** kad log javi `WAITING for the user`, snimi S2, pa pritisni zeleni gumb
**„MISIJA: po kutiju"**. Dalje ide samo do `MISSION COMPLETE`.

---

## Run B — hvat izbliza (S6, S7, S11)

```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
bash scripts/clean_ros.sh
RVIZ=$(./scripts/run_native.sh ros2 pkg prefix pas_dual_arm_bringup)/share/pas_dual_arm_bringup/rviz/cube.rviz
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mission.launch.py \
    rviz_config:="$RVIZ" |& tee log/slike-B.log
```

| # | Slika | Trenutak | Kadar |
|---|---|---|---|
| **S6** | Kamere na zapešćima vide markere | log: `STEP5a WRIST CAMERAS: markers 0.300 m apart` | RViz `cube.rviz`: **oba prozora kamera** („Slika – lijeva/desna ruka") s vidljivim markerom u svakom, plus oblak točaka i TF okviri. Cijeli RViz prozor |
| **S7** | Trenutak hvata | log: `STEP5d left tool tip … mm from its target` | Gazebo **izbliza**: obje šake na suprotnim plohama kocke, jastučići na plohi |
| **S11** | MoveIt scena *(neobavezno)* | dok ruke planiraju, prije `STEP5b` | RViz s `PlanningScene`: stol (ploča + noge) i kocka kao kolizijski objekti — dokaz da planer zna za svijet ([[P-23_moveit_scene]]) |

---

## Run C — negativan test (S12, neobavezno ali vrijedno)

Dokaz politike poštenja ([[D-12_honesty_abort_over_fake]]) — jaka slika za poglavlja
„Rezultati" i „Ograničenja". Postupak je M5 iz [[misija]]: makni kocku izvan dohvata
(u Gazebu je povuci mišem dalje od stola) pa pokreni misiju.

| # | Slika | Trenutak | Kadar |
|---|---|---|---|
| **S12** | Pošten abort | kad log javi abort **bez** attacha | Panel `nav_gui` u stanju `ABORTED` + vidljiv razlog; po mogućnosti i terminal s porukom |

---

## Što treba znati

- **Ne popravljati run zbog slike.** Ako nešto ne izgleda kako treba, snimi kako jest i zapiši —
  slika u seminaru mora odgovarati zabilježenom runu ([[D-12_honesty_abort_over_fake]]).
- S5 i S9 su **nosive slike** seminara (prolaz kroz vrata i dokaz odlaganja). Ako se neka ne
  uhvati, run se ponavlja; ostale se mogu i preskočiti.
- Ako se ruke tijekom vožnje rašire (`arms: … off DRIVE_V4`), to je [[P-37_arm_position_gain_sag]] —
  vrati ih sa `set_posture DRIVE_V4` i ponovi dionicu, nemoj snimati raširene ruke kao „pozu vožnje".
- Gazebo `Move To` samo pomiče kameru i **ne pokreće robota** — koristi ga slobodno za kadriranje.

## Zabilježiti

Red u [[runovi]] za run A, B i C (vrijeme, ishod, je li `PLACE VERIFIED` pao) + popis snimljenih
oznaka s vremenima, da se datoteke iz `~/Pictures/Screenshots/` mogu poslije preimenovati.
