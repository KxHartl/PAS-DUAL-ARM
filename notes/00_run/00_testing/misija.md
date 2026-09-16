---
id: TEST_MISIJA
type: upute
updated: 2026-09-16
---
# Misija (konačni sim) — pokretanje i testiranje

> Spaja dva dijela koji su dosad radila **odvojeno**: vožnju po spremljenoj karti
> ([[navigacija]]) i hvat kocke ([[hvat_kocke]], slijed V4). Redoslijed je iz [MAIL]:
> regija → pronađi → podigni → kroz vrata → odloži.
>
> Pravilo: **jedan inkrement = jedan run** ([[D-18_verified_baseline_first]]). Ispod je
> po jedan odjeljak za svaki inkrement; ne preskakati.

---

## M1 — Vožnja u `DRIVE_V4`, prazan robot (bez ijedne izmjene koda)

Zašto prvi: sve dosadašnje vožnje (runovi 46, 62, 70) vožene su u `ARM_CARRY_V2`. Poza vožnje je
od 16. 9. **`DRIVE_V4`** (`postures.ARM_DRIVE`) i u navigaciji **nikad nije provjerena**
([[P-44_grasp_from_reference_pose]], „Otvoreno"). Statički footprint u `nav2_params.yaml` mjeren je
za `ARM_CARRY_V2`; stvarni obris objavljuje `footprint_publisher` ([[D-19_dynamic_footprint]]).

**Terminal 1**
```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
bash scripts/clean_ros.sh
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_scripts pas_dual_arm_bringup
./scripts/run_native.sh python3 scripts/check_doors.py
./scripts/run_native.sh python3 scripts/check_zones.py
PAS_SIM_CARRY_ARMS=true ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
```

**Terminal 2** (čekaj da T1 javi `Configured and activated` za svih osam kontrolera)
```bash
mkdir -p /tmp/pas
cd /home/khartl/FSB/PAS-DUAL-ARM
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup nav2.launch.py 2>&1 | tee /tmp/pas/t2.log
```

**Terminal 3** — ruta M1 (ili tipke na panelu; `blue:dock` nema tipku, ide naredbom)
```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
./scripts/run_native.sh ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: blue:dock}"
./scripts/run_native.sh ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: red}"
./scripts/run_native.sh ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: home}"
```

### Što gledati
| Korak | Očekivano |
|---|---|
| `blue:dock` | prolaz kroz vrata 0 bez aborta, zadnja dionica **dock**, čelo ~10 cm od ploče stola |
| `red` | undock unatrag niz prilaznu os, pa kroz vrata 1, pred crvenim stolom |
| `home` | natrag u sredinu home sobe |
| cijelo vrijeme | bez dodira zida i stola; `arms: DRIVE_V4 held, worst joint …` prije svakog prolaza |

Mjerila se upisuju kao i za navigaciju ([[navigacija]] T5): vrijeme po dionici, `arrived N cm and
±N deg`, redak `alignment:`, najmanji bočni razmak, `footprint now …`.

### Što treba znati
- `DRIVE_V4` je **0.821 m** širok (vrata 0.980 m) → 7.9 cm po strani, slično kao `ARM_CARRY_V2`
  (0.854 m). Ako gate javi `arms: … off DRIVE_V4`, ruke su popustile ([[P-37_arm_position_gain_sag]]):
  `./scripts/run_native.sh ros2 run pas_dual_arm_scripts set_posture DRIVE_V4`.
- Ne dirati `min_side_clearance`, `doorway_margin` ni `ObstacleFootprint.scale` — slijepe ulice
  ([[AGENT_GUIDE]] §5).
- Referentni brojevi za usporedbu: run 46 (plava 33.1 s, crveni stol 60.6 s), run 62 (5/5 prolaza),
  run 70 ([[runovi]]).

### Zabilježiti
Red u [[runovi]] (i kad ne uspije) + red u tablici pokušaja pripadne P-kartice.

---

## M2 — Jedna naredba: vožnja + hvat

Sve iz jednog terminala. Robot se stvara u home sobi **s raširenim rukama** (`carry_arms:=false`,
spawn `ARM_ZERO`), sam ih složi u `DRIVE_V4`, pa **čeka korisnika**.

**Terminal 1**
```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
bash scripts/clean_ros.sh
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_scripts pas_dual_arm_bringup
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mission.launch.py
```

Kad u logu piše `WAITING for the user`, pritisni **„MISIJA: po kutiju"** (zeleni gumb na panelu),
ili iz drugog terminala:
```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
./scripts/run_native.sh ros2 topic pub --once /mission/start std_msgs/String "{data: blue}"
```

### Što gledati (u Gazebu, ne u logu)
| Faza | Očekivano |
|---|---|
| A | ruke se iz raširene spawn poze slože u `DRIVE_V4` **prije** nego se baza pomakne |
| B | robot stoji i čeka; ništa se ne miče samo |
| C | vožnja home → vrata 0 → plava soba → dock (čelo ~10 cm od stola) |
| D | slijed V4 kao dosad: `DETECTION_V4` → primicanje → `GRASP_V4` → vrhovi u markere → attach → dizanje 15 cm → privlačenje → odmak 0.50 m → vodilice 100 mm |

Redoslijed u logu:
```
mission: drive posture ... verified
WAITING for the user: press "MISIJA: po kutiju" ...
MISSION START: going for the cube in "blue"
NAV: asked room_navigator for "blue:dock"
NAV: arrived at "blue:dock"
STEP5a WRIST CAMERAS: markers 0.300 m apart ...
STEP5d left tool tip ... mm from its target
CARRIAGE LIFT MEASURED left=0.5500 right=0.5500 m
TASK COMPLETE: cube lifted and carried away from the table.
```

### Što treba znati
- M2 **staje nakon hvata** (odmak od stola). Nošenje u crvenu sobu je M3, odlaganje M4.
- `mission:=true` ne dira izolirani put hvata: bez njega `main_task` radi točno kao u runovima V2–V5.
- Vožnja ide kroz `room_navigator` (`blue:dock`), **ne** kroz `/navigate_to_pose` — stari
  `navigate_region` nema portalne poze ni dock ([[P-39_nav2_enters_doorway_at_an_angle]]).
- Ako javi `room_navigator is not listening on /room_navigator/goto`, `nav_zones`/`room_navigator`
  nisu digli (provjeri `zones:=true` i `/nav_graph`).

---

## M3 — Nošenje kroz vrata s kockom

Nastavak **istog runa** kao M2: kad hvat javi `TASK COMPLETE`, robot stoji 0.5 m od stola s kockom
u rukama, vodilice na 100 mm. Pritisni **CRVENA soba (pred stol)** na panelu, ili:
```bash
./scripts/run_native.sh ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: red}"
```

### Što gledati
| Korak | Očekivano |
|---|---|
| gate pred vratima | `arms: CARRY_V4 held, worst joint … rad` — **ne** `off DRIVE_V4` |
| prolaz | undock unatrag, pa kroz oboja vrata s kockom u rukama, bez struganja |
| brzina | brzo po sobi, vidljivo sporije u vratima i na prilazu stolu |
| kraj | robot pred crvenim stolom, kocka i dalje u rukama |

### Što treba znati
- Poza nošenja je **`CARRY_V4`** ([[08_poze]]), izmjerena s robota, 0.821 m široka → 8 cm po strani
  u otvoru od 0.980 m. Gate je dobiva na temi `/room_navigator/arm_posture`, koju misija objavi čim
  je kocka privučena ([[P-45_mission_integration]]).
- Ako javi `off CARRY_V4`, ruke su stvarno popustile ili je hvat završio drukčije nego 16. 9. —
  **ne dizati toleranciju**, nego izmjeriti pozu i usporediti s registrom.

---

## M4 — Odlaganje na marker u crvenoj sobi

**Marker postoji od 16. 9.**: crveni X je zamijenjen ArUco pločom 0.40 m (marker id 3, **0.36 m**) na
`place_table`, na (6.32, 0, 0.7515) — vidi se u Gazebu i čita ga glavna kamera
(`/place_marker/pose`, okvir `place_marker_frame`).

**Faza odlaganja je napisana 16. 9.** i vozi se sama, kao nastavak M2/M3 — nakon hvata misija sama
traži `red:dock` i odlaže kutiju. Slijed (`main_task._place_on_marker`):

Redoslijed je korisnikov (16. 9.), korak po korak:

| korak | radnja |
|---|---|
| 1 | dolazak pred stol (`red:dock`) |
| 2 | **zapamti centar markera** — glavna kamera čita ga s docka (ne izbliza, [[P-19_aruco_foreshortening_close]]); pamti se i u `odom`, jer će ga kutija prekriti |
| 3 | ruke i kutija na **visinu markera + 20 cm + pola kutije** (vodilice) |
| 4 | **postolje** se primakne stolu koliko se ono može primaknuti (`place_max_advance`); kutija je još privučena, ruke su u zraku i ne smetaju |
| 5 | **robot** se izjednači s centrom markera — strafe, bez okretanja |
| 6 | **ruke** izjednače kutiju s centrom markera, po x **i** y, jednim ravnim potezom do izračunate točke (`_move_cube_by`) |
| 7 | ruke se spuste na **visinu markera + pola kutije**, 1 cm/s; guard prekida ako kutija napusti jastučiće |
| 7b | **prvo dolje, pa pustiti**: mjeri se donja ploha kutije; ako je > 2 cm iznad ploče, kutija se **ne pušta** nego ostaje u rukama uz pošteni abort |
| 8 | **otpusti kruti spoj**, pa jastučići s plohe (nikad ruka s krute veze, [[P-17_detachable_joint_explodes]]) |
| 9 | ruke u **`DETECTION_V4`** |
| 10 | **odmak** od stola 0.50 m |
| 11 | ruke u **`DRIVE_V4`**, vodilice na 200 mm |
| dokaz | kutija se ponovno izmjeri glavnom kamerom i usporedi s markerom u `odom` → `PLACE VERIFIED: … mm` |

### Što gledati
- Kocka **sjeda**, ne pada i ne gura se po stolu.
- Oko nje ostaje **~3 cm markera sa svake strane**; jednak rub sa sve četiri = centrirano.
- U logu na kraju `PLACE VERIFIED: … mm od centra markera`. Bez tog retka run **nije** uspjeh,
  ma što ostalo pisalo ([[D-12_honesty_abort_over_fake]]).

> [!warning] Ako marker nije viđen
> Misija **ne odlaže kutiju nigdje** — pošteni abort s kutijom u rukama, ne „spusti otprilike tu".

---

## M5 — Negativni testovi (obavezno)

Kocka izvan dohvata → pošten abort bez attacha; marker odlaganja sakriven → abort prije spuštanja
([[D-12_honesty_abort_over_fake]]). Bez njih prolaz ne vrijedi.
