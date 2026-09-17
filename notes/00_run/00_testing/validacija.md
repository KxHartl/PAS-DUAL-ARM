---
id: TEST_VALIDACIJA
type: upute
updated: 2026-09-18
---
# Ponavljanje misije i mjerenje (ispitni sklop)

> Zatvara tri profesorova komentara odjednom: tvrdnju o **0,835 m**, **statistiku uspješnosti** i
> **jačinu izraza** ([[danas]]). Svaki run ostavlja log (*što je robot tvrdio*) i rosbag sa
> ground truthom (*što se stvarno dogodilo*) → [[D-24_measure_from_recordings_not_logs]].
>
> Pravilo: **prvo 3 runa, pa tek onda 20.** Ako prva tri padnu na istoj fazi, nemamo statistiku
> nego kvar, i šest sati vožnje ne bi ništa dodalo.

---

## V0 — Priprema (jednom)

```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
bash scripts/clean_ros.sh
./scripts/run_native.sh colcon build --symlink-install \
    --packages-select pas_dual_arm_scripts pas_dual_arm_bringup
```

Provjera da su prolaz za ground truth i scenarij na mjestu (mora ispisati `debug_truth`):

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup scenario_mission.launch.py --show-args \
    | grep -A2 debug_truth
```

**Ništa drugo ROS-ovo ne smije raditi.** Sklop pokreće `clean_ros.sh` prije i poslije svakog runa,
a ono ubija sve po uzorku imena (`gz sim`, `nav2_*`, `rviz2`, `amcl`, `ros_gz_bridge`…) i radi na
domeni 5. Otvoreni Gazebo iz druge sesije bit će ugašen.

---

## V1 — Tri runa (prvi put uvijek ovo)

```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
./scripts/run_native.sh python3 validation/run_batch.py --n 3
```

Što se događa po runu (~5–20 min): generira se vlastiti svijet s kockom pomaknutom do ±3 cm →
`clean_ros.sh` → misija bez GUI-ja i RViz-a, s ground truthom → čeka `WAITING for the user`
(najviše 240 s) → čeka **8/8 kontrolera** i transformaciju `map → base_footprint` (svaka najviše
45 s, +10 s) → objavi `/mission/start` →
čeka `MISSION COMPLETE` ili `Task aborted during:` (najviše 900 s) → ugasi snimanje, pa stack.

Prati uživo iz drugog terminala:

```bash
tail -f validation/results/latest/run_001/run.log
```

**Dovoljno dobro za nastavak:** barem jedan run `success`, a neuspjesi se ne ponavljaju u istoj
fazi. Ishod `not_ready` znači da se stack nije digao (npr. 7/8 kontrolera) — to nije neuspjeh
misije i ne ulazi u uspješnost ([[P-50_base_controller_missing_no_localisation]]). Inače: stani, pročitaj `last_phase` i `reason` iz `results.csv`, pa u [[03_problemi]].

---

## V2 — Mjerenje iz snimki

```bash
./scripts/run_native.sh python3 validation/analyze_runs.py
./scripts/run_native.sh python3 validation/summarize.py --latex --chart
```

`analyze_runs.py` mjeri svaki bag protiv svijeta **tog** runa i piše `metrics.csv` (+ `metrics.json`
po runu). Na kraju ispisuje izravan odgovor na profesorov komentar:

```
The report states 0.835 m from the table edge in open room space.
Driven, crossing a room: min X.XXX m, median X.XXX m, over N run(s).
```

`summarize.py` daje uspješnost, gdje su neuspjesi stali i medijan s rasponom; `--latex` piše
`results_table.tex` za izravno uključivanje u seminar.

---

## V3 — Puna serija

```bash
./scripts/run_native.sh python3 validation/run_batch.py --n 20
```

Nova serija = nova mapa s vremenskom oznakom. Dodavanje u postojeću:
`--batch 2026-09-18_0132` (numeracija se nastavlja).

**Trajanje:** gornja ograda po runu je ~20 min, pa 20 runova u najgorem slučaju traje preko 6 h.
**Prostor:** bag je nekoliko stotina MB po runu → serija od 20 mjeri se u gigabajtima.
`--no-bag` gasi snimanje (i mogućnost naknadnog mjerenja), `--stock-world` vozi na
`seminar_world.sdf` — kontrolni slučaj ako posumnjamo na generator svijeta.

---

## Gdje sve završi

```
validation/results/
    latest -> 2026-09-18_0132/
    2026-09-18_0132/
        results.csv  metrics.csv  results_table.tex  results.png
        run_001/  run.log  world.sdf  bag/  metrics.json
```

Ništa od toga ne ide u git (`validation/results/` je u `.gitignore`); u seminar idu brojke iz
`results.csv` i `metrics.csv`, ne prepričani pojedinačni runovi.

> [!warning] Prekid serije ne gasi simulaciju
> `run_batch.py` pokreće launch u **vlastitoj sesiji**, pa Ctrl-C stigne samo do skripte, a Gazebo
> i svi čvorovi ostaju živi. Poslije svakog prekida: `bash scripts/clean_ros.sh`, pa provjeri
> `pgrep -af "ign gazebo|gz sim"` (mora biti prazno) prije nove serije ([[P-50_base_controller_missing_no_localisation]]).

## Poznati rizik prije prvog pokretanja
[[P-47_headless_batch_map_odom_stale]]: jedini dosad odvoženi run sklopa pao je s
`room_navigator did not reach blue:dock` jer je `map → odom` bio ustajao. Sklop sad čeka da se
transformacija uspostavi (`wait_for_tf`), ali **to čekanje još nije provjereno u vožnji**. Padne li
V1 na prvoj dionici bez pomaka robota, to je i dalje P-47, a ne mjera sustava.
