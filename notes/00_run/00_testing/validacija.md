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

## V0b — GUI run (kad se želi vidjeti kako radi)

Isto što je vožено 16. 9. (run M4) i 17. 9. ujutro. Jedan terminal:

```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
bash scripts/clean_ros.sh
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mission.launch.py
```

Otvore se **Gazebo** (svijet i robot), **RViz** (karta, costmap, putanja) i **navigacijski panel**
s gumbom „MISIJA: po kutiju". Robot se stvori raširenih ruku, sam ih složi u `DRIVE_V4` i stane
čekati. Pritisni gumb.

Gledati, redom:
- **terminal** — koraci `[1/8]` … `[8/8]`, pa `PLACE VERIFIED` i `MISSION COMPLETE`;
- **Gazebo, dolje desno: `Real time factor`** — ispod ~0,5 lokalizacija zaostaje za satom i run je
  krhak bez obzira na kod (18. 9.: pri 0,27 Nav2 je dionicu prijavio kao stignutu bez pomaka);
- **RViz** — zelena putanja kroz vrata, robot na karti;
- **panel** — koja je faza u tijeku.

Od 18. 9. rani pritisak gumba više ne škodi: `main_task` čeka da `map → base_footprint` bude svjež
i stabilan 2 s i to ispiše (`localised: … driving`), a `room_navigator` ne prihvaća „stigao sam"
dalje od 30 cm od cilja ([[P-49_empty_goal_checker_id_with_two_checkers]],
[[P-50_base_controller_missing_no_localisation]]).

**Prije pokretanja stroj mora biti miran** — `bash scripts/clean_ros.sh` gasi i zaostale čvorove
projekta (detektore, `move_group`, `loc_error`), koji su 18. 9. ostajali živi i rušili RTF.

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

## Stanje sklopa (18. 9., nakon probnih runova)

Sklop je **odvožen** i radi: generira svijet, čeka stack, sam „pritisne gumb", klasificira ishod,
piše `results.csv` i snima bag. Dva nalaza iz probe:

| # | Nalaz | Status |
|---|---|---|
| 1 | [[P-47_headless_batch_map_odom_stale]] — `map → odom` zastari | **riješeno**: uzrok je bio `apt` ([[P-51_apt_upgrade_stops_amcl]]). U probnim runovima **0 ×** `Transform data too old` |
| 2 | **Generirani svijet gubi teksture markera** | **riješeno** 18. 9. u `scripts/gen_world.py` |

### Nalaz 2 — zašto je vrijedan pažnje
Prvi probni run pao je s `scan: marker not found`, što izgleda kao kvar percepcije. Nije bio.
`gen_world.py` kopira `seminar_world.sdf`, a u njemu su putanje tekstura **relativne**
(`materials/textures/aruco_marker_0.png`). Generirani svijet se piše u
`validation/results/<serija>/run_00N/`, gdje `materials/` ne postoji — Ignition tiho ne učita
teksturu i ArUco ploča ostane prazna. Ništa ne javi grešku: lidar radi, robot prođe oboja vrata, i
run pukne tek 3 minute kasnije na skeniranju.

Popravak: `gen_world.py` sada svaku relativnu putanju do resursa pretvara u apsolutnu (prema mapi
predloška). Provjereno u drugom probnom runu: `SCAN: marker found at base_link (0.78, 0.07)`.

> **Pouka za sklop:** svaki run na **generiranom** svijetu treba proći kroz percepciju prije nego se
> pokrene serija od 20. Vožnja i vrata ne dokazuju da svijet ima teksture.

### Prvi ispravan run kroz sklop (18. 9., `smoke2-18-09`)
Headless, generirani svijet, cijela misija: **`success`, 546 s, 8/8**, robot javlja
`PLACE VERIFIED: 4 mm`. Lanac mjerenja iz baga također radi (`analyze_runs.py`, 28 373 uzorka
ground trutha). Prva dva nalaza, oba važna za seminar:

| veličina | vrijednost (n=1) | što znači |
|---|---|---|
| `table_clear_transit_min_m` | **0.686 m** | seminar navodi **0,835 m**. Izmjereno u vožnji je **manje**. Tvrdnju treba prepisati na izmjereno, ne na offline izračun |
| `place_err_truth_mm` | **19.1 mm** | robot je za isti run tvrdio **4 mm**. Ground truth kaže 19.1 |
| `amcl_err_max_cm` / `rms` / `yaw` | 4.4 cm / 1.8 cm / 1.4° | greška lokalizacije kroz cijeli run |
| `door_clear_min_m` | 0.471 m | najtješnji prolaz kroz vrata |

> [!important] Robotova tvrdnja nije mjerenje
> `PLACE VERIFIED` (4 mm) i ground truth (19.1 mm) razilaze se za ~15 mm na istom runu. To je točno
> ono zbog čega postoji [[D-24_measure_from_recordings_not_logs]]. **U seminar ide broj iz
> `metrics.csv`**, a robotov ispis se navodi kao ono što jest — što sustav tvrdi o sebi.
> S n=1 ovo još nije nalaz nego opažanje; serija mu daje raspon.

### Podaci od 17. 9. su nevažeći
Sve što je sklop izmjerio 17. 9. snimljeno je na slomljenom ROS stacku
([[P-51_apt_upgrade_stops_amcl]]) i **ne smije se navoditi**. Serija se pokreće **od nule**.

---

## Redoslijed za punu seriju (preporuka)

1. **V1, tri runa** — mora dati barem jedan `success`, i nijedan pad na percepciji.
2. **V2, mjerenje** — provjeri da `metrics.csv` ima popunjene stupce `place_err_truth_mm` i
   `table_clear_transit_min_m`; ako su prazni, bag nije upotrebljiv i nema smisla voziti 20.
3. **V3, puna serija** — tek onda, i po mogućnosti preko noći (vidi trajanje niže).

**Trajanje, izmjereno 18. 9.:** uspješan run headless traje **~8–10 min** (pad na percepciji je
trajao 3 min). Za 20 runova računaj **3–4 h**, uz gornju ogradu od 15 min po runu (`--run-timeout`).
Stroj u to vrijeme ne smije raditi ništa drugo ROS-ovo.
