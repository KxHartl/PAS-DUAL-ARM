---
id: RUN_TEST_AUTO_MAP
type: upute
updated: 2026-09-17
---
# Automatsko mapiranje: kako radi, što se gleda, kako se ispituje

Scenarij 1 ([[R-14_slam_mapping]], [[R-15_region_goal_nav2]]): robot sam istražuje, sam zatvori
kartu i sam nastavi u misiju. Ručni i poluručni scenarij ostaju na gumbu — to je namjerna razlika,
ne propust.

> [!important] Ništa u lancu ne smije znati za sobe
> Ni broj soba, ni njihov redoslijed, ni imena. Jedino što se smije pretpostaviti: **karta je
> zatvorena, ima prolaze i ima stolove.** Svijet iz generatora (`scripts/gen_world.py`) je zato
> ispitni slučaj, ne ukras.

## 0. Zadani način: pokrivanje soba (`room_sweeper`)

> Od 17. 9. mapiranje **ne vozi frontier istraživanjem**. Razlog i brojke: [[D-23_coverage_sweep_instead_of_frontier]].
> Odjeljci 1–3 niže opisuju frontier lanac, koji se i dalje može pozvati s `explore_mode:=frontier`.

```
nađi prolaze → zapečati ih → pokrij sobu trakama → dopuni rupe →
prođi kroz neposjećen prolaz → ponovi
```

| Faza | Tko vozi | Zašto tako |
|---|---|---|
| pokrivanje sobe | `room_sweeper` izravno na `/cmd_vel` | čiste translacije po osima karte; smjer se **ne mijenja nijednom**, a okret je jedino gibanje u kojem baza kliže ([[P-10]]) |
| prolaz kroz vrata | **Nav2** (`NavigateToPose`, zadano stablo) | koridor za središte robota je ~12 cm; drži ga samo provjera otiska ([[P-45]]) |
| zaobilaženje prepreke | Nav2, automatski | ako ravna linija do sljedeće točke nije slobodna, potez ide planeru |
| sigurnost | `collision_monitor` → `/cmd_vel_safe` | sweeper objavljuje na `/cmd_vel`, dakle **u** monitor, ne mimo njega |

**Ruta se izračuna prije nego se robot pomakne**, pa se može nacrtati i provjeriti bez simulatora:

```bash
./scripts/run_native.sh python3 scripts/check_sweep.py --ascii
```

Ispisuje sobu, prolaze i rutu, i pada ako ruta izađe iz sobe, ostavi dio sobe nepregledan ili
zahtijeva ijedno dijagonalno (dakle rotacijsko) gibanje.

### Pokretanje

```bash
bash scripts/clean_ros.sh
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup scenario_auto_map.launch.py
```

Ništa se ne pritišće. Gumb **MAPIRANJE GOTOVO** ostaje za rani prekid. Za usporedbu sa starim
ponašanjem: `... scenario_auto_map.launch.py explore_mode:=frontier`.

### Što se gleda

| Gleda se | Dobro | Loše |
|---|---|---|
| smjer tijela | **stoji isti cijelo pokrivanje** | robot se okreće između traka |
| ruta | ravne dionice od zida do zida, bočni korak, pa natrag | lukovi, dijagonale, vraćanje |
| stol | traka se rasiječe oko njega i nastavi s druge strane | robot stane pred stolom |
| panel | `unknown_reachable` pada, `passages` raste | stoji, a robot se giba |
| vrata | poravna se pa ravno kroz (to je **jedini** okret) | ulazi ukoso |
| kraj | „sve sobe pokrivene i svi prolazi prođeni" | „nema neposjećenih prolaza, ali … nepoznato" |

### Parametri koje ima smisla dirati

| Parametar | Zadano | Što radi |
|---|---|---|
| `lane_spacing` | 1.5 m | razmak traka. Nije domet senzora nego **paralaksa** — koliko se robot pomakne prije nego pogleda iza noge stola iz drugog kuta |
| — | — | **Trake se voze samo blizu nepoznatog.** Prazna soba koju je lidar već vidio cijelu daje **praznu rutu** i robot odmah ide na prolaz; to je ispravan ishod, ne greška |
| `clearance` | 0.60 m | koliko traka drži od zida. Veće od poluduljine robota (0.52) i veće od pola otvora vrata (0.49), pa traka ne može ući u vrata |
| `closure_clearance` | 0.44 m | zasebno, tek iznad upisanog polumjera (0.427) — inače bi „dohvatljivo" isključilo sobu iza vrata |
| `sweep_speed` | 0.22 m/s | brzina dionice |
| `patch_passes` | 1 | koliko puta ponoviti sobu s trakama pomaknutim pola razmaka, ako ostane nepoznatog **u toj sobi** |
| `room_budget` | 240 s | najviše vremena po sobi; kad istekne, ide se na prolaz. Istek **nije** greška |
| `line_clearance` | 0.50 m | kad se potez vozi ravno, a kad ide Nav2-u. Kroz vrata nikad ravno — to drže pečati prolaza, ne ovaj broj |

### Prolaz se provjeri prije nego se na njega krene

Detekcija prolaza teče **uživo** cijelo mapiranje (`door_candidates(both_sides=False)` na svakoj
turi), ali sama detekcija nije dozvola. Prije nego pošalje ijedan Nav2 cilj, `prepare_passage`
provjerava, **nad kartom kakva je sada, ne kakva je bila pri detekciji**:

| Provjera | Zašto |
|---|---|
| otvor se **izmjeri iznova** i mora biti ≥ `min_passage_width` (0.90 m) | robot je u vožnji 0.821 m širok; širina zapisana pri detekciji je sa starije karte |
| poza za poravnanje (1.10 m ispred) mora biti prohodna | ta je poza u sobi koju vidimo, pa se drži punog kriterija |
| izlazna poza (0.90 m iza) ne smije biti u zidu | ta je poza u **nepoznatoj** sobi; nepoznato je dopušteno, zid nije — poza se skraćuje dok ne sjedne |
| os prolaza ne smije biti zapriječena | to je dio koji stvarno provlači robota kroz otvor |

Ako bilo što padne, prolaz se **preskoči s razlogom** (`doorway-rejected: otvor je 0.24 m, a treba
barem 0.90 m`) umjesto da se potroše dva Nav2 cilja po 90 s i dobije „navigation did not succeed".

> [!note] Dvije mjere nepoznatog, namjerno
> Panel pokazuje `nepoznato u sobi` i `ukupno`. Prva odlučuje treba li ponoviti sobu, druga je li
> karta gotova. Miješanje to dvoje je 18. 9. natjeralo robota da polaznu sobu pokrije dvaput,
> tjerajući 4485 „rupa" koje su zapravo bile dvije neposjećene sobe.

## 1. Lanac upravljanja — tko šalje brzinu, kada i zašto

| # | Tko | Što odlučuje | Zašto tako |
|---|---|---|---|
| 1 | `frontier_explorer` | **kamo**: bira granicu i šalje jedan `NavigateToPose` | odvojeno od izvedbe; nije regulator i ne dira brzine |
| 2 | `bt_navigator` | **ponašanje**: planiraj → prati → oporavak → replaniraj | oporavak za mapiranje ima **vlastito** stablo (niže) |
| 3 | `planner_server` (NavFn) | globalna putanja na `/map` iz SLAM-a + keepout iz `nav_zones` | točka, ne otisak — zato postoji potencijalno polje ([[D-20_single_potential_field_costmap]]) |
| 4 | `controller_server` (DWB, 20 Hz) | uzorkuje trajektorije na 1.2 s i boduje ih kritikama | jedini dio koji provjerava **otisak**; zato Nav2 vozi vrata, a gradijent polja ne ([[P-45_mission_integration]]) |
| 5 | smoother → `collision_monitor` → `cmd_vel_relay` | `/cmd_vel_safe` → `mecanum_drive_controller` | monitor se ne može zaobići: relay ignorira sirovi `/cmd_vel` dok monitor živi |
| 6 | povratna veza | `/scan_filtered` → slam_toolbox → `map → odom`; kotači → `odom → base_footprint` | dva izvora poze; kad prvi stane, sve stane ([[P-47_headless_batch_map_odom_stale]]) |

## 2. Gibanje pri mapiranju ima vlastiti profil

Do 17. 9. je mapiranje vozilo **istim** profilom kao misija, i to se vidjelo:

| Simptom | Izmjereni uzrok |
|---|---|
| pirueta na kraju svake dionice | cilj granice nosio je yaw, a `yaw_goal_tolerance` je 0.025 rad (1.4°) uz `RotateToGoal.scale: 64` |
| scan „pliva" po zidovima | okret u mjestu je gibanje u kojem baza kliže (`mu2 = 0`, [[P-10_skid_steer_cannot_turn]]); odometrija laže o yawu, scan matcher to ispravlja |
| dodatni okreti niotkuda | `spin` recovery iz zadanog BT stabla, nakon svakog palog cilja |
| vožnja unatrag kao obično gibanje | `min_vel_x: -0.30` **i** nepostojanje `PathAlign`/`GoalAlign` ([[P-48_no_path_alignment_critic]]) |

Zato mapiranje sada vozi **svojim** profilom, a misijski lanac ostaje netaknut
([[D-18_verified_baseline_first]]):

- `behavior_trees/explore_to_pose.xml` — bez `Spin` u oporavku; `ClearCostmap`, `BackUp` i `Wait`
  ostaju (`BackUp` je izlaz iz zaglavljenja, ne obrazac vožnje).
- `explore_goal_checker` — `yaw_goal_tolerance` praktički neograničen. **Lidar je 360°, pa
  orijentacija na granici ne nosi nijedan bit informacije.** Nema čega tjerati robota u okret.
- `FollowPathExplore` — `min_vel_x: -0.10` (unatrag postoji, ali sporo, pa nije najjeftiniji način
  putovanja), bez `RotateToGoal`, s dodanim `PathAlign` (32) i `GoalAlign` (8), `max_vel_theta: 0.25`
  (brzina koju ručna tura već koristi zbog [[P-11_nav2_slam_drift]]).

> [!important] Zašto uopće `PathAlign`
> Misijski popis kritika nema nijednu koja boduje **kurs** — samo gdje trajektorija završi. Na omni
> bazi to znači da se do cilja iza sebe dolazi vožnjom unatrag. To je cijelo objašnjenje „zašto vozi
> unazad": nije odluka, nego nedostatak. Zabrana unatrag bez te kritike zaustavi robota
> ([[P-48_no_path_alignment_critic]]).

**Okret u mjestu nije zabranjen** — poravnanje pred vratima ga treba, pa poze prolaza i dalje idu
zadanim stablom i uskim `general_goal_checker`-om. Razlika je u tome što više nije rutina.

## 3. Uvjet zaustavljanja: zatvorenost karte

> **Karta je zatvorena kad iz robotove poze, kroz prolaz širi od robota, ne postoji nijedna
> dohvatljiva nepoznata ćelija.**

Mjeri se tako da se slobodan prostor erodira za polumjer robota, poplavi iz robotove ćelije, i
prebroje nepoznate ćelije koje dodiruju tu dohvatljivu komponentu. Broj ide na
`/exploration/status` svaki krug, pa panel pokazuje stvarni napredak, a ne samo „radim“.

Ranije „gotovo“ je značilo samo *nemam više granice koja mi odgovara*: nakupina manja od praga,
nakupina na koju robot ne stane, ili nakupina na crnoj listi tiho su nestajale. Zato krug oko stola
nije bio napravljen. Sada se takva nakupina ne odbacuje nego dobiva **pozu promatranja** — najbližu
pozu u koju robot stane, s čistom linijom pogleda na nakupinu. Obilazak stola time ispada sam od
sebe, bez ijedne riječi o stolovima u kodu.

## 4. Pokretanje

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup scenario_auto_map.launch.py
```

Ništa se ne pritišće. Gumb **MAPIRANJE GOTOVO** ostaje za rani prekid.

Za usporedbu, ručno mapiranje (gumb je obavezan):
```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup scenario_manual_map.launch.py
```

## 5. Što se gleda u GUI-ju

| Gleda se | Dobro | Loše |
|---|---|---|
| kraj dionice | robot stane i odmah kreće dalje | vrti se u mjestu prije nego stane |
| scan vs zidovi | scan stoji na zidovima dok se robot giba | scan se okreće zajedno s robotom |
| smjer vožnje | naprijed i bočno; unatrag samo u oporavku | unatrag kao obično gibanje |
| `unknown_reachable` u panelu | pada prema 0 | stoji, a robot se giba |
| stol | robot ga obiđe dok oko njega ima nepoznatog | „gotovo“ dok je iza stola nepoznato |
| kraj | karta spremljena i misija krenula sama | čeka gumb u scenariju 1 |

## 6. Offline provjere iste karte

```bash
python3 scripts/check_frontier.py ~/.ros/pas_dual_arm/live_map.yaml     # zatvorenost
python3 scripts/check_map_geometry.py ~/.ros/pas_dual_arm/live_map.yaml # zid, otvor, os prolaza
```

Isti kod koji vozi robota — kopija logike u ispitnom sklopu je sklop koji prolazi dok robot stoji.

## 7. Kad stane, a ne bi trebao: lanac skenova

Zastoj `map → odom` ([[P-47_headless_batch_map_odom_stale]]) izgleda kao da je kriva navigacija, a
nije: presuši `/scan_filtered` pa stane sve što o njemu ovisi.

```
Ignition gpu_lidar → ros_gz bridge → /scan → scan_filter → /scan_filtered → slam_toolbox / collision_monitor
```

**Terminal 1**, prije simulacije (čeka sam):
```bash
./scripts/run_native.sh python3 scripts/scan_watch.py
```

| Ispis gledatelja | Značenje | Kamo dalje |
|---|---|---|
| `/scan` živ, `/scan_filtered` tih | puca `scan_filter` | Python čvor na 1080 zraka / 25 Hz |
| oba tiha, **Ignition JEST živ** | puca most ili DDS dostava | `ros_gz_bridge`, `fastdds_profiles.xml` |
| oba tiha, **Ignition NIJE živ** | puca senzor u simulatoru | Sensors sustav, teret 3 RGBD + lidar |

> [!note] Gledatelj namjerno radi na stvarnom satu
> Mjeri kvar koji zaustavlja sve što ovisi o sim vremenu; watchdog koji se zamrzne zajedno sa
> svojim predmetom ne javlja ništa.

### Par runova koji razdvaja most od senzora i DDS-a

Most nosi **šest teških tema** neprekidno — po jedan oblak točaka i jednu dubinsku sliku za glavnu
i obje kamere na zapešću — a mapiranje ne treba nijednu; treba `/scan`. [[P-20_pointcloud_starves_clock]]
je već izmjerio što jedan takav oblak radi izvršitelju. `heavy_sensors:=false` ih izbacuje iz mosta,
pa ista scena odvožena dvaput razdvaja most od preostale dvije karike:

```bash
# Run A — kakav jest, sa svim temama (ovo je referenca)
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup scenario_auto_map.launch.py

# Run B — isto, bez oblaka i dubinskih slika na mostu
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup scenario_auto_map.launch.py \
    heavy_sensors:=false
```

| Ishod | Presuda |
|---|---|
| A stane u ~8.7 s, **B vozi dalje** | kriv je **most** (propusnost / DDS pod teretom) |
| oba stanu jednako | most nije kriv → senzor u simulatoru ili DDS dostava |
| B stane ranije ili drukčije | zapisati kao zaseban nalaz, ne tumačiti |

> [!warning] `heavy_sensors:=false` nije za misijski run
> `measure_box` čita `/camera/points`, a V4 hvat se zatvara po dubini s kamera na zapešću. Bez njih
> mapiranje radi, a hvat ne. Zato je zastavica **mjerni instrument**, ne postavka.

Popis izbačenih tema ispisuje se u zaglavlju generiranog `~/.ros/pas_dual_arm/bridge_light.yaml`;
datoteka se izvodi iz `config/bridge.yaml` pri svakom pokretanju, pa se te dvije ne mogu raziću.

### Što sada javlja sam istraživač

`frontier_explorer` mjeri starost `map → odom` (parametar `tf_stale_timeout`, zadano 5 s) i u
petlji vožnje i prije svakog izbora granice. Kad transformacija zastari, run **stane s porukom**
```
EXPLORE stalled: map → odom nije osvježen 12.3 s — skenovi su stali.
```
umjesto da 25 s šalje isti cilj u prazno. Granica se pritom **ne stavlja na crnu listu** — nije ona
kriva, pa bi je tri takva ciklusa potrošila da se dođe do krivog zaključka.

Cijeli ispis (s vremenima) ide u red u [[runovi]] i u tablicu pokušaja pripadne P-kartice — i kad
ništa ne padne.
