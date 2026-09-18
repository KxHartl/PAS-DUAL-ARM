---
id: P-47
type: problem
status: rijeseno
verified: "uzrok pronađen 18. 9. — [[P-51_apt_upgrade_stops_amcl]]; nakon vraćanja ROS stacka misija vozi do kraja"
updated: 2026-09-18
requirements: ["[[R-14_slam_mapping]]", "[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]"]
solutions: ["[[S-06_navigation]]"]
decisions: []
---
# P-47: `map → odom` zastari, pa prva dionica pukne

> [!warning] Nije headless-specifično (17. 9. navečer)
> Isti kvar se dogodio u **GUI runu automatskog mapiranja**, gdje `map → odom` objavljuje
> slam_toolbox, a ne AMCL. Zajednički nazivnik nije ni headless ni AMCL, nego **prestanak
> dotoka skenova**. Naslov datoteke je ostao radi poveznica.

> [!important] RIJEŠENO 18. 9. — uzrok je bio izvan repozitorija
> Kvar opisan ispod uzrokovala je **`apt` nadogradnja 439 `ros-humble-*` paketa 17. 9. u 18:09**
> ([[P-51_apt_upgrade_stops_amcl]]). Nakon vraćanja stacka na snapshot 2026-08-07 misija vozi
> `MISSION COMPLETE`. **Ništa od popravaka u ovoj kartici nije bilo uzrok ni lijek.**
>
> **Ispravak dijagnoze:** ispod piše da prestaje dotok `/scan_filtered`. **To nije točno.**
> Mjereno uživo tijekom kvara 18. 9.: `/scan` 13.6 Hz, `/scan_filtered` 13.3 Hz,
> `/base_controller/odom` 56 Hz, `/clock` 459 Hz — svi živi; **šuti samo `/amcl_pose`**.
> AMCL prima skenove i ne obrađuje ih. Zaključak „tri nezavisna potrošača stanu u istoj sekundi,
> dakle tema je stala" bio je pogrešan: sva tri su potrošači koji su **stali sami**, svaki iz
> istog razloga. Analiza iz 17. 9. ostaje zapisana jer pokazuje kako se do krivog osumnjičenika
> došlo.

## Simptom
U automatiziranom ispitivanju (`validation/run_batch.py`, `headless:=true`) misija se
**svaki put** prekida na prvoj navigacijskoj dionici:

```
[controller_server] [tf_help]: Transform data too old when converting from map to odom
                    Data time: 47s 160000000ns, Transform time: 7s 60000000ns
[room_navigator] leg 1/4 ... arrived 184.0 cm and +90.0 deg from the goal
[room_navigator] ABORT before leg 2/4: +90.0 deg off square (limit 5.0)
[main_task]      Task aborted during: room_navigator did not reach blue:dock
```

Robot se **nije pomaknuo**: „arrived" je proglašen 0,5 s nakon slanja cilja, 1,84 m i 90° od
cilja. `room_navigator` zatim ispravno odbija ući u vrata ukoso i prekida.

## Što je isključeno (provjereno)
| Sumnja | Provjera | Ishod |
|---|---|---|
| generator svijeta | run s `--stock-world` (izvorni `seminar_world.sdf`) | **isto pada** — nije generator |
| harness šalje start prerano | dodano čekanje na `map → odom` + 10 s; razmak WAITING → cilj 58 s | **isto pada** |
| lidar ne radi | `room_navigator` mjeri „tightest gap 252.2 cm"; geometrija daje 3,0 − 0,427 = 2,57 m | skenovi su živi i točni |
| `scan_filter` je pao | traceback u logu je tek pri gašenju (SIGINT) | bio je živ |
| simulacija juri ispred čvorova | RTF izračunat iz vremena: 47 s sim / 85 s stvarno | **0,55** — sim je sporiji, ne brži |

## Gdje je trag
Zadnja objavljena transformacija `map → odom` nosi žig **7 s** simulacijskog vremena, dok je sim
vrijeme već **47 s**. Dakle AMCL ju je objavio na početku i **prestao je osvježavati** ~40 s sim
vremena. `room_navigator` to ne primijeti jer TF gleda s `Time(0)` (najnovije dostupno), a
`controller_server` traži transformaciju u sadašnjem trenutku i odbija je kao prestaru.

AMCL u logu **nema nijedan zapis** jer `quiet:=true` diže razinu nav2 zapisa na `warn` — sljedeći
korak je pustiti run s `quiet:=false` i vidjeti aktivira li se uopće i prima li skenove.

## Zašto se nije vidjelo prije
Sva dosadašnja izvođenja su GUI runovi u kojima čovjek pritisne gumb desetak sekundi nakon
pokretanja, i to uz uključen prikaz. Ovo je prvi put da se misija pokreće automatski i bez
prikaza. **Nalaz je sam po sebi vrijedan**: pokazuje da izmjereni rezultati vrijede za način rada
u kojem su snimljeni, a ne bezuvjetno.

## Prvi plan (headless, ujutro 17. 9. — nadiđen nalazom niže)
1. `quiet:=false` + `--ros-args --log-level amcl:=debug` u headless runu → objavljuje li AMCL.
2. Provjeriti prima li AMCL `/scan_filtered` (`ros2 topic info`, broj pretplatnika).
3. Usporediti s GUI runom na istom stroju — je li headless uvjet ili samo pojačava.
4. Ako AMCL stane bez gibanja, razmotriti `transform_tolerance` i periodičko
   ponovno objavljivanje.

Do tada `validation/run_batch.py` mjeri **prekid na prvoj dionici**, a ne pouzdanost misije, pa se
brojke iz njega ne smiju navoditi kao uspješnost sustava.

## Isti kvar u mapiranju: prestao je dotok skenova (GUI run, 17. 9.)

Automatsko mapiranje (`scenario_auto_map.launch.py`, GUI): robot krene prema prvoj granici, jednom
se pomakne i stane. Iz logova runa (`~/.ros/log/2026-09-17-21-18-53-...`):

| Mjera | Vrijednost | Što znači |
|---|---|---|
| zadnji `map → odom` | žig **8.84 s**, nepomičan 18 s | slam_toolbox ga stampa kao *zadnji obrađeni sken* + `transform_timeout` (0.2) → zadnji sken je **8.64 s** |
| `collision_monitor` | traži `8.690000` u svakoj poruci | njegov zadnji sken je **8.69 s** |
| `/map` | `85 granica … 3108 ćelija`, identično 25 s | karta prestala rasti |
| `/clock`, `odom → base_footprint` | teku dalje (RTF 0.57) | fizika i odometrija su **žive** |
| log `scan_filter`-a | **prazan** | nije odbacio nijedan sken zbog TF-a; nije ga ni dobio |
| Ignition `server_console.log` | nijedna greška; render nit stala tek na SIGINT | Ogre/senzor nisu pukli s iznimkom |
| `cmd_vel_relay` | `collision monitor is live` u **756.67 s** stvarnog | sim 8.69 ↔ stvarnih 756.65 — **razmak 20 ms** |

Tri nezavisna potrošača `/scan_filtered` stali su u istoj sekundi, a `scan_filter` nije prijavio
nijedno odbacivanje. Dakle **`/scan_filtered` je prestao**, i to u trenutku u kojem je robot prvi
put dobio naredbu brzine.

**Lanac posljedica (potvrđen, ne pretpostavljen):** sken stane → slam_toolbox prestane osvježavati
`map → odom` → `controller_server` odbija prestaru transformaciju → nema brzine → robot stoji →
`frontier_explorer` beskonačno šalje isti cilj. U misiji (headless) je lanac isti, samo `map → odom`
drži AMCL, koji ga također objavljuje **na svaki sken**.

**Nije uzrok, ali je za popraviti kad dođe red:** `cmd_vel_relay._safety_live()` mjeri
`time.monotonic()` (stvarni sat) s pragom 1.0 s, dok je sve ostalo na sim vremenu — krši
invarijantu 4 iz [[AGENT_GUIDE]]. Pri RTF 0.57 još prolazi; pri nižem bi zatreperilo.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 17. 9. | analiza logova GUI runa automatskog mapiranja | zastoj lociran na `/scan_filtered`, uzrok sužen na tri karike | nije headless, nije AMCL, nije `scan_filter` |
| 2 | 17. 9. | `sim.launch.py heavy_sensors:=false` — most bez 6 teških tema (3 oblaka + 3 dubinske slike), izveden iz `bridge.yaml` pri pokretanju | 🧪 čeka par runova A/B ([[automated_mapping]]) | most je jedina od tri karike koju zastavica može isključiti bez diranja svijeta |
| 3 | 17. 9. | watchdog starosti `map → odom` u `frontier_explorer` (`tf_stale_timeout`, 5 s) | 🧪 čeka vožnju | kvar se dosad vidio samo kao tišina; sada ga run imenuje i stane |
| 4 | 17. 9. | `cmd_vel_relay._safety_live()` prebačen sa `time.monotonic()` na sim vrijeme, čvor dobio `use_sim_time` | ⚠ nije promijenilo ishod | nije uzrok, ali je kršenje invarijante 4; pri RTF 0.57 je 1.0 s stvarnog tek 0.57 s sim. **Izmjena je zadržana** |
| 5 | 18. 9. | **A/B sa stablom od 16. 9.** (`d594fd8`) u istom okolišu | **pada identično** (`too old` 94, padova planera 14, ista poruka `PREKID [3/8]`) | **kod nije krivac** → [[P-51_apt_upgrade_stops_amcl]] |
| 6 | 18. 9. | mjerenje tema uživo tijekom kvara | skenovi teku 13 Hz, `/amcl_pose` šuti | **stane AMCL, ne dotok skenova** — ispravak dijagnoze iz #1 |
| 7 | 18. 9. | vraćanje ROS stacka na snapshot 2026-08-07 + `hold` + čist rebuild | ✅ **`MISSION COMPLETE`, 5 mm od centra markera**, 0 × `Transform data too old` | **rješenje** ([[P-51_apt_upgrade_stops_amcl]]) |

## Ishod
A/B par iz koraka 1 (`heavy_sensors` A/B, `scan_watch.py`) **nije bio potreban** — pitanje koje je
trebao razriješiti („senzor, most ili DDS?") bilo je krivo postavljeno, jer nijedna od te tri
karike nije stala. `scripts/scan_watch.py` ostaje u repozitoriju kao koristan alat.

Zadržano od popravaka iz 17. 9. (vrijedi neovisno o uzroku):
- watchdog starosti `map → odom` u `frontier_explorer` — kvar se sada **imenuje** umjesto da šuti;
- `cmd_vel_relay` na sim vremenu (invarijanta 4 iz [[AGENT_GUIDE]]);
- isti watchdog u `room_navigator` i dalje **nije** napravljen.
