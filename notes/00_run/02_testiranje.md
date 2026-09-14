---
id: RUN_TESTIRANJE
type: upute
updated: 2026-09-14
---
# Testiranje i kriteriji uspjeha

## Brze provjere bez vožnje

```bash
./scripts/run_native.sh python3 -m py_compile src/pas_dual_arm_scripts/pas_dual_arm_scripts/{postures,mapping_tour,main_task,scan_filter,feature_registry}.py
./scripts/run_native.sh python3 scripts/check_map.py src/pas_dual_arm_bringup/maps/seminar_map.yaml
```

Druga naredba **sada treba javiti da karta ne postoji**. To je očekivano stanje,
ne razlog za stvaranje lažne karte. Nakon pokretanja simulacije i mapiranja:

```bash
./scripts/run_native.sh ros2 topic hz /scan_filtered
./scripts/run_native.sh ros2 topic echo /joint_states --once
./scripts/run_native.sh ros2 topic echo /semantic_features --once
```

`/semantic_features` je registar opažanja, ne dokaz da su vrata/stol/kutija
pouzdano lokalizirani. Pregledati koordinate, izvor i više pogleda u RViz-u.

## Provjera ruku i prolaza

Poza `ARM_CARRY_V2` je geometrijski izmjerena na **0.854 m**, ali u simulaciji
nakon gibanja izmjerena je **1.109 m**. Vrata su 1.0 m. MoveIt `SUCCEEDED`
ne vrijedi kao potvrda fizičke poze. Prije vožnje usporediti svaki zglob u
`/joint_states` s `postures.py` i izmjeriti stvarnu širinu meshova/TF-a; isto
ponoviti za vrijeme i nakon kratke vožnje u sigurnom dijelu sobe. `mapping_tour`
zaustavlja pokušaj ako je odstupanje preveliko, no to nije zamjena za GUI pregled.

## SLAM prihvat

Zapisati za svaki segment zadanu i odometrijski izmjerenu udaljenost, sken te
promjenu `map→odom`. Zadnji neuspjeli run: 4.5 m zadano, 1.89 m izmjereno,
skok korekcije 1.04 m. Takvu kartu **ne spremati za Nav2**. Prihvatiti tek kad
su sve tri sobe i oba prolaza konzistentni, nema velikog skoka korekcije,
fizička širina stane kroz otvor i `check_map.py` prođe.

## Lokalizacija/Nav2 prihvat

Nakon spremanja karte: svježi start, AMCL, početna poza u RViz-u, stabilan TF,
plan kroz vrata koji ima stvarni razmak od zidova, zatim kratki cilj u istoj
sobi prije međusobnog prolaza. Uspjeh akcije `NavigateToPose` bez izmjerenog
dolaska i bez provjere kolizija nije dovoljan.

Svaki pokušaj i neuspjeh upisati u [[runovi]] i pripadnu P-karticu; ne prepisivati
stari red. Glavni otvoreni problemi: [[P-37_arm_position_gain_sag]] i
[[P-11_nav2_slam_drift]]. Ako se kutija sama pomakne prije kontakta, prekini
mapiranje i slijedi izolacijski test u [[P-38_spontaneous_box_motion]].

## Navigacija sa zonama (od 14. 9., [[P-39_nav2_enters_doorway_at_an_angle]])
Redom, i ne preskakati — prva dva koraka ne trebaju simulator:

| # | Provjera | Prolazi ako |
|---|---|---|
| 1 | `python3 scripts/check_doors.py` | oboja vrata nađena, promašaj centra < 10 cm, širina 0.85–1.05 m |
| 2 | `python3 scripts/check_zones.py` | 3 sobe, 2 vrata, 2 stola; sve poze na kojima navigator stane izvan zona i robot se u njima smije okrenuti u mjestu (osim `dock`); **brazde nulte cijene** kroz vrata i prilaz stolu čiste; **napuhana maska ne zatvara prolaz** (≥ 2 × tolerancija cilja za centar robota); **soba prohodna** — slobodan prostor erodiran za polovicu širine robota i dalje spaja sve poze |
| 2b | `python3 scripts/check_costmap_path.py` | sve poze na koje navigator šalje robota međusobno dostupne, i putanja drži razmak od stvarnih prepreka u rasponu **0.60–0.90 m** (ispod 0.52 m `collision_monitor` zaglavi robota, iznad 0.90 m putanja nepotrebno struže po suprotnom zidu). Traje ~1 min |
| 2c | brazde i polje (dio `check_zones.py`) | sve brazde **nulte** cijene po cijeloj duljini i širini (uvjet iz runa 65 — NavFn ne može proći zasićeni potencijal), jezgra netaknuta uz svaku uzorkovanu zidnu ćeliju, koridor kroz vrata nije uži od onoga što sam otvor dopušta, dock poza nije lethal |
| 3 | RViz `/nav_zones_markers` | zelena traka u oba otvora; **oko stolova nema zone** (od 14. 9.). Za usporedbu se u RViz-u može uključiti `Keepout (planer, napuhano)` — to je ista zona narasla za 0.427 m, onako kako je vidi globalni planer |
| 4 | **ručni** „2D Goal Pose" unutar polazne sobe | stigao, |Δyaw| < 0.05 rad, bez dodira zida |
| 5 | **ručni** „2D Goal Pose" u drugoj sobi | putanja ulazi u otvor **okomito**, prolaz bez dodira |
| 6 | **tipka** PLAVA, pa CRVENA, pa HOME | stane pred stol; log javi bočni razmak > 2.5 cm i |Δyaw| na pragu < 0.05 rad |
| 7 | ponovljivost | 3 uzastopna prolaza po smjeru bez dodira; skok `map→odom` < 0.2 m |
| 8 | **negativni test** | ruke izvan `ARM_CARRY_V2` → abort **prije** zone vrata, ne pokušaj prolaza |

**Nav2 vozi svaku dionicu**, i prilaznu i onu kroz vrata. Kod vrata postoje samo dvije
stvari: portalne poze na osi prolaza i preduvjet `arms_ok()` + `aligned_with()`
(8 cm / 5°), koji pošteno stane umjesto da struže ([[D-18_verified_baseline_first]]).

`NavigateToPose` koji javi uspjeh nije dokaz: mjeri se izmjerena poza po dionici, kut na
pragu vrata i najmanji bočni razmak (oboje ispisuje `room_navigator`).

**Referentni brojevi** (run 46, [[runovi]]) — ako neka izmjena ovo pokvari, izmjena je kriva,
ne mjerilo:

| Provjera | Očekivano |
|---|---|
| ručni cilj (0.0, −4.5, −90°) → plava soba | ~**33 s**, Vrata 0 s ≤ 5 cm i ≤ 5° |
| `goto red` | 5 dionica, pred crvenim stolom za ~**60 s** |
| oboje | bez dodira zida, bez aborta |
