---
id: RUN_TESTIRANJE
type: upute
updated: 2026-09-13
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
