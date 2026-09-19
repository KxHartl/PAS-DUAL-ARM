---
id: D-28
type: decision
status: vazeca
verified: "12/12, serija fin2-par, 20. 9. 2026., mjereno u simuliranom vremenu."
updated: 2026-09-20
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]", "[[R-20_place_at_destination]]"]
problems: ["[[P-54_place_bias_not_localisation]]", "[[P-55_doorway_skew_is_control]]", "[[P-56_speed_is_capped_by_slip]]"]
decisions: ["[[D-18_measure_do_not_argue]]", "[[D-27_dwb_stays_mppi_cannot_fit]]"]
---
# D-28: najbrži postav koji pouzdano radi — 173 s naspram 224 s

## Rezultat
| serija | izmjena | uspjeh | misija (sim s) |
|---|---|---|---|
| `finale2-19-09` | polazište | 10/10 | 224,2 |
| `manip-par` | ubrzano rukovanje | 12/12 | 188,3 |
| `door-par` | vrata 0,26 m/s | 12/12 | 179,3 |
| `fin-par` | + prilaz stolu 0,10, vrata 0,30 | 11/12 | 174,0 |
| **`fin2-par`** | **+ spašavanje laktovima** | **12/12** | **173,3** |

**−23 % vremena misije, bez gubitka pouzdanosti.**

## Kako se mjerilo
Zidno vrijeme prestalo je nešto značiti čim su runovi krenuli paralelno: tri simulacije dijele
stroj, RTF padne s 0,46 na 0,29, i svaki run „usporava" a da robot ne radi ništa drukčije.
Zato `validation/speed_summary.py` mjeri **simulirano** vrijeme od kretanja po kocku do
`MISSION COMPLETE`, a `validation/phase_times.py` ga razbija po fazama.

Paralelni sklop (`validation/run_parallel.py`) skratio je pokus s ~100 min na ~35, što je i
omogućilo da se svaka izmjena sudi na **12 runova** umjesto na tri.

## Što je prihvaćeno
**Rukovanje je bilo 56 % misije i nitko ga nije mjerio.** Sve su to bile brzine postavljene za
najgori slučaj i primijenjene na cijelu kretnju:

| | prije | sada |
|---|---|---|
| spuštanje kocke na marker | 1 cm/s cijelih 20 cm | 5 cm/s do 2 cm iznad, pa 1 cm/s |
| odmicanje baze od stola (2×) | 0,08 m/s | 0,15 m/s |
| povlačenje kocke i poravnanje na markeru | 0,02 m/s | 0,04 m/s |
| spust ruku na detekcijsku pozu (kroz zrak) | 0,03 m/s | 0,06 m/s |
| prilaz stolu s kockom iznad njega | 0,05 m/s | 0,10 m/s |
| **prolaz kroz vrata** | 0,18 / 0,22 m/s | **0,30 / 0,34** |

Kontaktni prilaz (`grasp_approach_speed`, 1 cm/s) i **dokiranje** (0,18/0,22) namjerno su
ostali spori: ondje preciznost dolaska određuje hvat.

## Što je odbijeno, i zašto
| izmjena | rezultat |
|---|---|
| MPPI + `Omni` | 8/12, **240 s** — sporiji i manje pouzdan ([[D-27_dwb_stays_mppi_cannot_fit]]) |
| maska brzine po karti | s DWB-om 0/5 — vozi vrata na 80 % gdje po dionici ide 0,22 |
| replaniranje 20 Hz | ušlo zajedno s maskom; vraćeno s njom, nije zasebno dokazano |
| vrata 0,34 m/s | 11/12 — zastoj u vratima |
| lakše uzorkovanje DWB-a (9375 → 2475) | rokovi se nisu popravili; regulator **čeka**, ne računa |
| grublja provjera sudara (1 → 2,5 cm) | ne pomaže rokovima, a u vratima s 5 cm zaliha je samo rizik |
| fizika 2 ms | +24 % RTF, ali 500 Hz simulira ruku čija stvarna petlja radi na 1 kHz |

## Ograda
Spašavanje laktovima (`extra_elbow_out_deg`) **nije se okinulo ni u jednom od tih 12 runova** —
kvar koji rješava pojavljuje se oko 1 u 12. Ovih 12/12 dokazuje da je postav pouzdan na tom
uzorku, ne da popravak radi. To se pokaže tek kad se blokada ponovno dogodi.
