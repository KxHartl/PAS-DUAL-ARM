---
id: P-19
type: problem
status: rijeseno
requirements: ["[[R-16_find_box]]"]
solutions: ["[[S-05_perception]]", "[[S-06_navigation]]"]
decisions: ["[[D-04_visual_servo_instead_nav2]]"]
updated: 2026-09-15
---
# P-19: Marker se gubi kad je robot blizu kutije

## Simptom
U vizualnom prilazu ArUco nestaje ispod ~0.9 m.

## Uzrok
**Potvrđeno:** nisko postavljen vertikalni marker traži strm nagib kamere, pa se marker
foreshorten-a i detektor ga odbaci.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. `33bc2ac` | servo samo do ~0.9 m, zatim ravni dovoz + mjerenje dubinom (dubina ne pati od kuta) | pouzdan prilaz | rješenje |
| 2 | 16. 7. `c504720` | mjerenje s 0.95 m **prije** dovoza, dovoz korigiran odometrijom | točan centar nakon dovoza | poboljšanje ([[P-22_depth_self_view_clusters]]) |
| — | 15. 9., radno stablo | GUI run korisnika, `--characterize`: stisak prekinut nakon 4 s s `stale TF right_grasp_marker_frame: +0.400 s` (granica `force_vision_age` = 0.4 s) | Kamere **nisu** uzrok: izmjereno u **sim** vremenu, sve tri idu 15.15 Hz s razmakom točno 0.066 s, bez ijedne rupe. Znači detektor je prestao **prepoznavati** marker, ne kamera slati slike. Procjena sile je cijelo vrijeme bila valjana (201/201 uzoraka) | Prag svježine vida postavljen je bez mjerenja koliko detektor stvarno pouzdano vidi marker na 2 cm od plohe. Snimač od sada bilježi `/wrist_*/marker_pose`, pa idući run daje raspodjelu rupa u prepoznavanju |
| — | 15. 9., radno stablo | Snimač dopunjen da bilježi `/wrist_*/marker_pose`, pa je izmjereno **što se stvarno događa** kod `stale TF ..._grasp_marker_frame` | **Detektor ne gubi marker.** 576 detekcija po ruci, svaki razmak točno 0.066 s (15 Hz), **nijedan preko 0.4 s**; detekcije su tekle do t=155 s, a prekid je bio na t=102.7 s. Kašnjenje od oznake kamere do pretplatnika 22–43 ms, max 76 ms. Kamere u **sim** vremenu idu 15.15 Hz bez ijedne rupe | Prekid **nije** problem prepoznavanja izbliza. Zastarijeva TF **kako ga čita sam čvor**: izmjereno median **0.32 s**, p95 0.388, max 0.407 — a granica je bila 0.400 s, dakle **ispod tipične starosti**. Hipoteza za ostatak od ~0.25 s: `/tf` nosi `robot_state_publisher` na frekvenciji zglobova, a `spin_once` obradi jednu povratnu funkciju po pozivu, pa međuspremnik zaostaje |

## Trenutno rješenje
`visual_approach(target_x=0.90)`, `look_down(0.65)`, `measure_box`, `drive` + odometrija.


### Stvarni uzrok: zagušenje `/tf` u procesu koji ga čita
Izmjereno u tri koraka, jer su me dvije pretpostavke prije toga obmanule:

| mjerenje | rezultat |
|---|---|
| starost očitanja **u petlji stiska**, granica 0.4 s | median **0.320 s**, p95 0.388, max 0.407 |
| ista stvar nakon podizanja granice na 0.8 s | median **0.732 s**, p95 0.793, max 0.802 |
| isti TF u **mirnom** čvoru koji ne radi ništa drugo | median **0.060 s**, p95 0.090, max 0.111 |

Starost **prati prag** — to je zaostatak koji raste, ne zastario senzor. Uzrok:
`robot_state_publisher` republicira `/tf` na frekvenciji zglobova, a to je **1000 Hz** pod
`force_grasp` profilom (`controller_manager.update_rate`). Python konzument to ne stigne
isprazniti dok glavna petlja drži GIL. `TransformListener(..., spin_thread=True)` **nije pomogao**
(median 0.732 → 0.738), jer GIL dijele obje dretve.

**Rješenje:** vid u `force_grasp` više ne ide kroz `/tf` nego izravno s teme
`/wrist_*/marker_pose`, jednako kao što sila ide sa svoje teme (i zato je svježa, 0.010 s).
Kamera je **fiksno dijete** end-effector linka, pa se transformacija kamera→alat pročita jednom
prije stiska i tijekom regulacije nema nijednog TF čitanja za vid.

Smanjenje takta kontrolera je razmatrano i **odbačeno**: granica prigušenja je `d < 2I/dt`, pa bi
na 200 Hz izmjerena `d` skočila s 15 % na 75 % granice i zglobovi bi opet zatitrali
([[P-41_effort_pid_arm_actuator_profile]]).

## Napomena (15. 9.): ova kartica nije uzrok prekida stiska
Prekidi `stale TF ..._grasp_marker_frame` **nisu** iz foreshorteninga. Izmjereno je da detektor
izbliza radi bez ijednog promašaja; kriva je bila granica svježine, postavljena bez mjerenja.
Vidi zadnji red tablice i [[06_parametri]] (`force_vision_age` 0.4 → 0.8 s).
Foreshortening izbliza ostaje otvoren kao zaseban rizik, ali ga ovi runovi **nisu** pokazali.
