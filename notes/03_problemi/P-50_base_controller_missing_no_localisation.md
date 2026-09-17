---
id: P-50
type: problem
status: rijeseno-ceka-run
verified: "run 1 serije 2026-09-18_0143; uzrok izmjeren iz baga, popravak još nije vožen"
updated: 2026-09-18
requirements: ["[[R-15_region_goal_nav2]]", "[[R-21_deliverables]]"]
solutions: ["[[S-10_build_run_environment]]", "[[S-06_navigation]]"]
decisions: ["[[D-24_measure_from_recordings_not_logs]]"]
---
# P-50: bez `base_controller` nema odometrije, pa ni lokalizacije — a run se broji kao neuspjeh misije

## Simptom
Nakon popravka [[P-49_empty_goal_checker_id_with_two_checkers]] misija pada **odmah**, bez ijednog
pokušaja vožnje:

```
room_navigator: cannot go to "blue:dock": no map -> base_footprint transform; is AMCL localised?
```

## Što bag kaže (a log ne)
Prva korist od snimanja ([[D-24_measure_from_recordings_not_logs]]) — `ros2 bag info`:

| Tema | Run 1 serije 0143 (pao) | Run 1 serije 0130 (radio do P-49) |
|---|---|---|
| `/amcl_pose` | **0** | 44 |
| `/particle_cloud` | **0** | 86 |
| `/base_controller/odom` | **0** | 7147 |
| `/tf` | 1222 | 10333 |
| `/scan` / `/scan_filtered` | 1105 / 1015 | — |

Skenovi teku normalno, dakle **nije** [[P-47_headless_batch_map_odom_stale]]. AMCL nije objavio
**nijednu** pozu.

## Uzrok
U logu je aktivirano **sedam od osam** kontrolera; `base_controller` nije ni „Loaded":

```
spawner_base_controller: waiting for service /controller_manager/list_controllers ...
[ERROR] [spawner-11]: process has died  (exit code -15, tj. ugašen pri raspremanju)
```

Lanac je time prekinut na prvoj karici: bez `base_controller`-a nema odometrije kotača → nema
`odom → base_footprint` → AMCL nema na što nasloniti `map → odom` → `map → base_footprint` ne
postoji → navigator odbija cilj.

**Zašto se dogodilo:** serija 0143 pokrenuta je u sekundi u kojoj je prethodna prekinuta
(`run_003` loga staje u 1789688606, novi Gazebo kreće u 1789688608). `clean_ros.sh` pošalje
`kill -9` i čeka jednu sekundu; Gazebo se gasi dulje, pa je novi stack krenuo preko umirućeg.
Potvrda da raspremanje nije čisto: nakon prekida serije u sustavu je ostalo ~25 procesa, među
njima `ign gazebo -s -r .../run_002/world.sdf`.

## Popravak
Tri brane u `validation/run_batch.py`, sve prije pritiska na „gumb":

1. **`wait_for_quiet()`** — nakon `clean_ros.sh` čeka dok ijedan `ign gazebo`/`gz sim` proces ne
   nestane (do 60 s). Run se ne pokreće preko prethodnog.
2. **`wait_for_controllers()`** — čeka svih **8** `Configured and activated`.
3. **`wait_for_tf()` sada traži `map → base_footprint`**, a ne `map → odom`. Stara provjera je
   prolazila upravo u stanju u kojem robota još nema u stablu transformacija.

Padne li ijedna brana, run se upisuje kao **`not_ready`**, a ne kao prekid misije.
`summarize.py` takve runove izuzima iz nazivnika uspješnosti i navodi ih odvojeno — **stack koji
se nije digao nije misija koja je pala**, i miješanje to dvoje upravo bi pokvarilo brojku zbog
koje sve ovo i radimo.

## Tablica pokušaja
| # | Datum | Što je pokušano | Ishod |
|---|---|---|---|
| 1 | 18. 9. 01:43 | serija pokrenuta odmah nakon prekida prethodne | `base_controller` nije učitan; 0 `amcl_pose` |
| 2 | 18. 9. | uzrok izmjeren iz baga (`ros2 bag info`), ne iz loga | lanac odometrija → AMCL → TF potvrđen brojkama |
| 3 | 18. 9. | tri brane + ishod `not_ready` | **čeka run** |

## Otvoreno
Ponoviti V1 iz [[validacija]]. Prije pokretanja provjeriti da ništa ne radi:
`pgrep -af "ign gazebo|gz sim"` mora biti prazno (ili `bash scripts/clean_ros.sh`).
**Prekid serije s Ctrl-C ne gasi simulaciju** — `run_batch.py` pokreće launch u vlastitoj sesiji,
pa Ctrl-C stigne samo do skripte; nakon prekida uvijek pokrenuti `clean_ros.sh`.
