---
id: P-13
type: problem
status: djelomicno
requirements: ["[[R-03_linear_rails_torso]]", "[[R-06_realistic_parameters]]"]
solutions: ["[[S-01_robot_description]]", "[[S-03_ros2_control_setup]]"]
decisions: ["[[D-09_lift_with_arms_not_torso]]"]
updated: 2026-09-14
---
# P-13: Klizači torza (prismatic) se ne dižu pod težinom ruke

## Simptom
`torso_controller` naredi visinu, a opterećeni vertikalni klizač ostaje na donjem limitu (0.05).
Lagani zglobovi (pan-tilt) rade normalno.

## Uzrok
U izoliranom headless pokusu 14. 9. potvrđeno je da postojeći `position` command interface
pod teretom ne pokreće vodilice, iako JTC vraća `SUCCEEDED`. `effort` interface uz PID u JTC-u
pomaknuo je obje vodilice i držao cilj. To je dokaz da blokada nije bila nepremostiva
geometrijska kolizija; dinamiku kontaktnog hvata s dodatnom masom još treba provjeriti.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. `397f080` | efort limit 100 → 1000 N | bez promjene | nije samo efort |
| 2 | 30. 6. `397f080` | `min`/`max` na command interfaceu | bez promjene | — |
| 3 | 30. 6. `397f080` | `position_proportional_gain` (sada 20) | bez promjene | — |
| 4 | 14. 9., `feature/cube-manipulation`, C1 | postojeći pozicijski kontroler, cilj 0.20 m s rukama u `ARM_CARRY_V2` | akcija `SUCCEEDED`, stvarno 0.050/0.050 m | nema stvarnog pomaka; status akcije nije dokaz |
| 5 | 14. 9., `feature/cube-manipulation`, C2 | `effort` interface + JTC PID: p=1500, i=500, d=100, i_clamp=300; isti cilj | nakon 2 s 0.1944/0.1944 m; nakon 10 s 0.2000/0.2000 m; povratak na 0.0500/0.0500 m | vodilice rade pod težinom ruku i drže visinu |
| 6 | 15. 9. `ee4dd4d` | **`position` sučelje na `main`-u** (Codexov `effort` odbačen), `table_ready` zada 0.200 m preko `torso_controller`, ruke u `table_arms` pozi, GUI | akcija `SUCCEEDED`, stvarno **0.0500 / 0.0500 m** — nula pomaka | potvrđen C1: pozicijsko sučelje pod teretom ne pomiče vodilice **uopće**, ne samo djelomično |

## Zašto se činilo da rade

`scripts/joint_gui.py:170` objavljuje `sensor_msgs/JointState` **izravno na `/joint_states`** —
to je zamjena za `joint_state_publisher_gui` za RViz, bez ikakve fizike. Ondje naredba *jest*
stanje, pa se vodilice pomiču. Isti obrazac je i u referentnom projektu BATRACS
(`mock_components/GenericSystem`). Gazebo s DART fizikom je drugi slučaj.

## Trenutno rješenje
Na radnoj grani `feature/cube-manipulation` `torso_controller` šalje `effort` kroz PID.
Pokus je proveden u zasebnoj ROS domeni 75 i Gazebo particiji; skripta
`scripts/probe_torso.py` mjeri stvarne zglobove. Misijski `main_task` još podiže rukama,
pa [[D-09_lift_with_arms_not_torso]] ostaje opis starog misijskog slijeda dok se ne
integrira i ne provjeri dizanje kocke vodilicama.

## Sljedeći korak
Provjeriti da obje vodilice drže visinu tijekom obostranog kontaktnog hvata s kockom
i da im stvarne poze ne razmiču jastučiće. Tek tada zamijeniti dizanje rukama u misiji.

## Ne ponavljati
- Dalje dizati efort ili gain (probano do 1000 N).
