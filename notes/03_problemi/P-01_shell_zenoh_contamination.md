---
id: P-01
type: problem
status: rijeseno
requirements: ["[[R-07_ros2_humble_fortress_control]]"]
solutions: ["[[S-10_build_run_environment]]"]
decisions: ["[[D-11_project_scoped_ros_env]]"]
updated: 2026-09-13
---
# P-01: Kontaminiran shell (rmw_zenoh, tuđi workspaceovi) ruši čvorove

## Simptom
Svaki lokalni čvor se rušio ili nije vidio druge. Kasnije (rujan) je Panda/MoveIt pucao na
`libgeometric_shapes`.

## Uzrok
**Potvrđeno:** globalni `~/.bashrc` je postavljao `RMW_IMPLEMENTATION=rmw_zenoh_cpp` (router
nedostupan), `ROS_DOMAIN_ID=5` i učitavao `~/ws_moveit2`, `~/stage_ws`, `~/astro_ws`.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 12. 6. `7aaab94` | `SetEnvironmentVariable` Fast DDS + prazan `ZENOH_CONFIG_OVERRIDE` u svakom launchu | launch radi | pokriva launch, ali ne ručne naredbe |
| 2 | 10. 9. `31df80e` `eeefb28` `ebd392e` | neutralan `~/.bashrc` + `scripts/run_native.sh` (samo Humble + overlay, domena 5, localhost) | čisti rebuild 25/25 | **rješenje** |

## Trenutno rješenje
[[D-11_project_scoped_ros_env]]. Svaki terminal otvoriti s `./scripts/run_native.sh`.

## Ne ponavljati
- Postavljati RMW/domenu/workspace globalno u `~/.bashrc`.
- Učitavati tuđi `setup.bash` u isti shell.
