---
id: D-11
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-07_ros2_humble_fortress_control]]"]
problems: ["[[P-01_shell_zenoh_contamination]]", "[[P-31_apt_upgrade_breakage]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-11: Projektni ROS okoliš (`run_native.sh`), neutralan globalni shell

## Kontekst
Globalni `~/.bashrc` je učitavao više workspaceova (`~/ws_moveit2` itd.), forsirao `rmw_zenoh` i
`ROS_DOMAIN_ID=5`. Posljedice: čvorovi su se rušili (router nedostupan) i ABI se nije slagao
(`libgeometric_shapes`, [[P-31_apt_upgrade_breakage]]).

## Odluka (10. 9.)
Globalni shell je neutralan. `scripts/run_native.sh` učitava samo `/opt/ros/humble` + lokalni
overlay; Fast DDS, domena 5, localhost-only; čisti naslijeđene putanje. Launch datoteke i dalje
forsiraju `rmw_fastrtps_cpp` (dvostruka zaštita).

## Posljedice
Svaki terminal za ovaj projekt se otvara s `./scripts/run_native.sh`. **Nikad** ne učitavati tuđi
`setup.bash` u isti shell.
