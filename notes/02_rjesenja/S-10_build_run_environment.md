---
id: S-10
type: rjesenje
status: ispunjeno
requirements: ["[[R-07_ros2_humble_fortress_control]]", "[[R-21_deliverables]]"]
problems: ["[[P-01_shell_zenoh_contamination]]", "[[P-31_apt_upgrade_breakage]]", "[[P-34_source_provenance]]"]
decisions: ["[[D-11_project_scoped_ros_env]]"]
files: ["scripts/run_native.sh", "ros2.repos", "scripts/apply_patches.sh", "patches/", "RUNNING.md", "README.md"]
updated: 2026-09-13
---
# S-10: Build i pokretanje (projektni okoliš)

## Što radi
Reproducibilan build i izoliran ROS okoliš, bez ovisnosti o globalnom `~/.bashrc`.

## Dijelovi
- **`scripts/run_native.sh`**: učitava samo `/opt/ros/humble` + lokalni overlay; Fast DDS, ROS
  domena **5**, localhost discovery; čisti naslijeđene putanje ([[D-11_project_scoped_ros_env]]).
  `./scripts/run_native.sh` otvara shell, a `./scripts/run_native.sh <cmd>` pokreće naredbu.
- **`ros2.repos`**: 5 upstream paketa (`aruco_ros`, `omni_base_simulation`, `pan_tilt_ros`,
  `realsense-ros`, `ros2_kortex`) pinanih na točan commit ([[P-34_source_provenance]]).
- **`patches/` + `apply_patches.sh`**: jedna zakrpa (Isaac-Sim argumenti iz Robotiq xacroa),
  idempotentno.

## Standardni run
```bash
./scripts/run_native.sh colcon build --symlink-install
# T1
./scripts/run_native.sh && ros2 launch pas_dual_arm_bringup sim.launch.py   # GUI
# T2
./scripts/run_native.sh && ros2 launch pas_dual_arm_bringup task.launch.py
```

## Provjereno
| datum | način | što | commit |
|---|---|---|---|
| 10. 9. | čisti rebuild | 25/25 paketa na `/opt/ros/humble` | `31df80e`, `eeefb28` |
| 10. 9. | vcs import | provenance + zakrpa | `f046e98` |

## Otvoreno
- Nakon 10. 9. **nije bilo** GUI runa misije u novom okolišu. Prvi run danas je ujedno i test
  okoliša.
