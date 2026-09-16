---
id: S-10
type: rjesenje
status: ispunjeno
requirements: ["[[R-07_ros2_humble_fortress_control]]", "[[R-21_deliverables]]"]
problems: ["[[P-01_shell_zenoh_contamination]]", "[[P-31_apt_upgrade_breakage]]", "[[P-34_source_provenance]]", "[[P-46_pinned_commit_not_on_upstream]]"]
decisions: ["[[D-11_project_scoped_ros_env]]"]
files: ["scripts/run_native.sh", "ros2.repos", "scripts/apply_patches.sh", "patches/", "RUNNING.md", "README.md", "MAPPING.md"]
updated: 2026-09-16
---
# S-10: Build i pokretanje (projektni okoliš)

## Što radi
Reproducibilan build i izoliran ROS okoliš, bez ovisnosti o globalnom `~/.bashrc`.

## Dijelovi
- **`scripts/run_native.sh`**: učitava samo `/opt/ros/humble` + lokalni overlay; Fast DDS, ROS
  domena **5**, localhost discovery; čisti naslijeđene putanje ([[D-11_project_scoped_ros_env]]).
  `./scripts/run_native.sh` otvara shell, a `./scripts/run_native.sh <cmd>` pokreće naredbu.
- **`scripts/verify_environment.sh`**: BATRACS-style read-only preflight; provjerava ROS,
  overlay, alate i pakete; `--live` provjerava topice već pokrenute simulacije.
- **`ros2.repos`**: 5 upstream paketa (`aruco_ros`, `omni_base_simulation`, `pan_tilt_ros`,
  `realsense-ros`, `ros2_kortex`) pinanih na točan commit ([[P-34_source_provenance]]).
  **Pin mora postojati na upstreamu** (`git ls-remote <url> | grep <commit>`); lokalni commit
  izgleda ispravno kod nas, a ruši svjež klon ([[P-46_pinned_commit_not_on_upstream]]).
- **`patches/` + `apply_patches.sh`**: dvije zakrpe, idempotentno — Isaac-Sim argumenti iz
  Robotiq xacroa, te inercije i effort limiti pan-tilt linkova za Ignition.

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
| 16. 9. | **svjež klon s GitHuba** | ❌ 0 paketa — pin na lokalni commit odveo na ROS 1 granu | `91fcbbf` ([[P-46_pinned_commit_not_on_upstream]]) |
| 16. 9. | svjež klon, nakon popravka | ✅ 25 paketa, 0 neuspjelih; `verify_environment` 17/17; gate-ovi `PASS` | `9663b3b` |

## Otvoreno
- Repozitorij se prije predaje testira **iz svježeg klona**, ne iz radnog workspacea: kvar iz
  [[P-46_pinned_commit_not_on_upstream]] bio je vidljiv isključivo tako.
