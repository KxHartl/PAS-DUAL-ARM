---
id: P-31
type: problem
status: rijeseno
requirements: ["[[R-07_ros2_humble_fortress_control]]"]
solutions: ["[[S-10_build_run_environment]]", "[[S-01_robot_description]]"]
decisions: ["[[D-11_project_scoped_ros_env]]"]
updated: 2026-09-17
---
# P-31: `apt upgrade` je slomio okoliš

## Simptom
1. xacro greška `name 'gazebo_version' is not defined`.
2. MoveIt pada: source-build `~/ws_moveit2` linkan na `libgeometric_shapes.so.2.3.2`, a apt je
   donio 2.3.4.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 15. 7. | definirati `gazebo_version=gazebo` u `robot.urdf.xacro:19` | xacro radi | trajno |
| 2 | 15. 7. | kompatibilnosni symlink za `geometric_shapes` | radi | privremeni hack |
| 3 | 10. 9. | čisti rebuild **samo** na `/opt/ros/humble`, bez `~/ws_moveit2`, a symlink workaround uklonjen | 25/25 paketa | **rješenje** ([[D-11_project_scoped_ros_env]]) |
| 4 | 17. 9. | isti obrazac, drugi paket: `colcon build` pada s `No rule to make target '.../librealsense2.so.2.58.3'`. Sustav ima **2.58.4**, a `build/realsense2_camera` je zadržao putanju na **2.58.3** od prije nadogradnje | `rm -rf build/realsense2_camera install/realsense2_camera` pa rebuild → **25 paketa, 0 neuspjelih** | **rješenje**: obrisati zastarjeli CMake cache pogođenog paketa, ne cijeli `build/` |

## Obrazac (vrijedi za svaki paket)
Nakon `apt upgrade` koji promijeni verziju neke biblioteke, CMake cache u `build/<paket>/`
i dalje pokazuje na **staru** verziju datoteke koja više ne postoji. Poruka uvijek izgleda kao
`No rule to make target '<putanja>.so.<stara verzija>'`.

Lijek je uvijek isti i uvijek uzak:

```bash
rm -rf build/<paket> install/<paket>
./scripts/run_native.sh colcon build --symlink-install
```

Pogođeni paket u pravilu nije naš — 17. 9. je to bio `realsense2_camera`, RealSense **driver**,
koji ovom projektu nije potreban (potreban je samo `realsense2_description`, opis kamere). Ali
`colcon` nakon pada prekida i pakete koji su bili na redu, pa je ispalo da su i
`pas_dual_arm_bringup` i `pas_dual_arm_scripts` „pali“ iako s njima nije bilo ništa.

Brza provjera koja verzija stvarno postoji:
```bash
ls /opt/ros/humble/lib/x86_64-linux-gnu/librealsense2.so*
```

## Ne ponavljati
- Kompat-symlinkove za ABI; učitavanje `~/ws_moveit2` u ovaj projekt.
- Brisanje cijelog `build/` zbog jednog paketa — rebuild svega traje, a ne treba.
