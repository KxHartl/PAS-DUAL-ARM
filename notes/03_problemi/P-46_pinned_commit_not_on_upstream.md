---
id: P-46
type: problem
status: rijeseno
requirements: ["[[R-07_ros2_humble_fortress_control]]", "[[R-21_deliverables]]"]
solutions: ["[[S-10_build_run_environment]]", "[[S-01_robot_description]]"]
decisions: ["[[D-11_project_scoped_ros_env]]"]
updated: 2026-09-16
---
# P-46 — Objavljeni repozitorij se nije dao buildati (pin na commit koji ne postoji na upstreamu)

## Simptom
Svjež klon s GitHuba, po uputama iz `README.md` (`vcs import` → `apply_patches.sh` →
`colcon build`), **ne izgradi nijedan paket**:

```
CMake Error at CMakeLists.txt:7 (find_package):
  By not providing "Findcatkin.cmake" ... asked CMake to find "catkin"
Failed   <<< pan_tilt_description
Failed   <<< pan_tilt_bringup
Summary: 0 packages finished
  2 packages failed, 10 packages aborted, 13 not processed
```

U našem radnom workspaceu sve radi. Kvar je vidljiv **isključivo** iz svježeg klona.

## Uzrok
**Potvrđeno.** `ros2.repos` je pinirao `pan_tilt_ros` na `b0f653428d…`, a to je **naš vlastiti
lokalni commit** („fix: add inertials and effort limits to pan-tilt links for Ignition"), koji
nikad nije gurnut ni na jedan upstream ref. Lanac posljedica:

1. `vcs import` ne može checkoutati commit koji na remoteu ne postoji;
2. **tiho** ostavlja zadanu granu repozitorija — a to je kod `pan_tilt_ros`-a `master`, **ROS 1
   (catkin)** grana, ne `humble-devel` (ament_cmake);
3. `colcon` pada na `find_package(catkin)` i ruši cijeli workspace;
4. usput nestaje i sam popravak iz tog commita: bez `<inertial>` blokova `urdf2sdf` izbaci
   pan-tilt linkove i sruši graf modela, pa `ign_ros2_control` ne krene, a uz effort `0.0`
   zglobovi nisu upravljivi.

Ostala četiri paketa su bila uredno pinirana na stvarne upstream commitove.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 16. 9. / `91fcbbf` | Klon objavljenog repoa u izoliranu mapu, `vcs import`, `apply_patches`, `colcon build` | ❌ **0 paketa**; `pan_tilt_description` i `pan_tilt_bringup` padaju na `find_package(catkin)`, 10 prekinuto | kvar postoji i reproducibilan je izvan našeg workspacea |
| 2 | 16. 9. | Usporedba checkoutanih commitova s pinovima iz `ros2.repos` | ❌ `pan_tilt_ros` na `01b26f3` umjesto `b0f6534`; ostala četiri točna | `vcs import` je tiho odstupio, bez greške |
| 3 | 16. 9. | `git cat-file -e b0f6534` u svježem klonu | ❌ `Not a valid object name` | pinani commit **ne postoji** na upstreamu |
| 4 | 16. 9. | `git branch -r --contains` nad svih pet paketa u našem workspaceu | `pan_tilt_ros` jedini **nije** ni na jednoj remote grani | commit je naš lokalni; ostali su čisti |
| 5 | 16. 9. | Provjera grane na koju se palo: `build_type` u `pan_tilt_description/package.xml` | `master` → catkin; `humble-devel` → `ament_cmake` | fallback je odveo na ROS 1 granu, odatle greška |
| 6 | 16. 9. / `9663b3b` | Re-pin na roditelja `9b08758` (postoji na `origin/humble-devel`) + naš popravak izvučen u `patches/pan_tilt_ros-inertials-and-effort-limits.patch` | ✅ `git apply --check` čist; obje zakrpe se primjenjuju | popravak se nosi kao i svaka druga izmjena tuđeg koda |
| 7 | 16. 9. / `9663b3b` | Ponovni izolirani test: klon → `vcs import` → `apply_patches` → `colcon build` | ✅ **25 paketa, 0 neuspjelih**; `verify_environment` **17/17**; `check_doors`/`check_zones` `PASS`; karta instalirana u `share/` | riješeno |

> [!note] Lažni `[FAIL]` u testiranju
> Jedan prolaz `verify_environment.sh` javio je 16/17. Uzrok je bio moj `PAS_DUAL_ARM_ROS_DOMAIN_ID=7`
> postavljen radi izolacije, a skripta provjerava da je domena **5**. Bez override-a: 17/17.

## Rješenje i prevencija
- Pin ide **isključivo** na commit koji postoji na upstreamu. Provjera:
  `git ls-remote <url> | grep <commit>` — mora nešto ispisati.
- Svaka naša izmjena tuđeg paketa ide u `patches/`, nikad kao lokalni commit u tom repozitoriju.
  Lokalni commit izgleda ispravno kod nas i pada **samo** onome tko klonira.
- Pravilo je upisano u zaglavlje `ros2.repos`.
- **Repozitorij se prije predaje testira iz svježeg klona**, ne iz radnog workspacea — vidi
  [[verify-remote-by-cloning]] obrazac u [[02_testiranje]].

Vezano: [[S-10_build_run_environment]], [[vanjski_paketi]].
