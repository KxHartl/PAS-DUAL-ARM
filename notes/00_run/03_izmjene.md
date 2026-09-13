---
id: RUN_IZMJENE
type: upute
updated: 2026-09-13
---
# Gdje i kako mijenjati sustav

| Što | Datoteka | Nakon izmjene |
|---|---|---|
| Svijet, robot, senzori | `src/pas_dual_arm_bringup/worlds/`, `urdf/robot.urdf.xacro` | build bringup + restart simulacije; GUI i TF provjera |
| Kontroleri | `src/pas_dual_arm_bringup/config/controllers.yaml` | build bringup + restart; izmjeri `/joint_states` |
| Putna poza ruku | `src/pas_dual_arm_scripts/pas_dual_arm_scripts/postures.py` i [[08_poze]] | build scripts; ponovno izmjeri širinu i visinu |
| Sken | `scan_filter.py`, `config/bridge.yaml` | build pogođenog paketa; usporedi `/scan` i `/scan_filtered` |
| SLAM / tura | `config/slam_params.yaml`, `mapping_tour.py`, `base_drive.py` | build; novi run, odometrija i `map→odom` |
| AMCL / Nav2 | `config/nav2_params.yaml`, `launch/nav2.launch.py` | build bringup; novi start s valjanom kartom |
| Značajke | `feature_registry.py` | build scripts; potvrdi opažanja iz više pogleda |
| Zadatak | `main_task.py`, `launch/task.launch.py` | build oba paketa; test po fazama |

Nemoj mijenjati pozu samo zato što simulacijski kontroler ne drži naredbu:
uzrok i izmjerenu stvarnu geometriju prvo zabilježi u [[P-37_arm_position_gain_sag]].
Promjena kontrolerskog parametra ide i u [[06_parametri]]; svaki pokušaj u
P-tablicu i [[runovi]]. Kod se builda, ali uspjeh se potvrđuje zasebnim
simulacijskim runom. Postojeće necommitane izmjene u radnom stablu ne brisati.
