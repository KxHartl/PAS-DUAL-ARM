---
id: P-37
type: problem
status: otvoreno
requirements: ["[[R-09_moveit_arm_control]]", "[[R-14_slam_mapping]]", "[[R-17_dual_arm_lift]]", "[[R-06_realistic_parameters]]"]
solutions: ["[[S-03_ros2_control_setup]]", "[[S-06_navigation]]", "[[S-08_grasp_squeeze_attach]]"]
decisions: []
updated: 2026-09-15
---
# P-37: Ruke ne drže naređenu pozu — MoveIt javi „stiglo", a ruka visi

## Simptom
Nakon što je MoveIt javio `ARM_CARRY_V2: OK`, izmjereni zglobovi bili su **do 1.148 rad (66°)**
od naređenih. Šaka je zbog toga visjela toliko nisko da su **vrhovi prstiju ušli u ravninu
lidara** (0.2086 m), pa je svaki sken nosio lažnu prepreku ~0.55 m ispred robota i autonomna
tura se zaustavila na vlastitim hvataljkama.

## Uzrok
**Potvrđeno čitanjem proširenog URDF-a.** `ign_ros2_control` **ne drži** poziciju — pretvara je
u brzinu:

```
joint_velocity = position_proportional_gain × (naredeno − stvarno)
```

Komentar u `robot.urdf.xacro:300-301` to već opisuje za torzo, i torzo zato ima gain **20.0**.
Ali zglobovi ruku dolaze iz `kortex_description` (`load_robot`), koji ih emitira kao goli
`<command_interface name="position">` **bez ikakvog gaina** → vrijedi tvornički, vrlo mekan.
Ruka puzi prema cilju, gravitacija je vuče natrag, i ravnoteža se uspostavi daleko od cilja.

Zato najviše otpadaju **zglobovi zapešća** (najveći gravitacijski moment za dani kut), i zato je
odstupanje **asimetrično**: u `ARM_CARRY_V2` je lijevi `j6 = +1.571`, a desni `j6 = −1.571`, pa
ih gravitacija ne opterećuje jednako.

## Zašto je ovo važno šire od SLAM-a
Ako ruka ne stoji tamo gdje je naređeno, onda su **sve** provjere koje uspoređuju naredbu sa
stvarnošću mjerile ovaj kvar, a ne ono što su mislile da mjere. To je vjerojatan zajednički
uzrok iza „IK lutrije", `settle-wait`-a i asimetričnih sweet-spotova
([[P-24_press_path_chain]], [[P-25_asymmetric_arm_reach]], [[P-28_gate_too_strict]]).
Te kartice treba ponovno vrednovati kad se hvat bude radio.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 13. 9. | izmjereni zglobovi nakon `ARM_CARRY_V2: OK` | 4 zgloba odstupaju, najgori **1.148 rad**; prsti u ravnini skena | MoveIt-ov „OK" ne znači da je ruka stigla |
| 2 | 13. 9. | provjera je li kriva poza ili kontrola: pročitan prošireni URDF | zglobovi ruku **nemaju** `position_proportional_gain`, torzo ima 20.0 | uzrok je kontrola, ne poza |
| 3 | 13. 9. | gain **20.0** ubrizgan u sve `position` command_interface (regex u `sim.launch.py`, isti obrazac kojim se već brišu negativne skale) | odstupaju **2 zgloba**, najgori 0.612 rad; desna ruka do 0.016 rad; **sken čist** | veliko poboljšanje, dovoljno za SLAM |
| 4 | 13. 9. | čista headless simulacija, `ARM_CARRY_V2`, zatim autonomna tura; stvarna širina izmjerena `mesh_extent.py` nakon gibanja | **1.109 m**, najširi lijevi vrh prsta; TF vrha na y=0.537 m; vrata su 1.0 m | gain 20 **nije dovoljan** tijekom gibanja; poza je fizički neprolazna i status ostaje otvoren |
| 5 | 13. 9. | provjera stvarnog ROS parametra `/gz_ros2_control/position_proportional_gain` te Humble izvornog koda; pokusi s YAML-om i SDF child tagom | parametar ostaje **0.1**; plugin ignorira `command_interface` gain i SDF child; uveden prekid vožnje na odstupanju te otpuštanje kontrolera ruku tijekom vožnje, ali stabilnost još nije potvrđena | raniji zaključak da je gain 20 radio bio je pogrešan; novi način držanja treba izmjeriti u vožnji |
| 6 | 13. 9. | korisnički GUI run `mapping_tour` (run 41) | MoveIt/JTC `SUCCEEDED`; pet sekundi nakon otpuštanja kontrolera `left_joint_6` +0.614 rad od cilja; kontroleri vraćeni; baza nije dobila naredbu ture | sigurnosni prekid radi; ciljna poza nije fizički održana |
| 7 | 15. 9., radno stablo | `effort` sučelje s PID-om u JTC-u umjesto pozicijskog (`force_grasp:=true`), pojačanja iz izmjerene inercije ([[P-41_effort_pid_arm_actuator_profile]]) | obje ruke stigle na pred-hvat u **8 mm**, staging javio `Goal reached` za obje; u mirovanju svi zglobovi **točno** na naredbi, brzina 0.0000 | Uzrok iz ove kartice (plugin pretvara poziciju u brzinu uz gain 0.1) **zaobiđen** dok je `force_grasp` profil aktivan. Za zadani `position` profil kartica ostaje otvorena |
| 8 | 15. 9., radno stablo | **Ispravak pokušaja 5.** Plugin **ne** ignorira gain — `libgz_hardware_plugins.so` ga podržava i ispisuje `The position_proportional_gain has been set to:` **jednom po `ros2_control` komponenti**. Provjereno: (a) `<param>` na zglobu, (b) yaml pod ključem `gz_ros2_control:`, (c) yaml pod wildcardom `/**:`, (d) `ros2 param set` u letu | (a), (b), (c) → plugin i dalje javlja **0.1** za svih sedam komponenti; `<parameters>` datoteka ne stiže do tog čvora. (d) parametar **primljen** (`ros2 param get` vraća 20.0), ali vodilice se ne pomaknu: `probe_torso --height 0.20` daje `before 0.0500 / after 0.0500, error 0.15 m` | Gain je parametar **čvora plugina**, ne zgloba, i čita se **jednom pri inicijalizaciji**. Nijedna podržana SDF oznaka (`parameters`, `namespace`, `remapping`, `controller_manager_name`, `robot_param`, `robot_param_node`) ne omogućuje da se do njega dođe. Pozicijsko sučelje je time iscrpljeno za teret |

## Trenutno rješenje
Ranije ubrizgavanje `position_proportional_gain = 20.0` u `command_interface`
uklonjeno je iz `sim.launch.py`: instalirani Humble plugin ga ne čita. Stvarni
parametar plugin čvora ostao je 0.1. `mapping_tour` sada zaustavlja vožnju ako
stvarni zglobovi odstupaju od `ARM_CARRY_V2`; novi postupak otpuštanja kontrolera
za vožnju nije još potvrđen mjerenjem širine u pokretu.

## Otvoreno
- Lijevi `j5`/`j6` još odstupaju; tijekom vožnje ruka se raširi na 1.109 m.
  Najprije riješiti držanje **postojeće** putne poze, pa izmjeriti širinu u vožnji.
- Razjasniti podržan način postavljanja plugin parametra pri stvaranju čvora;
  prethodni YAML/SDF pokusi nisu promijenili runtime vrijednost.

## Ne ponavljati
- Vjerovati da je ruka u pozi zato što je MoveIt vratio SUCCEEDED. **Izmjeriti `/joint_states`.**
- Tražiti uzrok „čudnog ponašanja ruke" u planiranju prije nego se provjeri drži li kontroler
  poziciju uopće.
