---
id: P-49
type: problem
status: rijeseno-ceka-run
verified: "uzrok pročitan iz logova runova 1 i 2 serije 2026-09-18_0130; popravak još nije vožen"
updated: 2026-09-18
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
solutions: ["[[S-06_navigation]]"]
decisions: []
---
# P-49: prazan `goal_checker_id` prestane raditi kad postoje dva provjerivača cilja

## Simptom
Prva serija ispitnog sklopa (18. 9., `validation/results/2026-09-18_0130/`): runovi 1 i 2
**oba** prekinuti na istom mjestu, identično i u sekundu (139,1 s):

```
room_navigator: leg 1/4 - home -> blue: line up at the doorway -> (-0.00, -1.84, -90.0 deg)
bt_navigator:   Goal failed
room_navigator: ... Nav2 finished with status 6
main_task:      Task aborted during: room_navigator did not reach blue:dock
```

Robot se **nije pomaknuo ni centimetar**: `loc_error` cijelo vrijeme pokazuje dx +0,8 cm,
dy −1,6 cm, dyaw 0,4° bez promjene, a `arrivals = 0`.

## Uzrok
U logu, od prve sekunde vožnje pa do aborta, svakih ~10 s:

```
controller_server: FollowPath called with goal_checker name  in parameter
'current_goal_checker', which does not exist. Available goal checkers are:
general_goal_checker explore_goal_checker .
[follow_path] [ActionServer] Aborting handle.
```

Ime je **prazno**. Nav2-ov standardni `navigate_to_pose` BT poziva
`<FollowPath path="{path}" controller_id="FollowPath"/>` — **bez** `goal_checker_id`.
`ControllerServer::findGoalCheckerId` prazno ime prihvaća samo ako je učitan **točno jedan**
provjerivač cilja; tada uzima taj jedan. Čim ih je dva, prazno ime više ništa ne označava i
svaki `FollowPath` biva odbijen. BT to vrti kroz recovery grane dok `navigate_to_pose` ne padne
sa statusom 6.

**Regresija je naša i datira od commita `9af9027` (17. 9.)**, gdje je uz mapiranje dodan drugi
provjerivač:

```diff
-    goal_checker_plugins: ["general_goal_checker"]
+    goal_checker_plugins: ["general_goal_checker", "explore_goal_checker"]
```

Istraživački BT (`behavior_trees/explore_to_pose.xml`) svoj provjerivač **imenuje**, pa je
mapiranje radilo; misija je vozila na Nav2-ovom standardnom stablu i tiho ostala bez upravljanja.
Isto objašnjava i „Nav2 vraća `ABORTED` na pozi poravnanja, bez ijedne poruke u logu" iz
[[D-23_coverage_sweep_instead_of_frontier]] — poruka je postojala, ali u `controller_server`, a ne
u navigatoru.

## Popravak
`src/pas_dual_arm_bringup/behavior_trees/navigate_to_pose_mission.xml` — Nav2-ovo stablo s jednom
razlikom:

```xml
<FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="general_goal_checker"/>
```

i u `nav2_params.yaml`:

```yaml
default_nav_to_pose_bt_xml: $(find-pkg-share pas_dual_arm_bringup)/behavior_trees/navigate_to_pose_mission.xml
```

Zašto tako, a ne micanjem drugog provjerivača: ovako **svaki** `NavigateToPose` cilj imenuje
provjerivača — i misijski, i onaj iz RViz-a („2D Goal Pose") — pa navigacija prestaje ovisiti o
tome koliko je provjerivača slučajno učitano. Micanje `explore_goal_checker`-a vratilo bi misiju,
ali bi obeskorijenilo istraživački BT.

## Tablica pokušaja
| # | Datum | Što je pokušano | Ishod |
|---|---|---|---|
| 1 | 17. 9. | dodan `explore_goal_checker` za mapiranje (`9af9027`) | mapiranje dobilo svoje tolerancije; **misija ostala bez upravljanja** (tada neprimijećeno) |
| 2 | 18. 9. | serija `2026-09-18_0130`, runovi 1–2 | `aborted` na `leg 1/4`, robot se ne miče; uzrok pročitan iz `controller_server` |
| 3 | 18. 9. | vlastiti BT s `goal_checker_id="general_goal_checker"` + `default_nav_to_pose_bt_xml` | zamjena putanje provjerena izvan simulatora (`ParameterFile` + `RewrittenYaml` razrješuje `$(find-pkg-share …)` u instaliranu datoteku); **vožnja još nije napravljena** |

## Otvoreno
Ponoviti V1 iz [[validacija]] (tri runa). Tek ako prođu, brojke iz serije imaju smisla.
Serija `2026-09-18_0130` je **neupotrebljiva kao mjera sustava** i služi samo kao dokaz ovog kvara.
