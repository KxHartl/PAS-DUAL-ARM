---
id: R-15
type: zahtjev
status: otvoreno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-06_navigation]]", "[[S-09_task_orchestration]]"]
problems: ["[[P-11_nav2_slam_drift]]", "[[P-33_nav2_undershoot_base_shift]]"]
decisions: ["[[D-04_visual_servo_instead_nav2]]"]
updated: 2026-09-13
---
# R-15: Korisnik zada približnu regiju, robot tamo ode (Nav2)

## Izvor (doslovno)
> „potom mu vi kažete otprilike regiju u koju da ode“ … „Za mapiranje i navigaciju koristiti
> nav2_stack i slam_toolbox.“ [MAIL]

## Tehnički znači
Operater zada cilj (npr. `/goal_pose` u RViz-u ili parametar regije), a Nav2 isplanira i odveze
bazu do regije. Tek tamo počinje traženje kutije ([[R-16_find_box]]).

**Kriterij prihvaćanja:**
- [ ] cilj regije zadaje čovjek (RViz „2D Goal Pose“ ili param), ne hardkodirano
- [ ] Nav2 `NavigateToPose` dovede bazu u regiju bez sudara
- [ ] nakon toga slijedi lokalno traženje kutije

## Trenutno stanje
❌ Trenutno robot **ne dobiva regiju**. `main_task` odmah skenira pan-tilt kamerom s mjesta i
visual-servoa do markera ([[D-04_visual_servo_instead_nav2]]). Nav2 `NavigateToPose` je radio u M6
(23. 6., `45c32f1`: nav do kutije, staging ispred vrata, prolaz). Parametri `pregrasp_xy`,
`door_xy`, `preplace_xy` su ostaci toga u `main_task.py`.

## Kako se rješava
- [[S-06_navigation]]: Nav2 konfiguracija
- [[S-09_task_orchestration]]: gdje u slijedu ide „idi u regiju“
