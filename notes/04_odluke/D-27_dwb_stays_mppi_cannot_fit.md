---
id: D-27
type: decision
status: vazeca
verified: "Dva pokusa, 19. 9. 2026., oba prekinuta na prvoj dionici."
updated: 2026-09-19
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]"]
problems: ["[[P-55_doorway_skew_is_control]]", "[[P-56_speed_is_capped_by_slip]]"]
decisions: ["[[D-18_measure_do_not_argue]]"]
---
# D-27: MPPI ne može voziti robota koji jedva prolazi

## Odluka
Lokalni regulator ostaje **DWB**. MPPI je isproban i odbijen — ne zbog podešavanja nego zbog
geometrije.

## Zašto je uopće probavan
DWB ne ocjenjuje brzinu nijednim kritičarem. `PathDist` i `GoalDist` nagrađuju napredovanje duž
putanje unutar `sim_time`, pa je brzina samo posredna, a blizu cilja brža putanja preleti i
izgubi. Izmjereno: Nav2 traži punu brzinu u **24,2 %** naredbi, a medijan u otvorenom (0,143 m/s)
ispadne **niži** od medijana u vratima (0,162), jer su u otvorenom svi krajevi dionica.

MPPI ima `PathFollowCritic` koji želi robota **na putanji i u napredovanju**, i
`motion_model: "Omni"`, koji bočnu brzinu tretira kao upravljačku veličinu.

## Zašto ne ide
MPPI-jev `CostCritic` ocjenjuje putanju po **napuhanoj cijeni** ispod nje. Bez inflacijskog
sloja odbija sve:

```
No inflation layer found in costmap configuration
[follow_path] [ActionServer] Aborting handle.   (u petlji)
```

A kad se sloj doda, costmapa odbija polumjer manji od **upisanog polumjera robota**:

```
The configured inflation radius (0.150) is smaller than the computed inscribed radius
```

Upisani polumjer ovog tijela (0,83 × 0,72 m) je **~0,36 m**. Napuhavanje od 0,36 m s obje strane
otvora od **0,998 m** ostavlja **0,28 m** slobodno — a robot je širok **0,83 m**.

**Vrata bi bila zatvorena cijenom.**

## Zašto DWB može
`ObstacleFootprint` provjerava **stvarni obris** nad mrežom prepreka, bez napuhavanja. To je
jedini model koji radi za robota namjerno građenog da jedva prođe — i zato lokalna costmapa
nikad nije ni imala inflacijski sloj. To nije bio propust.

## Što bi trebalo da se MPPI ipak htjelo
`CostCritic` s `consider_footprint: true` provjerava obris umjesto cijene. Time gubi brzinu
zbog koje se uzima, i traži podešavanje svih osam kritičara od nule. Nije isprobano.

## Što je od tog pokusa ipak ostalo
- **replaniranje 1 Hz → 20 Hz** (spojene dionice 0,333 → 20); `expected_planner_frequency` je
  bio 20,0 cijelo vrijeme, dakle to nikad nije bila planerova granica
- **zakucavanje jezgri** (`scripts/pin_cores.sh`): `controller_server` na vlastitoj E-jezgri radi
  na **100 %** — računski je vezan, ne izgladnjen, i pripada P-jezgri
- mjerenje da **naša vlastita percepcija troši više od regulatora**: četiri ArUco detektora
  zajedno **179 %**, naspram 109 % regulatora
