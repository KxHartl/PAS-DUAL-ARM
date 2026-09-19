---
id: P-56
type: problem
status: otvoreno
verified: "Izmjereno iz runa 19. 9. 2026., /cmd_vel + /joint_states + Gazebo istina."
updated: 2026-09-19
requirements: ["[[R-15_region_goal_nav2]]"]
solutions: ["[[S-06_navigation]]"]
problems: ["[[P-09_omni_drive_on_fortress]]"]
---
# P-56: brzina nije ograničena Nav2-om nego klizanjem kotača

## Nalaz
Mjereno na runu, vožnja ravno (`|vy| < 0,03`, `vx > 0,30`):

| | |
|---|---|
| Nav2 naređuje | **0,566 m/s** |
| sva četiri kotača | 6,09–6,13 rad/s → **0,466 m/s** obodno |
| robot se kreće | **0,230 m/s** |

Dva odvojena gubitka: kotači se vrte **18 % sporije** nego što naredba traži (za 0,566 m/s
trebalo bi 7,43 rad/s), a između oboda i poda gubi se **još 49 %**.

Kontroler pritom **javlja da uspijeva**: `/base_controller/odom` daje 0,583 m/s dok istina
kaže 0,267. Zato se gubitak nigdje ne vidi bez ground trutha.

## Uzrok
Mecanum valjci su emulirani **anizotropnim trenjem** (`mu1=0,80`, `mu2=0,20`, `fdir1` na ±45°),
jer se Ignitionov `MecanumDrive` plugin na Fortressu ne instancira ([[P-09_omni_drive_on_fortress]]).
Pri ±45° je i vožnja **naprijed** dijelom po osi s malim trenjem, pa kotač kliže po konstrukciji.

## Što iz toga slijedi
**Dizanje granica brzine u Nav2 ne daje ništa.** Izmjereno:

| `fast_vel_x` | trajanje runa |
|---|---|
| 0,45 | 576 s |
| 0,60 | 585 s |

Kotači se samo brže vrte o pod. Zato je `fast_vel_x` vraćen na 0,45, a razlog je zapisan uz
sam parametar.

## Gdje vrijeme zapravo odlazi
Iz vremenskih žigova koraka, run od 585 s:

| korak | trajanje |
|---|---|
| čekanje naredbe (režija sklopa, ne misija) | 60,0 s |
| vožnja u plavu sobu | 61,6 s |
| **hvat i dizanje** | **119,2 s** |
| **prijenos u crvenu sobu** | **143,4 s** |
| **traženje markera za odlaganje** | **54,2 s** |

Robot je **u pokretu 29 % runa**. Vožnja je 205 s od 456 s. Dvostruko brža vožnja skratila bi
run za ~17 %, a hvat i markeri zajedno (173 s) nitko nije optimizirao.

## Što bi stvarno dalo brzinu, po izgledima
1. **`mu1` naviše** — jedan broj, mjerljiv, ali mijenja fiziku na kojoj je sve dosad izmjereno,
   pa traži ponovno mjerenje odometrije, AMCL-a i hvata
2. **hvat (119 s) i traženje markera (54 s)** — usporedivi s vožnjom, a netaknuti
3. 60 s čekanja je režija `run_batch`, ne misija — ne računa se u isporuku
