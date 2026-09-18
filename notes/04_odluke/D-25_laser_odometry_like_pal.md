---
id: D-25
type: odluka
status: predlozena
deviation: false
updated: 2026-09-18
requirements: ["[[R-08_omni_controller]]", "[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-20_place_at_destination]]"]
problems: ["[[P-52_no_margin_anywhere]]", "[[P-09_omni_drive_on_fortress]]"]
---
# D-25: `odom → base_footprint` iz lasera, kako to radi i PAL

## Odluka (predložena, radi se na grani `feat/laser-odometry`)
Prestati objavljivati `odom → base_footprint` iz odometrije kotača i objaviti ga iz
**laserske odometrije** (poravnavanje uzastopnih skenova). Odometrija kotača ostaje kao
izvor brzine i kao predikcija, ali prestaje biti izvor **poze**.

## Zašto — dokaz iz dva izvora

**1. Izmjereno kod nas** ([[P-52_no_margin_anywhere]], n=13, prema ground truthu):

| | naprijed | bočno |
|---|---|---|
| odometrija kotača | **0,8 mm** | **19,5 mm** |

Kotači mjere ono što se **otkotrlja**. Bočno ovaj pogon kliže — `mu2 = 0,20` je namjeran jer
Ignitionov `MecanumDrive` plugin ne radi ([[P-09_omni_drive_on_fortress]]) — a klizanje ne ostavlja
zakret za brojanje. `MecanumDriveController` računa punu 3-DOF odometriju i model mu je točan;
prekršena je **pretpostavka** o kotrljanju bez klizanja.

Iz te jedne činjenice slijedi sve što nas ruši: promašaj odlaganja ~20 mm, robotova nesposobnost
da to vidi (+15,4 mm, sd 0,2), i do 6 cm od osi u prolazu zbog čega gate odbija run.

**2. PAL je isti problem riješio isto tako.**
`/opt/ros/humble/share/omni_base_controller_configuration/config/mobile_base_controller.yaml`:

```yaml
# odom tf will be published by direct laser odometry
enable_odom_tf: false
```

Proizvođač te baze **izričito zabranjuje** kontroleru kotača da objavljuje tu transformaciju.
Njihovi kalibracijski množitelji (`wheel_radius_multiplier`, `wheel_separation_multiplier`,
`axis_separation_multiplier`) ispravljaju **geometriju kotrljanja**, ne klizanje — klizanje nema
konstantu kojom bi se pomnožilo. To potvrđuje da ovo nije naša greška u postavu nego svojstvo
pogona, i da je izlaz egzoceptivni senzor.

## Zašto ovo nije obično popuštanje granice
Ne dira se nijedna tolerancija ni zahtjev iz [ZAD]/[MAIL]. Mijenja se **izvor mjerenja** poze, i to
na onaj koji fizički robot također ima. Suprotno je od slijepih ulica iz [[AGENT_GUIDE]] §5: ondje
se prag spušta da run prođe, ovdje se robotu daje podatak koji mu je nedostajao.

## Stanje i prepreka
`rf2o_laser_odometry` i `laser_scan_matcher` **nisu** u zaključanom snapshotu (2026-08-07). Dostupni
su samo gradivni blokovi: `libpointmatcher`, `mp2p_icp`, `fast_gicp`, te `robot_localization` za
fuziju. Uvođenje paketa izvan snapshota vratilo bi nas u [[P-51_apt_upgrade_stops_amcl]], pa se
laserska odometrija piše **unutar projekta**, protiv `/scan_filtered`.

## Plan na grani
1. Čvor `laser_odometry`: poravnavanje uzastopnih skenova (2D), objavljuje `nav_msgs/Odometry`.
2. Mjerenje prema ground truthu iz **postojećih 50 runova** prije nego išta preuzme TF —
   bočna greška mora biti mjerljivo manja od 19,5 mm ([[D-18_verified_baseline_first]]).
3. Tek ako prođe: `cmd_vel_relay` prestaje premošćivati `/base_controller/tf_odometry` u `/tf`,
   a `odom → base_footprint` objavljuje novi čvor.
4. Serija od 20 runova, usporedba s `P-52` tablicom.

## Ako ne prođe
Ostaje zapisano kao izmjereni nalaz i preporuka za budući rad, s PAL-ovom konfiguracijom kao
potvrdom smjera. To je samo po sebi rezultat: usporedba vlastite arhitekture s proizvođačevom,
s brojkama.
