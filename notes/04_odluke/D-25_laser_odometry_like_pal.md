---
id: D-25
type: odluka
status: vazeca
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

## Izmjereno (18. 9., offline na 10 snimljenih runova)
Zanos od ground trutha kroz cijeli run, u okviru robota:

| n=10 | naprijed | bočno |
|---|---|---|
| **laserska odometrija** | **0,9 mm** | **1,5 mm** |
| odometrija kotača | 30,1 mm | 22,3 mm |

Laser pobjeđuje u **svakom** runu na **obje** osi; najgori pojedinačni laserski rezultat je 8,3 mm
bočno, najbolji kotačima 4,1 mm a najgori **77,8 mm**. Prag iz ove odluke (ispod 19,5 mm bočno) je
prijeđen s velikom rezervom.

**Trošak u stvarnom vremenu:** 1078 točaka po skenu, podudaranje **19 ms** (max 25,8), dakle ~52 Hz
kapaciteta uz potrebnih ~4 Hz. Nije usko grlo.

### Tri iteracije, svaku presudilo mjerenje
| pristup | naprijed | bočno | zašto |
|---|---|---|---|
| sken-na-sken | 286 mm | 265 mm | jedna greška po skenu, 2205 ih se zbroji |
| + ključni skenovi (30 cm / 15°) | 22 mm | 22 mm | zbrajanje ograničeno na jedan po ključnom skenu |
| **+ točka-na-pravac** | **1,8 mm** | **8,3 mm** | ⟵ ključno, vidi niže |

**Zašto je zadnji korak toliko pomogao.** Podudaranje točka-na-točku smije **kliziti uzduž ravnog
zida**, jer ondje svaka točka jednako dobro odgovara susjedu kao ispravnom paru. Hodnik i prolaz su
uglavnom ravan zid, a klizanje u stranu je **točno greška koju lovimo** — slabost algoritma sjedila
je na istoj osi na kojoj je i slabost pogona. Bodovanje po **normali plohe** fiksira poklapanje
poprijeko zida, a uzduž njega ne tvrdi ništa.

## Plan na grani
1. Čvor `laser_odometry`: poravnavanje uzastopnih skenova (2D), objavljuje `nav_msgs/Odometry`.
2. ✅ **Napravljeno.** Mjerenje prema ground truthu iz postojećih runova, prije nego išta preuzme
   TF ([[D-18_verified_baseline_first]]) — tablica gore.
3. Tek ako prođe: `cmd_vel_relay` prestaje premošćivati `/base_controller/tf_odometry` u `/tf`,
   a `odom → base_footprint` objavljuje novi čvor.
4. Serija od 20 runova, usporedba s `P-52` tablicom.

## Ako ne prođe
Ostaje zapisano kao izmjereni nalaz i preporuka za budući rad, s PAL-ovom konfiguracijom kao
potvrdom smjera. To je samo po sebi rezultat: usporedba vlastite arhitekture s proizvođačevom,
s brojkama.
