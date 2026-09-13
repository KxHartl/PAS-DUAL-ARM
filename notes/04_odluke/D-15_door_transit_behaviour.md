---
id: D-15
type: odluka
status: zamijenjena
deviation: false
requirements: ["[[R-11_door_80cm]]", "[[R-14_slam_mapping]]", "[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
problems: ["[[P-12_door_too_narrow]]", "[[P-35_arm_span_too_wide_for_door]]", "[[P-11_nav2_slam_drift]]"]
superseded_by: "[[D-16_zones_from_detected_features]]"
updated: 2026-09-14
---
# D-15: Prolaz kao značajka + determinističko provlačenje (ZAMIJENJENA)

> [!warning] Zamijenjena s [[D-16_zones_from_detected_features]] (14. 9. 2026.)
> Ideja „prolaz je značajka" je preuzeta i implementirana, ali bez izlaska iz Nav2:
> vrata se detektiraju iz karte, a determinizam dolazi iz **zona** u costmapu umjesto iz
> vlastitog provlačenja. Determinističko poravnanje opisano ovdje ostaje plan B ako se
> uživo pokaže da 4.8 cm po strani nije dovoljno za DWB.

## Kontekst
Prolaz je uzak (0.9–0.95 m) u odnosu na robota (0.85 m s najužom pozom ruku,
[[P-35_arm_span_too_wide_for_door]]). Nav2 DWB je lokalni planer s uzorkovanjem: on kroz takav
otvor ulazi pod kutom, korigira usred prolaza i struže dovratak. Uz to costmap inflacija
(≥ upisanog radijusa 0.31 m, [[P-12_door_too_narrow]]) otvor od 0.9 m gotovo zatvori, pa planer
često uopće ne nađe put.

Zahtjev korisnika (13. 9.): vrata se **prepoznaju kao značajka**, a kroz njih se prolazi
**deterministički, po pravilima**, a ne prepušteno planeru.

## Prijedlog

### 1. Prepoznavanje vrata (značajka)
Nakon mapiranja ([[R-14_slam_mapping]]) iz zauzetosne karte izlučiti zidne segmente i u njima
**praznine širine 0.6–1.3 m**. Svaka praznina daje značajku:
```
door = { C: sredina otvora (x, y), n: normala zida (jedinična), w: širina otvora }
```
Provjera pri prilazu iz `/scan`: dvije najbliže prepreke lijevo i desno na očekivanoj udaljenosti,
razmak ≈ w. Ako se ne poklopi s kartom → abort, ne pogađati ([[D-12_honesty_abort_over_fake]]).
Vrata se drže u registru (`map` okvir), pa ih se može i vizualizirati u RViz-u kao markere.

### 2. Determinističko provlačenje (state machine, po korisnikovim pravilima)
| Faza | Što | Izlazni uvjet |
|---|---|---|
| **APPROACH** | Nav2 cilj u `C + 0.5·n` (0.5 m ispred otvora), yaw = −n (gleda kroz otvor) | Nav2 SUCCEEDED |
| **ALIGN** | okret u mjestu dok se uzdužna os ne poklopi s okomicom kroz središte otvora; bočni odmak od te okomice ≤ 2 cm | \|Δyaw\| ≤ 0.02 rad **i** \|Δy\| ≤ 0.02 m |
| **TRAVERSE** | **Nav2 se ne koristi**: ravna spora vožnja (v ≈ 0.10 m/s) izravno preko `cmd_vel`, bez okretanja | prijeđeno 0.5 m + duljina robota + 0.5 m |
| **EXIT** | 0.5 m iza otvora | predaja natrag Nav2 |

Tijekom TRAVERSE-a kontinuirano provjeravati **simetriju `/scan`-a** (najbliža prepreka lijevo vs
desno): razlika > 5 cm znači da robot ide ukoso → zaustaviti i abortirati, umjesto da se struže.
Ruke su cijelo vrijeme u `ARM_DOOR` pozi ([[P-35_arm_span_too_wide_for_door]]).

### 3. Zašto TRAVERSE zaobilazi Nav2
- DWB uzorkuje kutne brzine i u prolazu će ih koristiti; nama treba **čista ravna linija**.
- Costmap inflacija od 0.40 m proglasi cijeli otvor skupim; planiranje kroz njega je borba s
  costmapom.
- Već imamo provjereni alat: `main_task.drive()` tempiran **sim vremenom** i korigiran odometrijom
  ([[P-21_spin_once_not_pacing_rtf]]), a baza vozi ravno bez klizanja (okret i vožnja se nikad ne
  rade istovremeno, [[P-10_skid_steer_cannot_turn]]).

## Posljedice
- Nav2 ostaje zadužen za **putovanje po sobama** (i dalje ispunjava [[R-15_region_goal_nav2]]), a
  prolaz je posebno, determinističko ponašanje. To je standardan obrazac („door crossing behaviour“),
  ne zaobilaženje zahtjeva.
- Potrebna je nova komponenta: detektor vrata + state machine (procjena 150–250 linija u
  `pas_dual_arm_scripts`), plus `ARM_DOOR` poza.
- Rizik: ALIGN ovisi o kvaliteti `map → odom` iz SLAM-a. Ako lokalizacija skače
  ([[P-11_nav2_slam_drift]]), poravnanje treba raditi **iz `/scan`-a** (lokalno, geometrijski), ne
  iz karte.

## Odnos prema zahtjevima
Ispunjava [[R-18_door_pass_empty]] i [[R-19_door_pass_with_box]] uz zadržan Nav2/SLAM iz [MAIL].
**Preduvjet:** odluka o širini vrata (izmjereno 0.85 m + 10 cm = **0.95 m**) i `ARM_DOOR` poza.

## Status
**Nije implementirano** (13. 9., sesija identifikacije). Prije koda treba odluka korisnika o
širini vrata i prioritetu u odnosu na seminar ([[danas]]).
