---
id: POZE
type: registar
updated: 2026-09-13
---
# Registar poza ruku (izmjereno, ne procijenjeno)

> Poze postavlja korisnik u RViz-u, a sprema ih `scripts/capture_posture.py`. Svaka poza nosi
> **izmjerene** dimenzije iz TF-a, pa se zna što stvarno predstavlja. Širina = 2 × (max |y| linkova
> ruku + 0.06 m), najmanje 0.60 m (širina baze). „Vrata“ = širina + 10 cm (pravilo korisnika,
> 13. 9. 2026.). Vidi [[P-35_arm_span_too_wide_for_door]] i [[06_parametri]].

## Kako snimiti novu pozu
```bash
# 1) RViz (jednom)
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup display.launch.py
# 2) GUI sa STUPNJEVIMA i upisom vrijednosti (zamjenjuje joint_state_publisher_gui;
#    ugasi stari GUI da samo jedan objavljuje /joint_states)
./scripts/run_native.sh python3 scripts/joint_gui.py
#    (--poza ARM_CARRY_V2 ucita spremljenu pozu iz ovog registra)
# 3) namjesti zglobove, pa snimi pozu:
./scripts/run_native.sh python3 scripts/capture_posture.py ARM_DOOR "prolaz kroz vrata"
# 4) gabariti cijelog robota u trenutnoj pozi (gruba procjena):
./scripts/run_native.sh python3 scripts/measure_robot.py
# 5) STVARNA sirina: najuzi prorez kroz koji robot prolazi (treba move_group)
./scripts/run_native.sh python3 scripts/fit_test.py --poza ARM_CARRY_V2
# 6) prikaz otvora u RViz-u (marker, pomice se s robotom)
./scripts/run_native.sh python3 scripts/door_gauge.py --sirina 1.00
```

> [!warning] Dvije brojke
> Tablica niže nosi **procjenu** (ishodišta linkova + 6 cm). Za odluke koristi `fit_test.py`, koji
> mjeri **stvarnom kolizijskom geometrijom**. Za `ARM_CARRY_V2`: procjena 87 cm, stvarno **83.4 cm**.

| Poza | Širina | Vrata | Naprijed | Visina | Najširi link | Simetrična | Svrha | Snimljeno |
|---|---|---|---|---|---|---|---|---|
<!-- POSTURE ROWS -->
| `ARM_CARRY_V2` | 0.87 m | 0.97 m | 0.70 m | 0.26–0.72 m | right_spherical_wrist_2 | ne | priprema za hvatanje kutije i nošenje | 2026-09-13 16:28 |
| `ARM_ZERO` | 2.28 m | 2.38 m | 0.13 m | 0.87–1.81 m | robotiq_85_left_finger_tip | da | referenca: svi zglobovi 0 | 2026-09-13 |

### ARM_CARRY_V2
Korisnikova poza (13. 9., finalna): ovako se robot priprema za hvatanje kutije i tako je nosi.
Vrijednosti su okrugle (višekratnici 45°), pa je poza lako ponovljiva.

```python
# radijani (za main_task.py)
ARM_CARRY_V2_LEFT  = {1: 0.000, 2: 1.571, 3: 2.356, 4: -1.571, 5: -0.785, 6: 1.571, 7: 1.571}
ARM_CARRY_V2_RIGHT = {1: 0.000, 2: 1.571, 3: 0.785, 4: 1.571, 5: 0.785, 6: -1.571, 7: -1.571}
# stupnjevi (za citanje)
# lijeva: j1 0, j2 90, j3 135, j4 -90, j5 -45, j6 90, j7 90
# desna:  j1 0, j2 90, j3  45, j4  90, j5  45, j6 -90, j7 -90
```

**Gabariti cijelog robota u ovoj pozi** (`scripts/measure_robot.py`, referenca `base_footprint`,
klizači na donjem limitu 0.05 m):

| Skupina | Duljina X | Širina Y | Visina Z |
|---|---|---|---|
| baza | 0.76 m [−0.35, +0.41] | 0.57 m [−0.28, +0.28] | 0.02–0.35 m |
| torzo | 0.24 m | 0.36 m | 0.25–0.58 m |
| lijeva ruka | 0.82 m [−0.12, +0.70] | 0.46 m [−0.04, +0.42] | 0.35–0.79 m |
| desna ruka | 0.76 m [−0.06, +0.70] | 0.48 m [−0.44, +0.04] | 0.34–0.78 m |
| glava/kamera | 0.60 m | 0.62 m | 0.40–1.45 m |
| **UKUPNO** | **1.04 m** | **0.85 m** | **1.45 m** |

- Najdalje naprijed: prsti desne hvataljke (+0.64 m).
- **Stvarni minimalni otvor: 83.4 cm** (mjereno `scripts/fit_test.py` pravim kolizijskim
  meshovima). Prvi dodiruje **`right_half_arm_1_link`** (nadlaktica uz rame), ne zapešće.
  Uz pravilo „+10 cm“ → **vrata 93.4 cm**; kroz sadašnjih 90 cm robot prolazi s 3.3 cm po strani.
  Brojka 0.85 m u tablici gore je gruba procjena (ishodišta + 6 cm) →
  [[P-35_arm_span_too_wide_for_door]].
- Ruke su **nesimetrične** (desna je zrcaljena, ne kopirana), najniža točka 0.26 m.
- Kamera je na 1.45 m; zato su zidovi podignuti na 3.0 m ([[P-36_walls_lower_than_camera]]).

### ARM_ZERO
Nulta referenca: svi zglobovi ruku na 0 — ovako robot izgleda odmah nakon spawna u Gazebu, pa je to
razlog zašto se prije bilo kakve vožnje ruke moraju složiti.

```python
ARM_ZERO_LEFT  = {1: 0.000, 2: 0.000, 3: 0.000, 4: 0.000, 5: 0.000, 6: 0.000, 7: 0.000}
ARM_ZERO_RIGHT = {1: 0.000, 2: 0.000, 3: 0.000, 4: 0.000, 5: 0.000, 6: 0.000, 7: 0.000}
```
Širina 2.28 m → vrata 2.38 m; doseg naprijed 0.13 m; visina 0.87–1.81 m; najširi link
`right_robotiq_85_left_finger_tip_link`. Klizači su tada bili na 0.425 m, što mijenja visine, ali
ne i širinu.

## Poze iz koda (za usporedbu, izmjereno preko MoveIt FK 13. 9.)
| Poza | Širina | Vrata | Gdje je u kodu |
|---|---|---|---|
| `ARM_HOME` | 1.41 m | 1.51 m | `main_task.py:126` |
| `ARM_CARRY` | 1.29 m | 1.39 m | `main_task.py:131` |
| `ARM_CARRY_V2` (gore) | 0.87 m | 0.97 m | još nije u kodu |

Donja granica širine je **~0.64 m** (sama ramena na y = ±0.26 + polumjer linka), bez obzira na pozu.
