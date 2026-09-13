---
id: POZE
type: registar
updated: 2026-09-13
---
# Registar poza ruku (izmjereno, ne procijenjeno)

> Poze snima korisnik u RViz-u (`display.launch.py`, slideri), a sprema ih
> `scripts/capture_posture.py`. Svaka poza nosi **izmjerene** dimenzije iz TF-a, pa se zna što
> stvarno predstavlja. Širina = 2 × (max |y| linkova ruku + 0.06 m), najmanje 0.60 m (širina baze).
> „Vrata“ = širina + 10 cm (pravilo korisnika, 13. 9. 2026.).
> Vidi [[P-35_arm_span_too_wide_for_door]] i [[06_parametri]].

## Kako snimiti novu pozu
```bash
# 1) RViz (jednom)
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup display.launch.py
# 2) GUI sa STUPNJEVIMA i upisom vrijednosti (zamjenjuje joint_state_publisher_gui;
#    ugasi stari GUI da samo jedan objavljuje /joint_states)
./scripts/run_native.sh python3 scripts/joint_gui.py
# 3) namjesti zglobove, pa snimi pozu:
./scripts/run_native.sh python3 scripts/capture_posture.py ARM_DOOR "prolaz kroz vrata"
# 4) gabariti cijelog robota u trenutnoj pozi:
./scripts/run_native.sh python3 scripts/measure_robot.py
```
`capture_posture.py` ispiše dimenzije ruku i doda redak u tablicu ispod te blok s vrijednostima
zglobova. `measure_robot.py` mjeri **cijeli** robot (baza, torzo, ruke, kamera) i daje potrebnu
širinu vrata.

| Poza | Širina | Vrata | Naprijed | Visina | Najširi link | Simetrična | Svrha | Snimljeno |
|---|---|---|---|---|---|---|---|---|
<!-- POSTURE ROWS -->
| `ARM_CARRY_V2` | 0.85 m | 0.95 m | 0.72 m | 0.26–0.71 m | right_spherical_wrist_1 | ne | korisnikova poza: priprema za hvatanje kutije i nosenje | 2026-09-13 16:18 |
| `ARM_ZERO` | 2.28 m | 2.38 m | 0.13 m | 0.87–1.81 m | robotiq_85_left_finger_tip | da | referenca: svi zglobovi 0 (ruke ravno u stranu) | 2026-09-13 |

### ARM_ZERO
Nulta referenca: svi zglobovi ruku na 0. Ovako robot izgleda odmah nakon spawna u Gazebu, pa je to
i razlog zašto se prije bilo kakve vožnje ruke moraju složiti.

```python
ARM_ZERO_LEFT = {1: 0.000, 2: 0.000, 3: 0.000, 4: 0.000, 5: 0.000, 6: 0.000, 7: 0.000}
ARM_ZERO_RIGHT = {1: 0.000, 2: 0.000, 3: 0.000, 4: 0.000, 5: 0.000, 6: 0.000, 7: 0.000}
```
Širina 2.28 m → vrata 2.38 m; doseg naprijed 0.13 m; visina 0.87–1.81 m; najširi link
`right_robotiq_85_left_finger_tip_link`. Klizači torza bili su na 0.425 m (sredina), što mijenja
visine, ali ne i širinu.

## Poze iz koda (za usporedbu, izmjereno preko MoveIt FK 13. 9.)
| Poza | Širina | Vrata | Gdje je u kodu |
|---|---|---|---|
| `ARM_HOME` | 1.41 m | 1.51 m | `main_task.py:126` |
| `ARM_CARRY` | 1.29 m | 1.39 m | `main_task.py:131` |
| j2 = ±90°, j4 = ±90°, j6 = ∓70° (kandidat `ARM_DOOR`) | 0.85 m | 0.95 m | nije u kodu |

Donja granica širine je **~0.64 m** (sama ramena na y = ±0.26 + polumjer linka), bez obzira na pozu.

### ARM_CARRY_V2
Korisnikova poza (13. 9.): ovako se robot priprema za hvatanje kutije i tako je nosi.
**Gabariti cijelog robota u ovoj pozi** (`scripts/measure_robot.py`, referenca `base_footprint`):

| Skupina | Duljina X | Širina Y | Visina Z |
|---|---|---|---|
| baza | 0.76 m [−0.35, +0.41] | 0.57 m [−0.28, +0.28] | 0.02–0.35 m |
| torzo | 0.24 m | 0.36 m | 0.25–0.58 m |
| lijeva ruka | 0.84 m [−0.12, +0.72] | 0.45 m [−0.04, +0.41] | 0.34–0.79 m |
| desna ruka | 0.77 m [−0.06, +0.71] | 0.47 m [−0.43, +0.04] | 0.33–0.78 m |
| glava/kamera | 0.64 m | 0.58 m | 0.39–1.45 m |
| **UKUPNO** | **1.07 m** | **0.84 m** | **1.45 m** |

- Najdalje naprijed: `left_robotiq_85_left_finger_tip_link` (+0.66 m).
- Najširi: `right_spherical_wrist_1_link` (y = −0.37), pa `left_half_arm_1_link` (y = +0.35).
- **Potreban otvor: 0.94 m** (0.84 + 10 cm). Trenutna vrata su 0.9 m → **4 cm premalo**.
- Kamera je na 1.45 m. Zbog toga su zidovi 13. 9. podignuti s 1.2 m na **3.0 m**
  ([[P-36_walls_lower_than_camera]]).
- Ruke su u ovoj pozi **nesimetrične**, a najniža točka je 0.26 m (iznad ploče stola na 0.10 m).

```python
# lijeva ruka
ARM_CARRY_V2_LEFT = {1: 0.051, 2: 1.635, 3: 2.327, 4: -1.431, 5: -0.798, 6: 1.457, 7: 1.885}
# desna ruka
ARM_CARRY_V2_RIGHT = {1: 0.000, 2: 1.586, 3: 0.764, 4: 1.514, 5: 0.764, 6: -1.525, 7: -1.647}
```
Širina 0.85 m → vrata 0.95 m; doseg naprijed 0.72 m; visina 0.26–0.71 m; najširi link `right_spherical_wrist_1_link`. Snimljeno 2026-09-13 16:18.
