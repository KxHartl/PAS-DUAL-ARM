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
# 1) RViz sa sliderima (jednom)
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup display.launch.py
# 2) namjesti zglobove u Joint State Publisher prozoru, pa:
./scripts/run_native.sh python3 scripts/capture_posture.py ARM_DOOR "prolaz kroz vrata"
```
Skripta ispiše dimenzije i doda redak u tablicu ispod te blok s vrijednostima zglobova.

| Poza | Širina | Vrata | Naprijed | Visina | Najširi link | Simetrična | Svrha | Snimljeno |
|---|---|---|---|---|---|---|---|---|
<!-- POSTURE ROWS -->
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
