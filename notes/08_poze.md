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
# 6) neovisna provjera: iz vrhova meshova, vizualni vs kolizijski
./scripts/run_native.sh python3 scripts/mesh_extent.py
# 7) prikaz otvora u RViz-u (marker, pomice se s robotom)
./scripts/run_native.sh python3 scripts/door_gauge.py --sirina 1.00
```

> [!warning] Tablica niže nosi PROCJENU
> Stupac „Širina“ dolazi iz ishodišta linkova + pretpostavljenih 6 cm. Za odluke koristi
> `fit_test.py` ili `mesh_extent.py`. Za `ARM_CARRY_V2`: procjena 87 cm, **stvarno 85.4 cm**.
> U RViz-u za istinite omjere koristi pogled **TopDownOrtho** (perspektiva širi ruke, jer su
> bliže kameri od dna zidova).

| Poza | Širina | Vrata | Naprijed | Visina | Najširi link | Simetrična | Svrha | Snimljeno |
|---|---|---|---|---|---|---|---|---|
<!-- POSTURE ROWS -->
| `CARRY_V4` | 0.82 m | 0.92 m | 0.55 m | — | left_spherical_wrist_2 | ne | **nošenje kutije** (izmjereno s robota) | 2026-09-16 |
| `DRIVE_V4` | 0.83 m | 0.93 m | 0.31 m | 0.65–1.18 m | left_half_arm_1 | ne | spremljeno iz joint_gui | 2026-09-15 23:49 |
| `GRASP_V4` | 0.84 m | 0.94 m | 0.72 m | 0.84–1.27 m | left_spherical_wrist_2 | ne | spremljeno iz joint_gui | 2026-09-15 23:35 |
| `DETECTION_V4` | 1.38 m | 1.48 m | 0.72 m | 0.78–1.09 m | left_spherical_wrist_2 | ne | spremljeno iz joint_gui | 2026-09-15 23:32 |
| `GRASP_V3` | 0.84 m | 0.94 m | 0.71 m | 0.76–1.22 m | left_spherical_wrist_2 | ne | spremljeno iz joint_gui | 2026-09-15 22:45 |
| `GRASP_V2` | 0.92 m | 1.02 m | 0.66 m | 0.69–1.26 m | right_forearm | ne | spremljeno iz joint_gui | 2026-09-15 22:18 |
| `PREP_GRASP_V2` | 0.97 m | 1.07 m | 0.67 m | 0.71–1.26 m | right_spherical_wrist_2 | ne | spremljeno iz joint_gui | 2026-09-15 22:12 |
| `PREP_GRASP_V1` | 0.97 m | 1.07 m | 0.70 m | 0.83–1.26 m | right_spherical_wrist_2 | ne | spremljeno iz joint_gui | 2026-09-15 22:08 |
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
- **Stvarna širina: 85.4 cm.** Potvrđeno dvjema neovisnim metodama: `fit_test.py` (hodnik s
  prorezom u MoveIt sceni) daje 85.6 cm, a `mesh_extent.py` (vrhovi meshova kroz TF) 85.4 cm.
  Vizualna i kolizijska geometrija ruku su **identične**.
- Najširi je `right_spherical_wrist_2` (85.4), pa `forearm` (85.0), `half_arm_1` (83.4),
  `shoulder` (82.5) — cijela je ruka blizu granice.
- Uz pravilo „+10 cm“ → **vrata 95.4 cm**. Kroz 90 cm prolazi s 2.3 cm po strani; kroz **100 cm**
  ima 7.3 cm po strani (korisnik prihvatio 13. 9.). Vidi [[P-35_arm_span_too_wide_for_door]].
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

### PREP_GRASP_V1
spremljeno iz joint_gui

```python
# lijeva ruka
PREP_GRASP_V1_LEFT = {1: -0.073, 2: 1.504, 3: 2.285, 4: -1.552, 5: -0.882, 6: 1.619, 7: 1.630}
# desna ruka
PREP_GRASP_V1_RIGHT = {1: 0.052, 2: 1.463, 3: -2.272, 4: -1.596, 5: 0.899, 6: 1.628, 7: 1.506}
```
Širina 0.97 m → vrata 1.07 m; doseg naprijed 0.70 m; visina 0.83–1.26 m; najširi link `right_spherical_wrist_2_link`. Snimljeno 2026-09-15 22:08.

### PREP_GRASP_V2
spremljeno iz joint_gui

```python
# lijeva ruka
PREP_GRASP_V2_LEFT = {1: -0.239, 2: 1.768, 3: 2.153, 4: -1.212, 5: -0.743, 6: 2.020, 7: 1.732}
# desna ruka
PREP_GRASP_V2_RIGHT = {1: 0.209, 2: 1.726, 3: -2.147, 4: -1.260, 5: 0.752, 6: 2.033, 7: 1.398}
```
Širina 0.97 m → vrata 1.07 m; doseg naprijed 0.67 m; visina 0.71–1.26 m; najširi link `right_spherical_wrist_2_link`. Snimljeno 2026-09-15 22:12.

### GRASP_V2
spremljeno iz joint_gui

```python
# lijeva ruka
GRASP_V2_LEFT = {1: -0.318, 2: 1.751, 3: 2.373, 4: -1.208, 5: -0.855, 6: 2.020, 7: 1.744}
# desna ruka
GRASP_V2_RIGHT = {1: 0.301, 2: 1.714, 3: -2.368, 4: -1.249, 5: 0.862, 6: 2.030, 7: 1.390}
```
Širina 0.92 m → vrata 1.02 m; doseg naprijed 0.66 m; visina 0.69–1.26 m; najširi link `right_forearm_link`. Snimljeno 2026-09-15 22:18.

### GRASP_V3
spremljeno iz joint_gui

```python
# lijeva ruka
GRASP_V3_LEFT = {1: 0.243, 2: 2.088, 3: 0.909, 4: -1.314, 5: -0.946, 6: 1.824, 7: 2.367}
# desna ruka
GRASP_V3_RIGHT = {1: -0.320, 2: 2.076, 3: -0.897, 4: -1.361, 5: 0.974, 6: 1.874, 7: 0.740}
```
Širina 0.84 m → vrata 0.94 m; doseg naprijed 0.71 m; visina 0.76–1.22 m; najširi link `left_spherical_wrist_2_link`. Snimljeno 2026-09-15 22:45.

### DETECTION_V4
spremljeno iz joint_gui

```python
# lijeva ruka
DETECTION_V4_LEFT = {1: -0.191, 2: 1.317, 3: 2.008, 4: -0.667, 5: 4.763, 6: 1.964, 7: 2.463}
# desna ruka
DETECTION_V4_RIGHT = {1: 0.151, 2: 1.273, 3: -1.996, 4: -0.723, 5: 1.535, 6: 1.983, 7: 0.681}
DETECTION_V4_TORSO = {'torso_left_carriage_joint': 0.4, 'torso_right_carriage_joint': 0.4}
DETECTION_V4_GRIPPER = {'left_robotiq_85_left_knuckle_joint': 0.791, 'right_robotiq_85_left_knuckle_joint': 0.7918}
```
Širina 1.38 m → vrata 1.48 m; doseg naprijed 0.72 m; visina 0.78–1.09 m; najširi link `left_spherical_wrist_2_link`. Snimljeno 2026-09-15 23:32.

### GRASP_V4
spremljeno iz joint_gui

```python
# lijeva ruka
GRASP_V4_LEFT = {1: 0.574, 2: 1.429, 3: 1.549, 4: -1.045, 5: 4.464, 6: 1.544, 7: 2.652}
# desna ruka
GRASP_V4_RIGHT = {1: -0.634, 2: 1.391, 3: -1.525, 4: -1.104, 5: 1.847, 6: 1.574, 7: 0.498}
GRASP_V4_TORSO = {'torso_left_carriage_joint': 0.4, 'torso_right_carriage_joint': 0.4}
GRASP_V4_GRIPPER = {'left_robotiq_85_left_knuckle_joint': 0.791, 'right_robotiq_85_left_knuckle_joint': 0.7918}
```
Širina 0.84 m → vrata 0.94 m; doseg naprijed 0.72 m; visina 0.84–1.27 m; najširi link `left_spherical_wrist_2_link`. Snimljeno 2026-09-15 23:35.

### DRIVE_V4
spremljeno iz joint_gui

```python
# lijeva ruka
DRIVE_V4_LEFT = {1: 2.182, 2: 1.244, 3: 1.396, 4: -2.339, 5: 0.589, 6: -0.755, 7: 2.635}
# desna ruka
DRIVE_V4_RIGHT = {1: -2.255, 2: 1.203, 3: -1.278, 4: -2.395, 5: -0.504, 6: -0.769, 7: 0.536}
DRIVE_V4_TORSO = {'torso_left_carriage_joint': 0.2, 'torso_right_carriage_joint': 0.2}
DRIVE_V4_GRIPPER = {'left_robotiq_85_left_knuckle_joint': 0.791, 'right_robotiq_85_left_knuckle_joint': 0.7918}
```
Širina 0.83 m → vrata 0.93 m; doseg naprijed 0.31 m; visina 0.65–1.18 m; najširi link `left_half_arm_1_link`. Snimljeno 2026-09-15 23:49.

### CARRY_V4
Poza u kojoj robot **nosi kutiju** (korisnik, 16. 9.: „nazovi tu novu pozu carry i dozvoli gibanje
s njom"). Nije crtana ni računata: **očitana je s robota** na kraju hvata (run M2), nakon
`GRASP_V4` → privlačenje kutije 15 cm → laktovi 20° od torza. Vodilice na **0.10 m**, hvataljke
zatvorene na kutiji (0.791).

```python
# lijeva ruka
CARRY_V4_LEFT  = {1: 0.864, 2: 1.270, 3: 1.702, 4: -1.694, 5: -1.958, 6: 1.253, 7: 2.328}
# desna ruka
CARRY_V4_RIGHT = {1: -0.978, 2: 1.233, 3: -1.657, 4: -1.845, 5: 1.921, 6: 1.207, 7: 0.755}
CARRY_V4_CARRIAGE = 0.10
```

Širina **0.821 m** (izmjereno u runovima V5/M2) → kroz otvor od 0.980 m prolazi s **8 cm po strani**.
Gate pred vratima uspoređuje ruke s ovom pozom čim je kutija u rukama
([[P-45_mission_integration]]); prije toga s [[08_poze#DRIVE_V4|DRIVE_V4]].

> [!warning] `left_joint_5` je ovdje −1.958, a u `DRIVE_V4` +0.589
> Razlika je 2.547 rad i točno na tome je misija pala na vratima 16. 9.: gate je znao samo za
> `DRIVE_V4`. Uz to je gate uspoređivao kutove **bez 2π omota**, pa kontinuirani zglobovi mogu
> lažno ispasti daleko od cilja i kad su točno na njemu.
