# PICK AND PLACE - AUTONOMOUS OBJECT MANIPULATION

## 📋 Pregled

Pick and Place je završni zadatak koji kombinira sve prethodne vježbe i zadatke:
- **Zadaća 2**: YOLO detekcija voća
- **Zadaća 3**: Point Cloud obrada i 3D lokalizacija
- **Vjezba Trajektorija**: Generiranje glatkih trajektorija
- **Finalni Projekt**: Autonomna hvatanja i odlaganja

## 🎯 5 Obaveznih Točaka Trajektorije

```
1. HOME           → Početna sigurna pozicija
2. APPROACH       → Iznad objekta (100mm offset po Z)
3. PICK           → Hvatanja objekta (iz point cloud obrade)
4. APPROACH_PLACE → Iznad mjesta odlaganja (100mm offset)
5. PLACE          → Odlaganja objekta (fiksna)
```

## 📁 Struktura Foldera

```
for_pick_and_place/
├── 📄 README.md                              # Ovaj fajl
├── 📄 ANALIZA_VJEZBE_I_ZADATKA.md           # Detaljne informacije
├── 📄 requirements_pick_place.txt            # Dependencije
│
├── 📂 scripts/
│   ├── 01_generate_pick_place_trajectory.py  # Geneiriranje trajektorija
│   ├── 02_robot_communication.py             # Slanje na robota
│   ├── 03_visualize_trajectory.py            # Vizualizacija
│   ├── 04_integrate_with_tasks.py            # Integracija s 3D lokalizacijom
│   ├── 05_quintic_interpolation_reference.py # Referenca iz vjezbe
│   ├── 06_task_space_reference.py            # Referenca iz vjezbe
│   └── 07_send_trajectory_reference.py       # Referenca iz vjezbe
│
├── 📂 reference_points/
│   ├── home_configuration.json               # HOME točka
│   ├── place_locations.json                  # PLACE točke
│   └── reference_points.json                 # Sve 5 točaka
│
├── 📂 trajectories/
│   ├── trajectory_HOME_to_APPROACH.txt
│   ├── trajectory_APPROACH_to_PICK.txt
│   ├── trajectory_PICK_to_APPROACH_PLACE.txt
│   ├── trajectory_APPROACH_PLACE_to_PLACE.txt
│   ├── trajectory_PLACE_to_HOME.txt
│   └── example_fifth_order.txt               # Primjer iz vjezbe
│
├── 📂 output/
│   ├── generated_trajectories.zip            # Sve trajektorije
│   ├── reference_points.json                 # Korištene točke
│   ├── simulated_execution.log               # Simulacijski log
│   └── robot_execution.log                   # Pravi log (ako je izvršeno)
│
└── 📂 docs/
    ├── datasheets/                           # UR specifikacije
    └── trajectory_task_specification.md      # Specifikacija zadatka
```

## 🚀 Brzi Start

### 1. Instalacija Dependencija
```bash
cd for_pick_and_place
pip install -r requirements_pick_place.txt
```

### 2. Definiranje Referentnih Točaka

Kreiraj `reference_points.json`:
```json
{
  "HOME": [-1.571, -2.094, -1.571, 0.524, 1.571, 0.000],
  "PICK": [0.3, 0.2, 0.5, 3.14159, 0.0, 0.0],
  "PLACE": [0.2, 0.4, 0.5, 3.14159, 0.0, 0.0]
}
```

### 3. Geneiriranje Trajektorija

```python
from scripts.generate_pick_place_trajectory import ReferencePointsManager, TrajectoryPlanner

# Učitaj točke
pm = ReferencePointsManager()
pm.load_from_json("reference_points.json")

# Izračunaj APPROACH točke (offset 100mm)
pm.compute_approach_points(offset=0.1)

# Generiraj trajektorije
planner = TrajectoryPlanner()
trajectories = planner.generate_pick_and_place_trajectory(pm.get_all_points())

# Spremi u datoteke
for seg_name, seg_data in trajectories.items():
    planner.save_trajectory_to_file(seg_data['trajectory'], f"trajectories/trajectory_{seg_name}.txt")
```

### 4. Pokretanje Simulacije

```bash
python scripts/01_generate_pick_place_trajectory.py
```

### 5. Slanje na Robota

```bash
python scripts/02_robot_communication.py
```

## 📊 Ključni Moduli

### 1. **01_generate_pick_place_trajectory.py**

Geneiriranje trajektorija između 5 ključnih točaka.

**Ključne klase:**
- `ReferencePointsManager` - Upravljanje referentnim točkama
- `TrajectoryPlanner` - Geneiriranje trajektorija

**Primjer:**
```python
from scripts.generate_pick_place_trajectory import ReferencePointsManager, TrajectoryPlanner

pm = ReferencePointsManager()
pm.set_home(q_home)
pm.set_pick_from_point_cloud(pick_3d_coords)
pm.set_place(place_coords)
pm.compute_approach_points(offset=0.1)

planner = TrajectoryPlanner()
trajectories = planner.generate_pick_and_place_trajectory(pm.get_all_points())
```

### 2. **02_robot_communication.py**

TCP/IP komunikacija s UR robotom.

**Ključne klase:**
- `URRobotCommunication` - Slanje trajektorija robotu

**Primjer:**
```python
from scripts.robot_communication import URRobotCommunication

robot = URRobotCommunication(host='192.168.1.100', port=30003)
robot.connect()
robot.send_trajectory_from_file("trajectories/trajectory_HOME_to_APPROACH.txt")
robot.disconnect()
```

### 3. **03_visualize_trajectory.py** (trebam kreirati)

Vizualizacija trajektorija u 3D.

### 4. **04_integrate_with_tasks.py** (trebam kreirati)

Integracija s rezultatima zadaće 3.

## 🔗 Integracija s Prethodnim Zadacima

### Tijek Podataka:
```
┌─────────────────────────────────────────────────────────────┐
│                 KOMPLETAN PIPELINE                          │
├─────────────────────────────────────────────────────────────┤

1. YOLO DETEKCIJA (Zadaća 2)
   └─→ Detektira voće u sceni

2. POINT CLOUD OBRADA (Zadaća 3)
   └─→ Lokalizira objekta (3D koordinate u robot sustavu)
       └─→ Output: PICK točka (x, y, z)

3. TRAJEKTORNA VJEŽBA (Vjezba Trajektorija)
   └─→ Naučeni koncepti za geneiriranje trajektorija
       └─→ Quintic interpolacija, task space, ograničenja

4. PICK & PLACE (Finalni Projekt)
   ├─→ Učitaj PICK koordinate iz zadaće 3
   ├─→ Definiraj PLACE točku
   ├─→ Generiraj 5 segmentnih trajektorija
   ├─→ Primijeni quintic interpolaciju
   ├─→ Validiraj ograničenja robota
   └─→ Pošalji na robota (TCP/IP)

5. ROBOTSKA IZVRŠENJA
   └─→ Autonomna hvatanja i odlaganja objekta
```

## ⚙️ Konfiguracija

### Robot Parametri

```python
# UR10 ograničenja (primjer)
v_max = 0.5          # m/s (maksimalna translacijska brzina)
a_max = 1.0          # m/s² (maksimalna translacijska akceleracija)
omega_max = 1.0      # rad/s (maksimalna kutna brzina)

# Zglobna ograničenja
q_dot_max = 360      # °/s
q_ddot_max = 720     # °/s²
```

### APPROACH Offset

```python
# Fiksni offset 100mm iznad objekta
APPROACH = PICK + [0, 0, 0.1]  # U metrima
APPROACH_PLACE = PLACE + [0, 0, 0.1]
```

## 📈 Očekivani Output

### 1. Referentne Točke
```json
{
  "HOME": [q1, q2, q3, q4, q5, q6],
  "APPROACH": [x, y, z+0.1, rx, ry, rz],
  "PICK": [x, y, z, rx, ry, rz],
  "APPROACH_PLACE": [x, y, z+0.1, rx, ry, rz],
  "PLACE": [x, y, z, rx, ry, rz]
}
```

### 2. Trajektne Datoteke
Svaka datoteka sadrži N točaka s 6 stupnjeva slobode:
```
t    q1      q2      q3      q4      q5      q6
0.0  -1.571  -2.094  -1.571  0.524   1.571   0.000
0.002 -1.565 -2.095  -1.570  0.525   1.571   0.001
...
0.5   -0.785  -2.094  0.000  -1.571  0.000   0.000
```

### 3. Vizualizacije
- 3D putanja TCP-a s 5 ključnih točaka
- Grafovi brzina i akceleracija
- Validacija ograničenja

### 4. Simulacijski Log
```
SIMULACIJA IZVRŠENJA TRAJEKTORIJE
Vrijeme: 2026-05-20 11:54:10
Broj točaka: 500
Izvršavanje:
   0: [-1.5708, -2.0944, -1.5708,  0.5236,  1.5708,  0.0000]
   1: [-1.5651, -2.0952, -1.5704,  0.5248,  1.5708,  0.0009]
   ...
```

## ⚠️ Važne Napomene

✅ **Bez Blendinga Kutova** - Robot mora prolaziti kroz sve 5 točaka
✅ **Glatke Trajektorije** - Koristi quintic (5. red polinom)
✅ **Dovoljnа Diskretizacija** - Min 100+ točaka po segmentu
✅ **Respektuj Ograničenja** - Brzine, akceleracije, workspace
✅ **APPROACH Sigurnost** - Min 100mm iznad objekta
✅ **Transformacije** - Paziti na koordinatne sustave

## 🔧 Troubleshooting

| Problem | Rješenje |
|---------|----------|
| Robot nije dostupan | Provjeri IP i port, koristi simulaciju |
| Trajektorija je prekratka/predugačka | Prilagodi `v_max`, `a_max` |
| Objekt se ne hvata ispravno | Provjeri PICK koordinate iz zadaće 3 |
| Greške pri transformaciji | Validiraj koordinatne sustave |

## 📚 Dodatni Resursi

- UR dokumentacija: [Universal Robots](https://www.universal-robots.com)
- UR+ Ecosystem: Komercijalni programi i aplikacije
- Robotics Toolbox: Napredni alati za kinematiku
- ROS: Robot Operating System za kompleksnije sustave

## ✅ Checklist za Submisiju

- [ ] HOME točka definirana
- [ ] PICK točka učitana iz zadaće 3
- [ ] PLACE točka definirana
- [ ] APPROACH točke izračunate (100mm offset)
- [ ] 5 trajektorija generirano
- [ ] Quintic interpolacija primjena
- [ ] Ograničenja validirana
- [ ] Simulacija pokrenuta i testirana
- [ ] Robot communication modul spreman
- [ ] Dokumentacija potpuna
- [ ] Svi kod komentari
- [ ] Rezultati sprema u output/

---

**Sretno s Pick and Place projektom! 🤖🎯**
