# ANALIZA VJEZBE TRAJEKTORIJA → PICK AND PLACE ZADATAK

## VJEZBA TRAJEKTORIJA - CILJ I IMPLEMENTACIJA

### Što je bila vjezba trajektorija?
**Cilj**: Naučiti geneiriranje i slanje trajektorija za UR robota u zglobnom i task prostoru.

### Što je kreirano u vjezbi:
1. **Polinom prvog reda** (`01_polinom_prvog_reda.py`)
   - Jednostavna linearna interpolacija između točaka
   - Bez razmatranja ograničenja robota

2. **Polinom 5. reda (Quintic)** (`02_polinom_petog.py`)
   - Glatke trajektorije s kontinuiranom brzinom i akceleracijom
   - Učitava početnu i konačnu konfiguraciju
   - Primjenjuje ograničenja:
     - v_max = 0.5 m/s (translacija)
     - a_max = 1.0 m/s² (translacija)
     - omega_max = 1.0 rad/s (rotacija)
     - q_dot_max = 360°/s po zglobu
     - q_ddot_max = 360-720°/s² po zglobu
   - Output: Diskretizirana trajektorija (1000 točaka)

3. **Task Space trajektorije** (`04_task_space.py`)
   - Trajektorije definirane u task space-u (x, y, z, orijentacija)
   - Koristi SLERP za interpolaciju orijentacije (smooth rotation)
   - Konverzija između task i joint space-a
   - Važno: Ove se trajektorije šalju kao `x_ref` (samo referentne točke)

4. **Slanje trajektorija robotu** (`03_send_UR_traj.py`)
   - TCP/IP komunikacija s UR robotom
   - Čitanje `.txt` datoteka s trajektorijama (URScript format)
   - Real-time slanje servoj naredbi robotu
   - Simulacija u PolyScope-u

### Output vjezbe:
- trajectory.txt - Trajektorija u zglobnom prostoru
- trajectory_fifth_order.txt - Quintic polinom verzija
- trajectory_task_space.txt - Trajektorija u task space-u
- Vizualizacijske slike (pozicija, brzina, akceleracija)

---

## PICK AND PLACE ZADATAK - NOVI ZAHTJEVI

### Što je potrebno za pick and place?

#### 1. **5 OBAVEZNIH TOČAKA TRAJEKTORIJE**
```
1. HOME - Početna pozicija robota (sigurna pozicija)
2. APPROACH - Iznad objekta (offset 100mm po Z od PICK točke)
3. PICK - Točka hvatanja objekta (iz point cloud obrade - ZADAĆA 3!)
4. APPROACH PLACE - Iznad mjesta odlaganja (offset 100mm po Z od PLACE)
5. PLACE - Točka odlaganja objekta (fiksna ili varijabilna)
```

#### 2. **DEFINICIJA TRAJEKTORIJE**
- Trajektorije definirane u **task space-u** (x, y, z, R, P, Y)
- Robot mora prolaziti kroz **svaku točku** (nema zaokruživanja kutova)
- Diskretizacija mora biti dovoljno fina (preporuka: 100+ točaka po segmentu)
- Robot radi samo **low-level regulaciju** i praćenje referencija
- **Glatke kretnje** koje poštuju ograničenja

#### 3. **OFFSET DEFINICIJA**
```
APPROACH = PICK + [0, 0, -0.1]  # 100mm iznad objekta
APPROACH_PLACE = PLACE + [0, 0, -0.1]  # 100mm iznad mjesta
```

#### 4. **OGRANIČENJA ROBOTA**
Trebaju biti respektirana:
- Maksimalne brzine zglobova
- Maksimalne akceleracije zglobova
- Fizička ograničenja workspace-a
- Singulariteti konfiguracija

#### 5. **IMPLEMENTACIJA**
```
Koraci:
1. Učitaj PICK koordinate iz task 3 (point cloud obrada)
2. Definiraj PLACE točku (fiksno ili iz parametara)
3. Izračunaj APPROACH i APPROACH_PLACE
4. Kreiraj 5 segmentnih trajektorija:
   - HOME → APPROACH (4 segmenta, bez blend-a)
   - APPROACH → PICK (pravac)
   - PICK (pause za захvat)
   - PICK → APPROACH_PLACE (pravac)
   - APPROACH_PLACE → PLACE (pravac)
   - PLACE (pause za odlaganje)
   - PLACE → HOME (4 segmenta)
5. Slijedi primjer iz vjezbe: koristi quintic interpolaciju
6. Slanje na robota (TCP/IP via PolyScope simulacija)
```

---

## RELEVANTNI FAJLOVI IZ VJEZBE PREBAČENI

### Skripte (u `scripts/`):

1. **05_quintic_interpolation_reference.py**
   - Ključni koncept za geneiriranje glatkih trajektorija
   - Polinom 5. reda s kontinuiranom brzinom i akceleracijom
   - Primjer: 5-dimenzionalna interpolacija
   - Ograničenja: v_max, a_max, omega_max

2. **06_task_space_reference.py**
   - Primjer task space trajektorija
   - SLERP za orijentaciju (smooth rotation)
   - Konverzija između prostora
   - Plot vizualizacije

3. **07_send_trajectory_reference.py**
   - TCP/IP komunikacija s UR robotom
   - Čitanje trajektorije iz datoteke
   - Slanje URScript programa
   - Real-time komunikacija

### Primjeri Trajektorija (u `trajectories/`):
- **example_fifth_order.txt** - Format trajektorije za slanje robotu

### Dokumentacija (u `docs/`):
- **trajectory_task_specification.md** - Specifikacija zadatka
- **datasheets/** - Specifikacije UR robota i senzora

---

## ŠTA JE POTREBNO IMPLEMENTIRATI ZA PICK AND PLACE

### 1. Main Pipeline Script
```
scripts/
├── 01_generate_pick_place_trajectory.py
│   ├── Učitaj PICK iz point cloud obrade (zadaća 3)
│   ├── Učitaj/definiraj PLACE točku
│   ├── Izračunaj sve 5 točaka
│   ├── Generiraj trajektorije između točaka
│   ├── Primijeni quintic interpolaciju
│   ├── Ispis u datoteku (format za robota)
│   └── Vizualizacija trajektorije
```

### 2. Trajectory Generator
```
scripts/
├── 02_trajectory_planner.py
│   ├── Quintic interpolation u task space-u
│   ├── Validacija ograničenja (brzina, akceleracija)
│   ├── Konverzija task → joint space (ako trebam)
│   ├── Blending između segmenata (ili bez, kao je zahtjev)
│   └── Diskretizacija trajektorije
```

### 3. Reference Points Manager
```
scripts/
├── 03_reference_points_manager.py
│   ├── Učitavanje HOME konfiguracije
│   ├── Integracija s rezultatima point cloud obrade
│   ├── Definicija PLACE točke
│   ├── Automatska kalkulacija APPROACH točaka
│   └── Validacija workspace-a
```

### 4. Robot Communication
```
scripts/
├── 04_send_to_robot.py
│   ├── TCP/IP socket konekcija
│   ├── URScript generiranje
│   ├── Real-time slanje servoj naredbi
│   └── Feedback i monitoring
```

### 5. Visualization
```
scripts/
├── 05_visualize_trajectory.py
│   ├── 3D plot putanje TCP-a
│   ├── Brzine i akceleracije
│   ├── Ograničenja i validacija
│   └── Animacija kretnje
```

---

## NOVE DEPENDENCIJE

Trebati će:
- **scipy** - Za SLERP i interpolacije (já postoji)
- **numpy** - Za numeričke proračune (já postoji)
- **matplotlib** - Za vizualizaciju (já postoji)
- **roboticstoolbox-python** - Za kinematiku robota (opcionalno)
- **socket** - Za TCP/IP (built-in Python)
- **json** - Za čitanje konfiguracija (built-in Python)

---

## OČEKIVANI OUTPUT

1. **Trajektnu datoteke** (.txt format):
   - HOME_to_APPROACH.txt
   - APPROACH_to_PICK.txt
   - PICK_to_APPROACH_PLACE.txt
   - APPROACH_PLACE_to_PLACE.txt
   - PLACE_to_HOME.txt

2. **Reference točke** (JSON format):
   ```json
   {
     "HOME": [q1, q2, q3, q4, q5, q6],
     "PICK": [x, y, z, rx, ry, rz],
     "PLACE": [x, y, z, rx, ry, rz],
     "APPROACH": [x, y, z, rx, ry, rz],
     "APPROACH_PLACE": [x, y, z, rx, ry, rz]
   }
   ```

3. **Vizualizacije**:
   - 3D putanja s 5 ključnih točaka
   - Brzina i akceleracija kroz vrijeme
   - Ograničenja validacija

4. **Simulacija na robotu** (PolyScope):
   - Autonomna izvršenja kretnje
   - Real-time feedback

---

## INTEGRACIJA S PRETHODNIM ZADACIMA

### Tijek podataka:
```
Zadaća 2 (YOLO Model)
    ↓
    Koristi za detekciju voća u slici
    
Zadaća 3 (Point Cloud + Object Localization)
    ↓
    Daj 3D koordinate PICK točke u robot sustavu
    
Pick & Place (Trajectory Generation)
    ↓
    Koristi PICK koordinate za geneiriranje trajektorije
    
Output: 5 segmentnih trajektorija za robota
```

---

## PRVI KORACI

### 1. Instaliraj dependencije
```bash
pip install numpy scipy matplotlib
# RoboticsToolbox (opcionalno za naprednu kinematiku)
pip install roboticstoolbox-python
```

### 2. Definiraj reference točke
- HOME - sigurna pozicija (npr. vertical, u zraku)
- PLACE - gdje trebam odlagati objekta

### 3. Učitaj PICK podatke
- Iz rezultata zadaće 3 (point cloud obrada)
- 3D koordinate u robot sustavu

### 4. Generiraj trajektorije
- Koristi quintic interpolaciju
- Prilagodi ograničenjima robota

### 5. Testiraj na simulatoru
- PolyScope simulacija
- Real-time slanje servoj naredbi

---

## PRIMJER KOD STRUKTURE

```python
from scripts.reference_points_manager import ReferencePointsManager
from scripts.trajectory_planner import TrajectoryPlanner
from scripts.send_to_robot import RobotCommunication

# 1. Definiraj reference točke
pm = ReferencePointsManager()
pm.set_home(q_home)
pm.set_place(x_place, y_place, z_place)

# 2. Učitaj PICK iz point cloud obrade
pick_coords = load_from_point_cloud_output()
pm.set_pick(pick_coords)

# 3. Izračunaj APPROACH točke
pm.compute_approach_points(offset=0.1)  # 100mm offset

# 4. Generiraj trajektorije
planner = TrajectoryPlanner()
traj_home_to_approach = planner.generate_trajectory(
    pm.get_home(),
    pm.get_approach(),
    trajectory_type='task_space',
    interpolation='quintic'
)

# 5. Slanje na robota
robot = RobotCommunication(host='192.168.1.100', port=30003)
robot.send_trajectory(traj_home_to_approach)
```

---

## VAŽNE NAPOMENE

⚠️ **Nema blendinga kutova** - Robot mora točno prolaziti kroz sve 5 točaka
⚠️ **Diskretizacija** - Mora biti fina (min 50+ točaka po segmentu)
⚠️ **Ograničenja** - Trebaju biti poštovana i validirana
⚠️ **Sigurnost** - APPROACH točka mora biti sigurne udaljenosti prije objekta
⚠️ **Koordinatni sustavi** - Paziti na transformacije između sustava

---

## KORISNI RESURSI

- scipy.spatial.transform.Rotation - Manipulacija rotacijama
- scipy.interpolate - Interpolacijske metode
- roboticstoolbox - Kinematika i dinamika robota
- UR dokumentacija - Specifikacije i ograničenja

