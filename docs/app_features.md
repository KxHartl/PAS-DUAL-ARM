# Pick-and-Place CLI Aplikacija — Značajke i Praćenje

Ovaj dokument prati razvoj centralne CLI aplikacije (`src/main.py`) i svih modula.

---

## Arhitektura

```
src/main.py → CLI state machine
    ├── 01_percepcija/   (RealSense + PCL obrada)
    ├── 02_detekcija/    (YOLO + 3D lokalizacija)
    ├── 03_planiranje/   (Quintic interpolacija + task-space)
    ├── 04_izvrsenje/    (TCP/IP → UR3e)
    └── utils/           (Transformacije + Vizualizacije)
```

---

## Status Modula

### Infrastruktura
- [x] `config.py` — Konfiguracija (IP, portovi, putanje, parametri)
- [x] `utils/transforms.py` — 4x4 matrice, rotacije, učitavanje kalibracije
- [x] `utils/visualization.py` — Open3D i Matplotlib pop-up prozori

### Pipeline Moduli
- [x] `01_percepcija/realsense_capture.py` — Snimanje RGB-D i generiranje PCL
- [x] `01_percepcija/pcl_processing.py` — PassThrough, SOR, VoxelGrid, ICP, Clustering
- [x] `02_detekcija/object_detector.py` — YOLO detekcija, cluster matching, centroid, cam→robot
- [x] `03_planiranje/trajectory_planner.py` — Referentne točke, quintic, SLERP, URScript generiranje
- [x] `04_izvrsenje/robot_controller.py` — Socket komunikacija, gripper, safety

### Glavna Aplikacija
- [x] `main.py` — CLI s interaktivnim menijem i state machineom

---

## Status Koraka Aplikacije (Features)

### 1. INIT — Inicijalizacija
- [x] Učitavanje konfiguracije
- [x] Učitavanje kalibracijskih matrica iz `data/processed/calibration/`
- [x] Spajanje na UR3e robot (ili offline mod)
- [x] Spajanje na RealSense (ili offline mod)
- [x] Ispis statusa svih komponenti

### 2. SCAN — Skeniranje scene
- [x] Snimanje RGB + Depth iz RealSense kamere
- [x] Generiranje point clouda za svaku poziciju
- [x] Spremanje u `data/raw/scene_X/`
- [x] Open3D pop-up vizualizacija snimljenih cloudova
- [x] Potvrda od korisnika

### 3. PROCESS — Obrada point clouda
- [x] PassThrough filter (Z-ROI)
- [x] Statistical Outlier Removal
- [x] VoxelGrid downsampling
- [x] ICP registracija i fuzija (za više snimki)
- [x] Open3D vizualizacija: raw → filtrirano → fusionirano
- [x] Ispis statistika (broj točaka u svakoj fazi)
- [x] Spremanje u `data/processed/scene_fused.pcd`

### 4. DETECT — Detekcija objekata
- [x] RANSAC plane segmentation (uklanjanje stola)
- [x] Euclidean clustering
- [x] YOLO detekcija na RGB slici
- [x] Preslikavanje YOLO detekcija na 3D klastere
- [x] Izračun centroida za svaki klaster
- [x] Open3D prikaz obojanih klastera
- [x] Tablica pronađenih objekata

### 5. SELECT_OBJECT — Odabir predmeta
- [x] CLI meni s popisom detektiranih objekata
- [x] Open3D highlight odabranog objekta
- [x] Ispis centroida u kamera i robot sustavu

### 6. SELECT_PLACE — Odabir odredišta
- [x] CLI meni s predefiniranim lokacijama
- [x] Opcija za ručni unos koordinata
- [x] Transformacija place koordinate u robot sustav

### 7. PLAN — Planiranje trajektorije
- [x] Izračun 5 referentnih točaka (HOME, APPROACH, PICK, APPROACH_PLACE, PLACE)
- [x] Quintic interpolacija za svaki segment
- [x] Task-space interpolacija sa SLERP rotacijom
- [x] Provjera UR3e brzinskih/akceleracijskih limita
- [x] Matplotlib grafovi: x(t), ẋ(t), ẍ(t)
- [x] Open3D 3D prikaz putanje
- [x] Spremanje u `data/processed/trajectory.json`

### 8. CONFIRM — Potvrda izvršenja
- [x] Sažetak operacije (objekt, pick, place, trajanje)
- [x] Eksplicitna korisnikova potvrda (d/n)

### 9. EXECUTE — Izvršenje na robotu
- [x] Generiranje URScript programa
- [x] Slanje preko TCP/IP socketa na port 30003
- [x] Sekvenca: HOME→APPROACH→PICK→grip→APPROACH_PLACE→PLACE→release→HOME
- [x] Progress bar za svaki segment
- [x] EMERGENCY_STOP mogućnost
- [x] Spremanje log-a

### 10. Ponavljanje
- [x] Pitanje "Ponovi s novim objektom? [d/n]"
- [x] Povratak na SCAN korak

---

## Brisanje Starih Datoteka
- [x] Obrisati `src/camera_calibration/` cijeli folder
- [x] Obrisati `src/object_detection/` cijeli folder
- [x] Obrisati `src/pick_and_place/` cijeli folder
- [x] Konsolidirati requirements u jedan `requirements.txt`
- [x] Ažurirati `STATE.md`
