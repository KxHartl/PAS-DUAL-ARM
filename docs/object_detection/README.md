# ZADACA 3 - Point Cloud Processing & Object Localization

## 📋 Pregled

Zadaca 3 zahtijeva kompletan pipeline za:
1. **Snimanje scene** - 3 point clouda iz različitih pozicija
2. **Obrada point cloud-a** - Filtriranje, registracija, fuzija
3. **Detekcija objekta** - Pronalaženje specifičnog voća u sceni
4. **Lokalizacija objekta** - 3D koordinate u kamerom i robot sustavu
5. **Transformacija koordinata** - Konverzija između koordinatnih sustava

---

## 📁 Struktura Foldera

```
for_zadaca_03/
├── scripts/
│   ├── capture_point_clouds.py       # Snimanje point cloudova
│   ├── pcl_processing.py              # Obrada (filtriranje, registration, itd.)
│   ├── object_detection_3d.py         # Detekcija i lokalizacija objekta
│   ├── run_full_pipeline.py           # Integracija cijelog pipeline-a
│   ├── predict_on_images.py           # Inference s YOLO (iz zadace 2)
│   ├── generate_3d_viz_reference.py   # Referenca za vizualizaciju
│   └── evaluate_model_reference.py    # Referenca za evaluaciju
├── point_clouds/
│   ├── voce_scene_pos1_*.pcd          # Raw point cloudovi
│   ├── voce_scene_pos2_*.pcd
│   └── voce_scene_pos3_*.pcd
├── output/
│   ├── final_merged_point_cloud.pcd   # Konačan obrađeni cloud
│   └── results.json                   # Rezultati detekcije
├── models/
│   ├── yolo26n.pt                     # Pretreniran YOLO model
│   ├── yolov8n-seg.pt                 # Segmentation model
│   ├── yolo26n_trained.pt             # Model obučen u zadaci 2
│   └── yolo26n-seg_trained.pt         # Segmentation obučen u zadaci 2
├── datasets/
│   └── Humanoidna_svo_voce_02.yolov8-obb/  # Dataset s 7 klasa voća
├── docs/
│   └── lectures/                      # Materijali s predavanja
├── requirements.txt                   # Python dependencije
├── README.md                          # Ovaj fajl
└── ANALIZA_ZADACA_2_I_3.md            # Detaljna analiza zadaca

```

---

## 🚀 Brzi Start

### 1. Instalacija Dependencija

```bash
# Premjesti se u folder
cd for_zadaca_03

# Instaliraj sve potrebne biblioteke
pip install -r requirements.txt

# Dodaj nove dependencije za Point Cloud obradu
pip install open3d pyntcloud transforms3d

# Opciono: Python RealSense SDK (ako koristiš Intel RealSense kameru)
pip install pyrealsense2
```

### 2. Testiranje Pipeline-a (sa sintetičkim podacima)

```bash
# Pokreni kompletan pipeline s testnim podacima
python scripts/run_full_pipeline.py

# Ili samo jednostavan primjer
python scripts/run_full_pipeline.py --simple
```

### 3. Snimanje Realnih Point Cloud-a

```python
from scripts.capture_point_clouds import PointCloudCapture

# Inicijalizacija
capture = PointCloudCapture(output_dir="point_clouds")

# Snimanje 3 scene s Intel RealSense kamere
files = capture.capture_multiple_positions(
    scene_name="voce_scene_01",
    num_positions=3,
    method="realsense"
)
```

---

## 📊 Moduli i Funkcionalnosti

### 1. **capture_point_clouds.py** - Snimanje

Modul za snimanje point cloudova s RGB-D kamere.

**Ključne klase:**
- `PointCloudCapture` - Glavna klasa za snimanje

**Ključne metode:**
- `capture_from_realsense()` - Snimanje s Intel RealSense D455/D435
- `capture_from_synthetic()` - Sintetički test clouds
- `capture_multiple_positions()` - Snimanje iz više pozicija
- `save_pointcloud()` - Spremanje u .pcd ili .ply

**Primjer:**
```python
from capture_point_clouds import PointCloudCapture

capture = PointCloudCapture(output_dir="point_clouds")
files = capture.capture_multiple_positions("scene_01", num_positions=3)
```

---

### 2. **pcl_processing.py** - Obrada Point Cloud-a

PCL pipeline sa svim obaveznim koracima.

**Ključne klase:**
- `PointCloudProcessor` - Svi koraci obrade

**Ključne metode:**

| Metoda | Svrha |
|--------|-------|
| `passthrough_filter()` | Uklanjanje točaka van ROI-a |
| `statistical_outlier_removal()` | Uklanjanje anomalnih točaka |
| `voxel_grid_downsampling()` | Smanjenje broja točaka |
| `icp_registration()` | Usklađivanje cloud-a |
| `register_and_merge_clouds()` | Spajanje više cloud-a |
| `euclidean_clustering()` | Segmentacija objekata |
| `ransac_plane_segmentation()` | Detekcija planarnih površina |

**Primjer:**
```python
from pcl_processing import PointCloudProcessor

processor = PointCloudProcessor()

# Obrada
pcd = processor.passthrough_filter(pcd, axis='z', min_val=0, max_val=2)
pcd = processor.statistical_outlier_removal(pcd)
pcd = processor.voxel_grid_downsampling(pcd, voxel_size=0.01)

# Spajanje više cloud-a
merged = processor.register_and_merge_clouds([pcd1, pcd2, pcd3])

# Clustering
labels, num_clusters = processor.euclidean_clustering(merged)
```

---

### 3. **object_detection_3d.py** - Detekcija i Lokalizacija

Modul za pronalaženje objekta u 3D sceni.

**Ključne klase:**
- `ObjectDetectionAndLocalization` - Detekcija i lokalizacija

**Ključne metode:**

| Metoda | Svrha |
|--------|-------|
| `segment_by_color()` | Segmentacija po HSV boji |
| `segment_by_size()` | Filtriranje po veličini |
| `segment_by_shape()` | Segmentacija po obliku |
| `find_centroid()` | Pronalaženje centra masa |
| `find_bounding_box()` | Bounding box objekta |
| `compute_principal_axes()` | PCA - glavne ose |
| `localize_in_camera_frame()` | 3D lokalizacija u kamerom sustavu |
| `transform_to_robot_frame()` | Transformacija u robot sustav |
| `detect_and_localize_object()` | Kompletan pipeline |

**Primjer:**
```python
from object_detection_3d import ObjectDetectionAndLocalization

detector = ObjectDetectionAndLocalization()

# Detekcija
results = detector.detect_and_localize_object(
    pcd,
    object_type='red_apple',
    camera_to_robot_transform=T_cam2robot
)

print(results['centroid_camera'])
print(results['centroid_robot'])
```

---

### 4. **run_full_pipeline.py** - Integracija

Kompletna integracija svih koraka.

**Koraci:**
1. Snimanje point cloudova (3 pozicije)
2. Obrada svakog clouda (filtriranje, downsampling)
3. Registracija i spajanje
4. Segmentacija objekata (clustering)
5. Detekcija i lokalizacija ciljnog objekta

**Primjer:**
```python
from run_full_pipeline import TaskaPipeline

pipeline = TaskaPipeline(output_dir="output")

# Korak 1: Snimanje
pipeline.step1_capture_point_clouds(method="synthetic")

# Korak 2: Obrada
pipeline.step2_process_point_clouds()

# Korak 3: Registracija
pipeline.step3_register_and_merge()

# Korak 4: Clustering
pipeline.step4_cluster_objects()

# Korak 5: Detekcija
results = pipeline.step5_detect_target_object(object_type="red_apple")
```

---

## 🎯 Specifični Zahtjevi za Zadacu 3

### Obavezni koraci obrade:
- ✅ PassThrough Filtering
- ✅ Statistical Outlier Removal
- ✅ VoxelGrid Downsampling
- ✅ SACSegmentation / RANSAC
- ✅ Euclidean Clustering
- ✅ ICP Registration
- ✅ Point Cloud Stitching/Fusion

### Očekivani output:
1. **3 raw point clouda** - `voce_scene_pos*.pcd`
2. **1 obrađeni cloud** - `final_merged_point_cloud.pcd`
3. **Segmentirani objekti** - 7 voćaka detektirano
4. **3D pozicija objekta u kamerom sustavu** - (x, y, z)
5. **3D pozicija objekta u robot sustavu** - (x, y, z)
6. **Dokumentacija** - Svi koraci objašnjeni

---

## 🔧 Konfiguracija Parametara

### Point Cloud Obrada

```python
# Passthrough filtering
passthrough_config = {
    'axis': 'z',
    'min_val': 0.0,
    'max_val': 2.5
}

# Statistical Outlier Removal
sor_config = {
    'nb_neighbors': 20,
    'std_ratio': 2.0
}

# VoxelGrid Downsampling
voxel_config = {
    'voxel_size': 0.01  # 1 cm
}

# ICP Registration
icp_config = {
    'threshold': 0.02,
    'max_iteration': 200
}

# Euclidean Clustering
clustering_config = {
    'eps': 0.05,
    'min_points': 10
}
```

### Transformacije Koordinata

```python
import numpy as np
from object_detection_3d import ObjectDetectionAndLocalization

detector = ObjectDetectionAndLocalization()

# Primjer: Kamera 10cm desno od baze, rotirana 45°
translation = np.array([0.1, 0.0, 0.0])
rotation = detector.create_rotation_matrix_z(np.pi/4)

T_camera_to_robot = detector.create_transformation_matrix(
    translation,
    rotation
)
```

---

## 📈 Očekivani Rezultati

### Primjer Outputa:

```
==============================================================
DETEKCIJA I LOKALIZACIJA: red_apple
==============================================================

1. SEGMENTACIJA
Segmentacija po boji: HSV (0, 150, 150)...
  Segmentovano: 2543 točaka

2. PRONALAŽENJE CENTRA
Centroid pronađen: (0.2134, 0.3245, 1.2567)
Bounding box:
  Min: (0.1200, 0.2100, 1.1200)
  Max: (0.3068, 0.4390, 1.3934)

3. TRANSFORMACIJA U ROBOT SUSTAV
  Centroid u robot sustavu:
    (0.3245, 0.2134, 1.1800)

==============================================================
REZULTATI
==============================================================
Objekat: red_apple
Centroid (camera): [0.2134 0.3245 1.2567]
Centroid (robot): [0.3245 0.2134 1.1800]
```

---

## ⚠️ Česti Problemi i Rješenja

| Problem | Rješenje |
|---------|----------|
| `ImportError: open3d` | Instaliraj: `pip install open3d` |
| `ImportError: cv2` | Instaliraj: `pip install opencv-python` |
| Point cloud je preskromno točaka | Smanjite `voxel_size` pri downsamplingu |
| Clustering pronalazi premore clustera | Povećajte `eps` ili `min_points` parametar |
| ICP registracija ne konvergira | Koristite preliminarnu poravnanja ili vršite downsampling |

---

## 📚 Dodatni Resursi

### Point Cloud biblioteke:
- **open3d** - [dokumentacija](http://www.open3d.org/)
- **pyntcloud** - [GitHub](https://github.com/daavoo/pyntcloud)
- **PCL (C++)** - [uradni website](https://pointclouds.org/)

### Transformacije i rotacije:
- **numpy** - Linearna algebra
- **transforms3d** - 3D transformacije (ako trebate)

### Reference iz zadace 2:
- YOLO modeli za detekciju voća
- Dataseti s 7 klasa voća
- Evaluacijske metrike

---

## ✅ Checklist za Submisiju

- [ ] **Point Cloud Capture** - 3 clouda snimljena
- [ ] **Point Cloud Processing** - Svi PCL koraci implementirani
- [ ] **Registration & Fusion** - Cloudovi registrirani i spojeni
- [ ] **Object Segmentation** - 7 voćaka detektirano
- [ ] **Object Localization** - 3D centroid pronađen
- [ ] **Coordinate Transform** - Transformacija u robot sustav
- [ ] **Documentation** - Svi koraci dokumentirani
- [ ] **Results Saved** - Output u `output/` folde
- [ ] **Code Comments** - Kod je jasno komentirano
- [ ] **Report** - Izvještaj s rezultatima

---

## 📝 Primjer Izvještaja

```
ZADACA 3 - IZVJEŠTAJ

Učenika: [Ime]
Datum: [Datum]

1. SNIMANJE SCENE
   - 3 point clouda snimljena iz različitih pozicija
   - Rezolucija: 640x480, FPS: 30
   - Trajanje: 5 sekundi po poziciji
   - Output: voce_scene_pos1/2/3.pcd

2. OBRADA POINT CLOUD-a
   - PassThrough filtering: z ∈ [0, 2.5] m
   - Statistical Outlier Removal: neighbors=20, std_ratio=2.0
   - VoxelGrid downsampling: voxel_size=0.01 m
   - Ukupna redukcija točaka: 85%

3. REGISTRACIJA I SPAJANJE
   - ICP registration: fitness=0.8234, RMSE=0.0045
   - Konačan cloud: 125,432 točaka

4. SEGMENTACIJA OBJEKATA
   - Euclidean clustering: eps=0.05, min_points=10
   - Pronađeno 7 clustera (voćke)

5. DETEKCIJA CILJNOG OBJEKTA
   - Objekt: Red Apple
   - Centroid (camera): (0.213, 0.325, 1.257) m
   - Centroid (robot): (0.325, 0.213, 1.180) m

6. ZAKLJUČAK
   [Dodatne napomene i rezultati]
```

---

## 🔗 Kontakt za Pitanja

Za pitanja i probleme, pogledajte:
- `ANALIZA_ZADACA_2_I_3.md` - Detaljne informacije
- Predavanja s materijala (u `docs/lectures/`)
- Dokumentaciju pojedinih biblioteka

---

**Sretno sa zadaćom 3! 🚀**
