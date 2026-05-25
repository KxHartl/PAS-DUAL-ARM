# ANALIZA ZADACE 2 → ZADACA 3

## ZADACA 2 - CILJ I IMPLEMENTACIJA

### Što je bila zadaca 2?
**Cilj**: Priučiti YOLO model na detekciju i segmentaciju 3D printanog voća (7 tipova) u scene.

### Što je kreirano u zadaci 2:
1. **YOLO treninig pipeline**
   - `train_yolo_fruit_seg.py` - Treniranje YOLOv8 nano segmentation modela
   - Korišteni dataset: `Humanoidna_svo_voce_02.yolov8-obb` (object detection)
   - Više varijacija datasetsa: `humanoidna_voce_sve_v3.yolov8`, `humanoidna_voce_crvena_jabuka.yolov8`
   - **Rezultat**: Obučeni modeli (`yolo26n.pt`, `yolo26n-seg.pt`)

2. **Predikcia i evaluacija**
   - `predict_on_images.py` - Inference na slici
   - `eval_yolo_fruit_seg.py` - Evaluacija performansi modela
   - `check_classes.py` - Provjera klasa u datasetima
   - `split_dataset.py` - Podjela dataseta
   - `remap_labels.py` - Remapiranje labela

3. **Vizualizacija**
   - `generate_3d_viz.py` - Vizualizacija 2D slike kao "point cloud" efekt
   - Detektiranje boje voća (HSV filtriranje)
   - Pronalaženje centra objekta

4. **Struktura dataseta**
   ```
   datasets/
   ├── Humanoidna_svo_voce_02.yolov8-obb/  (7 klasa voća)
   ├── humanoidna_voce_sve_v3.yolov8/
   ├── humanoidna_voce_crvena_jabuka.yolov8/
   └── classes/  (mape klasa)
   ```

### Output zadace 2:
- Obučeni YOLO modeli (`.pt` fajlovi)
- Prediktivni rezultati
- Evaluacijske metrike (confusion matrix, accuracy, precision, recall)
- Slike i izvještaji

---

## ZADACA 3 - NOVI ZAHTJEVI

### Što je potrebno za zadacu 3?

#### 1. **SNIMANJE SCENE - Point Cloud Acquisition**
- Snimiti scenama najmanje **3 point clouda** iste scene
- Različite pozicije kamere/robota za dodatnu geometrijsku informaciju
- Scena sadrži: **7 tipova 3D printanog voća** (po jedan od svakog tipa)
- Output: `.pcd` ili `.ply` fajlovi (Point Cloud Data format)

#### 2. **OBRADA POINT CLOUD-a** (PCL Pipeline)
Obavezni koraci:
- ✅ **PassThrough Filtering** - Uklanjanje šuma van ROI-a
- ✅ **Statistical Outlier Removal** - Uklanjanje anomalnih točaka
- ✅ **Downsampling (VoxelGrid)** - Smanjenje broja točaka
- ✅ **Registration (ICP)** - Usklađivanje cloud-a s različitih pozicija
- ✅ **Point Cloud Stitching/Fusion** - Spajanje registriranih cloud-a
- ✅ **Segmentacija (EuclideanClusterExtraction)** - Odvajanje objekata
- ✅ **RANSAC** - Detekcija planarnih površina

**Rezultat**: Jedan konačni registrirani point cloud scene

#### 3. **DETEKCIJA I LOKALIZACIJA OBJEKTA**
Za zadani objekt (student ima definirani tip voća):
- Segmentirati objekta iz scene
- Pronaći njegov **centar masa** (3D centroid)
- Odrediti **3D koordinatu u koordinatnom sustavu kamere**
- **Transformirati u koordinatni sustav baze robota**

#### 4. **Transformacije koordinata**
- Kamera → World/Robot koordinatni sustav
- Kalibracija kamera
- Poznavanje fiksne pose kamere na robotu

---

## RELEVANTNI FAJLOVI IZ ZADACE 2 PREBAČENI U for_zadaca_03

### Modeli (u `models/`):
- `yolo26n.pt` - Pretreniran YOLO model
- `yolov8n-seg.pt` - Pretreniran segmentation model
- `yolo26n_trained.pt` - Obučeni model iz zadace 2
- `yolo26n-seg_trained.pt` - Obučeni segmentation model iz zadace 2

### Skripte (u `scripts/`):
- `predict_on_images.py` - Inference kod (može se adaptirati za point cloud frames)
- `generate_3d_viz_reference.py` - Referenca za 3D vizualizaciju
- `evaluate_model_reference.py` - Evaluacijski kod

### Dataseti (u `datasets/`):
- `Humanoidna_svo_voce_02.yolov8-obb/` - 7 klasa voća
- `humanoidna_voce_sve_v3.yolov8/` - Alternativni dataset
- `classes/` - Mape klasa

### Dependencije (u `requirements.txt`):
- ultralytics (YOLO)
- opencv-python (obrada slika)
- torch, torchvision (deep learning)
- matplotlib (vizualizacija)
- scikit-learn (ML alati)

---

## ŠTO JE POTREBNO IMPLEMENTIRATI ZA ZADACU 3

### 1. Point Cloud Acquisition Module
```
scripts/
├── capture_point_clouds.py
│   ├── Snimanje s RGB-D kamere (ili drugog izvora)
│   ├── Sprema u .pcd ili .ply format
│   └── Višestruke pozicije (min 3)
```

### 2. Point Cloud Processing Pipeline
```
scripts/
├── pcl_processing.py
│   ├── PassThrough filtering
│   ├── Statistical Outlier Removal
│   ├── VoxelGrid downsampling
│   ├── ICP registration
│   ├── Point cloud fusion
│   ├── Euclidean clustering
│   └── RANSAC plane segmentation
```

### 3. Object Detection & Localization
```
scripts/
├── object_detection_3d.py
│   ├── Segmentacija objekta iz scene
│   ├── Pronalaženje centroid-a
│   ├── 3D lokalizacija (x, y, z) u kamerom sustavu
│   └── Transformacija u robot koordinatni sustav
```

### 4. Camera Calibration & Coordinate Transform
```
scripts/
├── camera_calibration.py
│   ├── Intrinsic parametri (ako potrebno)
│   └── Extrinsic (poznata pose kamere na robotu)
├── coordinate_transform.py
│   ├── Kamera → Robot bazni sustav
│   └── Homogene transformacijske matrice
```

### 5. Visualization & Testing
```
scripts/
├── visualize_point_clouds.py - Pregled raw i procesiranih cloud-a
├── test_detection_pipeline.py - Testiranje detekcije
```

### 6. Main Pipeline
```
scripts/
├── run_full_pipeline.py
│   ├── Učitaj sve point cloud-ove
│   ├── Obradi ih (filtriranje, registration)
│   ├── Detektiraj zadani objekt
│   ├── Odredi 3D poziciju
│   └── Ispiši koordinate u robot sustavu
```

---

## NOVE DEPENDENCIJE ZA ZADACU 3

Trebati će dodati:
- **open3d** - Point Cloud obrada i vizualizacija
- **pcl-py** ili **python-pcl** - PCL binding
- **numpy, scipy** - Numerička matematika
- **transforms3d** - 3D transformacije i rotacije
- **pyntcloud** - Alternativa za PCL obrada

---

## KORACI ZA POČETAK ZADACE 3

1. ✅ **Kreiraj folder strukturu** → `for_zadaca_03/` ✅ Gotovo
2. **Prebaci relevantne datoteke iz zadace 2** → ✅ Gotovo
3. **Postavi point cloud acquisition** → Snimanje 3 scene
4. **Implementiraj PCL pipeline** → Obrada point cloud-a
5. **Implementiraj object detection** → Detekcija voća
6. **Calibration & coordinate transform** → Transformacije
7. **Testiraj cijeli pipeline**
8. **Dokumentiraj rezultate**

---

## KORISNE KOMANDE

```bash
# Instalacija novih dependencija
pip install open3d pyntcloud transforms3d

# Ili koristi python-pcl (ako dostupan za Windows)
pip install python-pcl

# Provjera instaliranih biblioteka
python -c "import open3d; print(open3d.__version__)"
```

---

## OČEKIVANI OUTPUT ZADACE 3

- 3 raw point clouda scene
- 1 procesiran i registrirani point cloud scene
- Segmentirani objekti (7 voćaka identificirano)
- Centroid zadanog objekta u kamerom sustavu
- Centroid zadanog objekta u robot baznom sustavu
- Vizualizacijske slike/video
- Dokumentacija cijelog procesa

