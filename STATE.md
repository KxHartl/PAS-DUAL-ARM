# Stanje projekta

## Trenutni zadatak
**Zadaća 3:** Interaktivna Pick-and-Place CLI aplikacija za branje voća s UR5e robotom.

## Status

✅ **Integracija voce_zadaca pipeline-a u src/ — ZAVRŠENO**
- Svi radni moduli iz `voce_zadaca/` kopirani u `src/` bez izmjena.
- Jedina izmjena: `BASE_DIR` u `src/config.py` → pokazuje na `voce_zadaca/` (za calibration, models, data, output).
- Kreirana nova interaktivna CLI aplikacija: `src/cli_app.py`.

## Struktura

```
src/
├── cli_app.py              # ★ NOVA — Interaktivna CLI aplikacija
├── config.py               # Konfiguracija (BASE_DIR → voce_zadaca/)
├── io_utils.py             # IO pomoćne funkcije
├── capture_scene.py        # Snimanje scene (robot + kamera)
├── segment.py              # YOLO segmentacija voća
├── reconstruct_scene.py    # 3D rekonstrukcija point clouda
├── estimate_pick_pose.py   # Estimacija pick poze
├── trajectory_planner.py   # Planiranje trajektorije (kvintička interpolacija)
└── ur_executor.py          # Generiranje i slanje URScript programa
```

## Pokretanje

```bash
# Aktiviraj virtualno okruženje
.venv\Scripts\activate

# Puni mod (robot + kamera)
python -X utf8 src/cli_app.py

# Offline mod (bez robota i kamere)
python -X utf8 src/cli_app.py --offline

# Koristi već snimljene podatke
python -X utf8 src/cli_app.py --existing-captures voce_zadaca/output/run_XXXXX/captures
```

## CLI tok aplikacije
1. **SNIMANJE** — robot se pomiče u capture poze, kamera se prikazuje uživo, pauza na svakoj poziciji
2. **DETEKCIJA** — YOLO pronalazi SVE klase voća, prikazuje overlay s maskama i boundingboxevima
3. **IZBORNIK** — korisnik bira koje voće pokupiti iz tablice detekcija
4. **REKONSTRUKCIJA + TRAJEKTORIJA** — 3D rekonstrukcija, pick poza, trajektorija + 3D plot
5. **IZVRŠENJE** — potvrda, slanje URScript programa, real-time praćenje robota
