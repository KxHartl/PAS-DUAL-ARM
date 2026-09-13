---
id: P-04
type: problem
status: rijeseno
requirements: ["[[R-05_visual_match]]"]
solutions: ["[[S-02_world_and_sim_launch]]"]
decisions: []
updated: 2026-09-13
---
# P-04: Meshovi baze/torza/pan-tilta se ne renderiraju u Gazebu

## Simptom
Vidljive su samo Kinova ruke. U logu je 92× „Unable to find file“.

## Uzrok
**Potvrđeno:** Kinova meshovi su apsolutni `file://`, a ostali `package://` → `model://<pkg>/…`,
koje Fortress ne nalazi bez resource patha.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 12. 6. `d29498a` | `IGN_GAZEBO_RESOURCE_PATH` iz svakog `AMENT_PREFIX_PATH/share` u `sim.launch.py` | 92 → 0 grešaka, cijeli robot vidljiv | rješenje |
