---
id: R-13
type: zahtjev
status: ispunjeno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-02_world_and_sim_launch]]"]
problems: []
decisions: ["[[D-13_three_room_world]]"]
updated: 2026-09-13
---
# R-13: Zadano (decidirano) odredišno mjesto

## Izvor (doslovno)
> „… i potom ju odnese na decidirano mjesto.“ [MAIL]

## Tehnički znači
U svijetu postoji unaprijed određeno mjesto odlaganja (stol) u **drugoj sobi**, do kojeg se dolazi
kroz vrata, na poznatoj poziciji.

**Kriterij prihvaćanja:**
- [x] `place_table` u **crvenoj sobi** na (4.3, 0), ploča 0.6 × 0.6 m, ploha z = 0.10
  ([[D-13_three_room_world]])
- [x] iste visine kao `pick_table`, pa se odlaže na visini uzimanja (bez novog dosega)

## Trenutno stanje
✅ Od 13. 9. (novi svijet). Stari `target_table` (0.775 m, doseg neprovjeren) je uklonjen. Samo
odlaganje je [[R-20_place_at_destination]].
