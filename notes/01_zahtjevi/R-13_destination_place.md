---
id: R-13
type: zahtjev
status: ispunjeno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-02_world_and_sim_launch]]"]
problems: []
decisions: []
updated: 2026-09-13
---
# R-13: Zadano (decidirano) odredišno mjesto

## Izvor (doslovno)
> „… i potom ju odnese na decidirano mjesto.“ [MAIL]

## Tehnički znači
U svijetu postoji unaprijed određeno mjesto odlaganja (stol) **iza vrata**, na poznatoj poziciji.

**Kriterij prihvaćanja:**
- [x] `target_table` na (4.0, 0), ploča 1.0 × 1.5 m, gornja ploha z ≈ 0.775 m, iza zida (x = 2.0)
- [ ] provjereno da ruke dosežu plohu na 0.775 m (kocka se na `pick_table` hvata na ~0.25 m)

## Trenutno stanje
✅ Stol postoji. ⚠ Doseg na 0.775 m s kockom na lijevom zglobu **nije provjeren**. Alternativa
je spustiti `target_table` na visinu `pick_table` (dizajnerska sloboda, jer [MAIL] ne propisuje
visinu). Samo odlaganje je [[R-20_place_at_destination]].

Napomena: postoji i `place_table` (1.55, 0.25) ispred zida. To **nije** odredište iza vrata.
