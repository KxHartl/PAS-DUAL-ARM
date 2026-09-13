---
id: R-20
type: zahtjev
status: djelomicno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-08_grasp_squeeze_attach]]", "[[S-09_task_orchestration]]"]
problems: ["[[P-18_transport_drops_box]]", "[[P-29_place_drop_tips_cube]]", "[[P-30_stale_collision_object]]"]
decisions: []
updated: 2026-09-13
---
# R-20: Odlaganje kutije na zadano mjesto

## Izvor (doslovno)
> „… tamo pronađe kutiju podigne ju i potom ju odnese na decidirano mjesto.“ [MAIL]

## Tehnički znači
Kocka završi mirno i uspravno na [[R-13_destination_place]] (`target_table` iza vrata), a ruke se
odmaknu.

**Kriterij prihvaćanja:**
- [ ] kocka na `target_table` (poza provjerena izvana, npr. `ign topic … dynamic_pose/info`)
- [ ] kocka nije prevrnuta
- [ ] detach + odmak ruku bez udaranja kocke

## Trenutno stanje
⚠ Odlaganje radi samo **natrag na isti stol** s kojeg je uzeta (30. 6. sa pločom `5c9e201`).
S kockom se kocka zna ispustiti par cm previsoko i prevrne se ([[P-29_place_drop_tips_cube]]).
Popravak „spuštanje u kontakt −2 cm“ je 🧪 **neprovjeren** (`73617e8`).

## Kako se rješava
- [[S-08_grasp_squeeze_attach]]: STEP8 (spuštanje, detach, odmak)
- [[S-09_task_orchestration]]: dovoz do odredišta (ovisi o [[R-19_door_pass_with_box]])
