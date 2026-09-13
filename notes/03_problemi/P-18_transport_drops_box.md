---
id: P-18
type: problem
status: otvoreno
requirements: ["[[R-19_door_pass_with_box]]", "[[R-20_place_at_destination]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]", "[[S-09_task_orchestration]]", "[[S-06_navigation]]"]
decisions: ["[[D-07_carry_on_left_wrist]]", "[[D-05_contact_verified_attach]]"]
updated: 2026-09-13
---
# P-18: Vožnja baze s kutijom izbacuje kutiju

## Simptom
Okret baze (0.4 rad/s) s ispruženom rukom zaljulja kutiju i spoj je „pusti“, odnosno kutija
odleti. Zato se kutija dosad vraćala na isti stol.

## Uzrok
**Nepotvrđeno. Hipoteze, po vjerojatnosti:**
- **(A)** kontakt prsti/jastučić ↔ kutija se tuče s krutim spojem (isti mehanizam kao
  [[P-17_detachable_joint_explodes]]);
- **(B)** konzolni teret (1 kg; od 13. 9. 0.3 kg) na ispruženoj lijevoj ruci + ubrzanja baze → JTC tracking/solver;
- **(C)** DART solver s krutim spojem na pokretnom višetijelnom sustavu;
- **(D)** kutija dodiruje stol/tijelo tijekom vožnje.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. `ec766a2` | okret baze 0.4 rad/s s pločom na zglobu | kutija ispala | — |
| 2 | 30. 6. `5c9e201` | vožnja baze do zasebnog stola | kutija izbačena | odustano, place na isti stol |
| 3 | 16. 7. `73617e8` | **transport-proba** `probe_transport`: nakon lifta otvori oba jastučića (test A), vozi 0.4 m + okret ~60° | 🧪 **nije pokrenuto** | — |

## Trenutno rješenje
Nema: place na isti stol ([[R-20_place_at_destination]] ⚠).

## Sljedeći korak (prioritet 1 za misiju, time-box ~60 min)
1. `task.launch.py` **ne prosljeđuje** param. Dodati `DeclareLaunchArgument('probe_transport')` →
   `parameters=[{'use_sim_time': True, 'probe_transport': LaunchConfiguration(...)}]`, ili prije
   koraka 7 `ros2 param set /main_task_node probe_transport true`.
2. Pratiti pozu kocke izvana: `ign topic -e -t /world/seminar_world/dynamic_pose/info` (provjeriti
   točan topic s `ign topic -l`).
3. **Ishod A (kocka preživi):** integrirati carry pozu → staging ispred vrata → kroz vrata → pred
   `place_table` (crvena soba) ([[R-19_door_pass_with_box]]).
4. **Ishod B (odleti):** matrica, jedno po jedno:
   - (i) sporije rampe ili ω;
   - (ii) kocka bliže tijelu (carry poza iznad baze);
   - (iii) `parent_link` spoja na torzo/`base_link` („kocka leži na robotu“, zapisati kao
     odstupanje);
   - (iv) manja masa: **primijenjeno 13. 9.** (1.0 → 0.3 kg, [[D-14_light_box_free_size]]).
   Svaki pokušaj je novi red u ovoj tablici.

**Kriterij uspjeha:** nakon 0.4 m + 60° kocka unutar 5 cm od zgloba, bez rotacije > 10°.

## Ne ponavljati
- Brzi okret (≥ 0.4 rad/s) s kutijom na ispruženoj ruci i zatvorenim jastučićima.
