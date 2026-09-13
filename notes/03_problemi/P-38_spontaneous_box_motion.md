---
id: P-38
type: problem
status: rijeseno
requirements: ["[[R-10_mappable_world]]", "[[R-17_dual_arm_lift]]"]
solutions: ["[[S-02_world_and_sim_launch]]"]
decisions: ["[[D-05_contact_verified_attach]]"]
updated: 2026-09-13
---
# P-38: Kutija se sama pokreće i izlijeće iz prostorije

## Simptom
U GUI runu 41, odmah nakon starta `mapping_tour`, korisnik je vidio kako se
kutija sama miče i na kraju izlijeće iz prostorija. Tura nije naredila vožnju:
zaustavila se zbog odstupanja ruke. Read-only upit aktivnog Gazeba naknadno je
dao pozu `aruco_box` ≈ (−6.24, 3.30, 0.20) m, a SDF početna poza je
(0, −6.38, 0.25) m.

## Uzrok (potvrđeno čitanjem izvornog koda simulatora)
U izvornom kodu Ignition Fortress plugina (`DetachableJoint.hh:142`), autor je fiksno
postavio `private: std::atomic<bool> attachRequested{true};`. Zbog toga zglob pri
spawnu **uvijek započinje u stanju SPOJENO** (`starts attached`).
Kutija je na (0, −6.38) bila kruto vezana za lijevo zapešće robota (0, 0) preko 6.4 metra.
Čim je `mapping_tour` poslao naredbu lijevoj ruci da se savije u `ARM_CARRY_V2`, ruka
je djelovala kao poluga koja je kutiju iščupala sa stola i katapultirala je u luku
oko robota na udaljenost od ~7 m, pritom opteretivši `left_joint_6` za 0.614 rad.

## Pokušaji
| # | datum / commit | što | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 13. 9. / necommitano | korisnički GUI run 41; logovi terminala 1–4 i naknadna read-only Gazebo poza | kutija izvan prostorija; tura stala prije prve dionice | izolirati i pronaći izvor vanjske sile |
| 2 | 13. 9. / necommitano | Normally Open implementacija: automatski detach pri spawnu (`sim.launch.py`, `mapping.launch.py`, `mapping_tour.py`, `set_posture.py`) + ROS bridge za `/aruco_box/detach`, `attach` i `state` | kutija ostaje mirno na stolu, zglobovi ruku se miču bez opterećenja poluge | riješeno |

## Trenutno rješenje
1. `sim.launch.py` po izlasku `spawn_entity` i uz odgodu od 4 s automatski šalje `/aruco_box/detach`.
2. `mapping.launch.py`, `mapping_tour.py` i `set_posture.py` šalju detach prije bilo kakvog pokreta ruku.
3. `bridge.yaml` mosti `/aruco_box/attach`, `/aruco_box/detach` (ROS->GZ) i `/aruco_box/state` (GZ->ROS).

