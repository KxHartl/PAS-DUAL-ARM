---
id: R-03
type: zahtjev
status: djelomicno
source: "[MAIL-prilog]"
parent: "[[00_MAPA]]"
solutions: ["[[S-01_robot_description]]", "[[S-03_ros2_control_setup]]"]
problems: ["[[P-13_torso_prismatic_no_lift]]"]
decisions: ["[[D-09_lift_with_arms_not_torso]]"]
updated: 2026-09-13
---
# R-03: Linearne vodilice (torzo) iz priloga

## Izvor (doslovno)
> „linearne vodilice: u prilogu.“ [MAIL]. Prilog `dual_arm_torso-main.zip`: STL-ovi
> `assembly_main_simplified.STL`, `mts_carriage.STL` + CAD slika s mjerama.

## Tehnički znači
Torzo s dvije vertikalne vodilice. Svaka nosi klizač (prismatic zglob) na kojem je ruka, a klizač
se u simulaciji stvarno pomiče gore-dolje preko ros2_control.

**Kriterij prihvaćanja:**
- [x] STL-ovi u ispravnoj skali (0.001) i orijentaciji, klizači na širokim plohama (`LINKS.md`)
- [x] `torso_left/right_carriage_joint` (prismatic, **0.05–0.65 m**, hod stvarne vodilice) s `torso_controller`
- [ ] klizač se stvarno diže **pod težinom ruke** u Gazebu

## Trenutno stanje
⚠ Model i kontroler postoje. Klizači se pod težinom ruke (~100 N) **ne dižu** s donjeg limita
([[P-13_torso_prismatic_no_lift]]). Dizanje kocke zato rade ruke
([[D-09_lift_with_arms_not_torso]]). Mase su procijenjene: vodilice 12 kg, klizači 2 kg.

## Kako se rješava
- [[S-01_robot_description]]: `src/dual_arm_torso/urdf/dual_arm_torso.urdf.xacro`
- [[S-03_ros2_control_setup]]: `torso_controller` (JTC, position)

## Problemi
- [[P-13_torso_prismatic_no_lift]]

## Odluke / odstupanja
- [[D-09_lift_with_arms_not_torso]]
