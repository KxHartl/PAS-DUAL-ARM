---
id: R-03
type: zahtjev
status: djelomicno
source: "[MAIL-prilog]"
parent: "[[00_MAPA]]"
solutions: ["[[S-01_robot_description]]", "[[S-03_ros2_control_setup]]"]
problems: ["[[P-13_torso_prismatic_no_lift]]"]
decisions: ["[[D-09_lift_with_arms_not_torso]]"]
updated: 2026-09-15
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
- [x] klizač se stvarno diže **pod težinom ruke** u Gazebu (15. 9.: 0.06 → 0.20 → 0.35 m, greška 0.0000 mm, na **pozicijskom** sučelju; [[P-13_torso_prismatic_no_lift]] #11)

## Trenutno stanje
✅ Od 15. 9. Klizači se dižu pod težinom ruke na običnom `position` sučelju s MoveItom.
Uzrok ranijeg zastoja nije bio teret nego `initial_value` **0.05**, jednak donjem graničniku —
robot je startao zaglavljen, a Ignition na graničniku ne miče zglob ni u jednom smjeru
([[P-13_torso_prismatic_no_lift]] #10–11). Time [[D-09_lift_with_arms_not_torso]] gubi razlog
postojanja. Mase su i dalje procjena: vodilice 12 kg, klizači 2 kg.

## Kako se rješava
- [[S-01_robot_description]]: `src/dual_arm_torso/urdf/dual_arm_torso.urdf.xacro`
- [[S-03_ros2_control_setup]]: `torso_controller` (JTC, position)

## Problemi
- [[P-13_torso_prismatic_no_lift]]

## Odluke / odstupanja
- [[D-09_lift_with_arms_not_torso]]
