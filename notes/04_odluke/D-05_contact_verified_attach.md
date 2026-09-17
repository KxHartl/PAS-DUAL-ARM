---
id: D-05
type: odluka
status: vazeca
deviation: true
requirements: ["[[R-06_realistic_parameters]]", "[[R-17_dual_arm_lift]]"]
problems: ["[[P-15_dart_friction_no_hold]]", "[[P-16_fake_teleport_grasp]]", "[[P-17_detachable_joint_explodes]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-05: Kontaktom verificiran DetachableJoint (nije teleport)

## Kontekst
DART u Fortressu **ne drži slobodan objekt trenjem hvataljke** tijekom gibanja. Probano je
μ 2 → 5, masa 0.4 → 0.2 kg, kp i efort 120 ([[P-15_dart_friction_no_hold]]). Raniji „hvat“ je
bio teleport (`set_pose`), a kasnije zavarivanje iz daljine ([[P-16_fake_teleport_grasp]]).

## Opcije
1. Čisto trenje (fizički idealno, ali ne radi u DART-u).
2. Teleport / attach bez provjere (lažno).
3. **Kruti spoj (DetachableJoint `left_bracelet_link` ↔ `aruco_box`) koji se uključuje TEK nakon
   neovisno dokazanog kontakta.**

## Odluka (30. 6.,, s korisnikovim odobrenjem)
Opcija 3. Gate je danas: box-only kontakt na oba jastučića **I** geometrijski plauzibilan položaj
([[S-08_grasp_squeeze_attach]]).

## Posljedice
- Nošenje je fizički kruti spoj na lijevom zglobu. Dvije krute veze eksplodiraju solver
  ([[P-17_detachable_joint_explodes]], [[D-07_carry_on_left_wrist]]).
- Kad se robot nađe u „nemogućem“ stanju, spoj ga ne spašava. Zato politika
  [[D-12_honesty_abort_over_fake]].

## Odnos prema zahtjevima
Djelomično odstupa od „realistične interakcije“ ([[R-06_realistic_parameters]]): **hvat** je
fizički (kontakt), a **držanje** je simulacijska pomoć. U [[odstupanja]], uz obrazloženje
ograničenja DART-a.
