---
id: D-14
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-12_box_with_aruco]]", "[[R-17_dual_arm_lift]]", "[[R-19_door_pass_with_box]]"]
problems: ["[[P-18_transport_drops_box]]", "[[P-17_detachable_joint_explodes]]", "[[P-14_gripper_too_small_for_cube]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-14: Kutija slobodnih dimenzija, lagana (0.3 kg, „plastika“), veličina 0.30 m zadržana

## Kontekst
Korisnik je 13. 9. potvrdio da **dimenzije ni masa kutije nigdje nisu zadane** ([[izvori]]): 0.3 m /
1 kg je bio usmeni dogovor koji ne obvezuje. Cilj je kutija koju je što lakše naći i podignuti,
s ArUco markerom.

## Opcije
1. Manja kutija, npr. 0.20 m. Traži izmjenu svih konstanti hvata i ponovno ugađanje dosega
   ([[P-25_asymmetric_arm_reach]]) na zadnji dan.
2. Hvatljiva šipka kao 30. 6. Traži drugi način hvata (top-down) i drugi kod.
3. **Zadržati kocku 0.30 m, na kojoj je squeeze ugođen i provjeren (3 GUI ciklusa), i smanjiti
   masu na 0.3 kg.**

## Odluka (13. 9.)
Opcija 3:
- `aruco_box`: kocka 0.30 m, **0.3 kg** (I = m·s²/6 = 0.0045), μ 5, žuta boja;
- jedan marker (DICT_4X4_50, ID 0, 0.165 m) na plohi okrenutoj vratima plave sobe
  ([[D-13_three_room_world]]).

## Zašto
- Sve konstante hvata vezane su uz polovicu kocke (0.15 m): marker → centar, `squeeze_poses`,
  `fingertips_on_box`, kolizijska scena, sanity u `measure_box` ([[06_parametri]]). Promjena veličine
  znači ponovno ugađanje.
- Manja masa smanjuje konzolno opterećenje lijevog zgloba pri vožnji (hipoteza B u
  [[P-18_transport_drops_box]]) i napor solvera ([[P-17_detachable_joint_explodes]]).

## Posljedice
Ako kocka od 0.30 m zapne u prolazu od 0.9 m, sljedeći korak je manja kutija (opcija 1), ali tek uz
parametrizaciju polovice kocke u `main_task.py`.

## Odnos prema zahtjevima
Ispunjava [[R-12_box_with_aruco]]. Nadopunjuje [[D-06_cube_squeeze_grasp]] (metoda hvata ostaje
ista).
