---
id: D-06
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-12_box_with_aruco]]", "[[R-17_dual_arm_lift]]"]
problems: ["[[P-14_gripper_too_small_for_cube]]", "[[P-26_one_sided_press_bulldozes]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-06: Kocka 0.3 m + dvoručni squeeze (umjesto hvatljive šipke)

## Kontekst
Robotiq 2F-85 ima hod ~85 mm i ne može obuhvatiti plohu od 0.3 m. Od 30. 6. do 15. 7. meta je
zato bila šipka ili ploča koju svaka ruka obuhvati na svom kraju ([[P-14_gripper_too_small_for_cube]]).

## Opcije
1. Hvatljiva šipka/ploča (radila, ali „nije kutija po zadatku“).
2. **Kocka 0.3 m, stisnuta između dviju ruku zatvorenim hvataljkama kao jastučićima.**

## Odluka (15./16. 7.)
Opcija 2: kocka „po zadatku“.

## Posljedice
Znatno teži hvat: asimetrični dosezi ([[P-25_asymmetric_arm_reach]]), simultani press
([[P-26_one_sided_press_bulldozes]]), kartezijski lanac ([[P-24_press_path_chain]]), uspješnost
~35 %.

## Odnos prema zahtjevima
Ne odstupa. Napomena ([[izvori]]): dimenzije 0.3 m / 1 kg su **[USM]** i ne obvezuju. [MAIL]
traži samo „kutiju“ s markerom, podignutu objema rukama. **Rezervna opcija za danas:** ako kocka
bude kočila transport kroz vrata od 0.8 m, manja kutija je legitimna. Ta zamjena mora ići kao nova
D-kartica, uz korisnikovu odluku.

**13. 9.:** korisnik je potvrdio da su dimenzije i masa slobodne. Masa je smanjena na 0.3 kg, a
veličina 0.30 m zadržana → [[D-14_light_box_free_size]].
