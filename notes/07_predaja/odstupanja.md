---
id: ODSTUPANJA
type: registar
updated: 2026-09-13
---
# Odstupanja od zahtjeva (iskreno, za seminar)

> Svaka odluka s `deviation: true` mora biti ovdje. Ako se odstupanje danas ukloni, red se
> precrta i upiše commit.

| # | Zahtjev (izvor) | Traženo | Napravljeno | Zašto (tehnički) | Odluka | Danas popravljivo? |
|---|---|---|---|---|---|---|
| 1 | [[R-08_omni_controller]] [MAIL] | **obavezno** omni_controller | `diff_drive_controller`, skid-steer (x + yaw) | PAL omni kontroler je Classic-only i nije u Humble apt-u; Fortress Mecanum/VelocityControl pluginovi se ne instanciraju | [[D-03_diff_drive_base_temporary]] | možda: `mecanum_drive_controller` ([[P-09_omni_drive_on_fortress]]) |
| 2 | [[R-14_slam_mapping]], [[R-15_region_goal_nav2]] [MAIL] | mapiranje slam_toolboxom, Nav2 do regije koju zada korisnik | radilo 23. 6.; danas je visual servo bez karte | skid-steer kliže pri okretu → drift ~30 m | [[D-04_visual_servo_instead_nav2]] | možda: hibrid ([[P-11_nav2_slam_drift]]) |
| 3 | [[R-11_door_80cm]] [MAIL] | vrata ~80 cm | 2.0 m | Nav2 costmap nije prolazio 0.8 m (inflation 0.35) | [[D-08_door_widened]] | **da** (SDF + test) |
| 4 | [[R-19_door_pass_with_box]] [MAIL] | prolaz s kutijom | ne | vožnja baze s kutijom na spoju je izbacuje | [[P-18_transport_drops_box]] | možda |
| 5 | [[R-20_place_at_destination]] [MAIL] | odnijeti na zadano mjesto | vraća se na isti stol | ovisi o #4 | — | ovisi o #4 |
| 6 | [[R-06_realistic_parameters]] [ZAD] | realna interakcija | držanje je DetachableJoint (uz dokazan kontakt) | DART ne drži objekt trenjem (iscrpno probano) | [[D-05_contact_verified_attach]] | ne (ograničenje simulatora) |
| 7 | [[R-06_realistic_parameters]] [ZAD] | realno trenje kotača | mu2 = 0 | skid-steer se inače ne okreće | [[P-10_skid_steer_cannot_turn]] | nestaje s #1 (ako mecanum + anizotropno trenje) |
| 8 | [[R-03_linear_rails_torso]] [MAIL] | vodilice kao aktuator | klizači ne dižu pod teretom; dižu ruke | uzrok nepoznat (probano 1000 N, gain) | [[D-09_lift_with_arms_not_torso]] | ne (niski prioritet) |
| 9 | [[R-06_realistic_parameters]] [ZAD] | parametri po specifikacijama | masa torza procijenjena (12 / 2 kg) | nema podataka od proizvođača vodilica | — | ne |
| 10 | [[R-17_dual_arm_lift]] [MAIL] | podići objema rukama | hvat obje ruke; nošenje lijeva + spoj | dvije krute veze eksplodiraju solver | [[D-07_carry_on_left_wrist]] | ne |

## Kako to napisati u seminaru
Kratko i činjenično, po obrascu: *zahtjev → pokušano → izmjereni ishod → uzrok → što smo napravili
umjesto toga → što bi bio sljedeći korak*. Tablice pokušaja iz P-kartica su izravni izvor.
Naglasiti politiku poštenja ([[D-12_honesty_abort_over_fake]]): **nijedan rezultat nije lažiran**,
a aborti su dokumentirani.

## Nije odstupanje (napomena)
- Vlastiti ArUco detektor: `aruco_ros` je u mailu prijedlog ([[D-02_own_aruco_detector]]).
- Kocka 0.3 m / 1 kg je [USM]. Mogli smo koristiti i manju kutiju ([[D-06_cube_squeeze_grasp]]).
