---
id: ODSTUPANJA
type: registar
updated: 2026-09-17
---
# Odstupanja od zahtjeva (iskreno, za seminar)

> Svaka odluka s `deviation: true` mora biti ovdje. Kad se odstupanje ukloni, red se precrta
> i upiše datum i oznaka runa koji to dokazuje (**ne commit hash** — vidi napomenu na dnu).

## Preostala odstupanja (stanje 17. 9. 2026.)

Tri, i sva tri su u seminaru (Tab. 2.1 i §10).

| # | Zahtjev (izvor) | Traženo | Napravljeno | Zašto (tehnički) | Odluka |
|---|---|---|---|---|---|
| 1 | [[R-06_realistic_parameters]] [ZAD] | realna interakcija i parametri po specifikacijama | kutiju drži `DetachableJoint`, uključen tek nakon dokazanog obostranog dodira; mase vodilice i klizača su procjena (12 kg / 2 kg) | DART ne drži objekt trenjem jastučića (iscrpno probano, [[P-15_dart_friction_no_hold]]); proizvođač vodilica ne objavljuje mase | [[D-05_contact_verified_attach]], [[D-14_light_box_free_size]] |
| 2 | [[R-08_omni_controller]] [MAIL] | pogonski kontroler proizvođača baze | `mecanum_drive_controller` iz `ros2_controllers` | PAL-ov kontroler je za Gazebo Classic i na Fortressu ne objavljuje ništa ([[P-03_pal_base_classic_control]], [[P-09_omni_drive_on_fortress]]) | [[D-03_diff_drive_base_temporary]] (zamijenjena) |
| 3 | [[R-11_door_80cm]] [MAIL] | prolaz „recimo 80 cm" | prolazi **1.0 m** (na karti 0.980 m), i to **dvoja** | robot je s uvučenim rukama 0.821 m širok, pa kroz 0.80 m fizički ne prolazi. Umjesto toga je test otežan na drugoj osi: dvoja vrata, u oba smjera, prazan i s kutijom | [[D-13_three_room_world]] |

## Uklonjena odstupanja

Sve niže je nekad bilo odstupanje i više nije. Ostavljeno je jer je dio priče o razvoju.

| Zahtjev | Bilo | Riješeno | Dokaz |
|---|---|---|---|
| ~~[[R-08_omni_controller]]~~ | diff_drive, pa skid-steer koji ne može skrenuti | omnidirekcijski pogon (mecanum, `mu1=0.80`, `mu2=0.20`) | 13. 9., [[P-09_omni_drive_on_fortress]], [[P-10_skid_steer_cannot_turn]] |
| ~~[[R-14_slam_mapping]], [[R-15_region_goal_nav2]]~~ | „radilo u lipnju; danas je vizualni servo bez karte" jer je skid-steer klizao i lokalizacija je skakala ~30 m | SLAM karta prihvaćena geometrijskim gateom, Nav2 + AMCL vozi svaku dionicu misije; vršna greška lokalizacije 3.0 / 2.9 cm i 0.3° | runovi 60, 62, 70 ([[P-11_nav2_slam_drift]], [[D-20_single_potential_field_costmap]]) |
| ~~[[R-19_door_pass_with_box]]~~ | „vožnja baze s kutijom na spoju je izbacuje" | kutija je prošla **oboja** vrata, ruke u `CARRY_V4` | run M4, 16. 9. ([[P-18_transport_drops_box]]) |
| ~~[[R-20_place_at_destination]]~~ | „vraća se na isti stol" | odloženo na `place_table` u crvenoj sobi, na marker | run M4: `PLACE VERIFIED: 5 mm od centra markera` |
| ~~[[R-03_linear_rails_torso]]~~ | „klizači ne dižu pod teretom, uzrok nepoznat (probano 1000 N, gain)" | uzrok je bio `initial_value = 0.05` **točno na donjem graničniku**; nakon 0.05 → 0.06 vodilice dižu pun teret ruku | run F8, 15. 9. ([[P-13_torso_prismatic_no_lift]]) |
| ~~[[R-17_dual_arm_lift]]~~ | „dvije krute veze eksplodiraju solver" pa se vodilo kao neispunjen zahtjev | **dizanje jest dvoručno**: uvjet za dizanje je potvrđen dodir **obiju** ruku s kutijom, inače se misija prekida. Kruta veza se uspostavlja na jedno zapešće — to je detalj izvedbe, jer dvije istovremene krute veze čine kutiju preodređenom i ruše numeričko rješavanje fizike | run M4, 16. 9. ([[D-05_contact_verified_attach]], [[D-07_carry_on_left_wrist]], [[P-17_detachable_joint_explodes]]) |

## Kako to napisati u seminaru

Kratko i činjenično, po obrascu: *zahtjev → pokušano → izmjereni ishod → uzrok → što smo
napravili umjesto toga → što bi bio sljedeći korak*. Tablice pokušaja iz P-kartica su izravni
izvor. Načelo ([[D-12_honesty_abort_over_fake]]): **nijedan rezultat nije lažiran**, a svaki je
prekid zabilježen s razlogom. U tekstu rada koristiti formalni izraz *„prekid uz zapis razloga"*,
ne žargon.

## Nije odstupanje (napomena)

- Vlastiti ArUco detektor: `aruco_ros` je u mailu prijedlog, ne obveza ([[D-02_own_aruco_detector]]).
- Dimenzije i masa kutije nisu zadane (potvrdio korisnik 13. 9.). Odabrano je **0.30 m brida i
  0.30 kg mase** ([[D-14_light_box_free_size]]).
- Dvoja vrata i tri sobe su **stroži** test od traženog „jednog prolaza" ([[D-13_three_room_world]]).

> [!warning] O commit hashevima
> Ranije verzije ovog registra i P-kartica citirale su hasheve u koloni „datum / commit". Ti
> hashevi **ne postoje u ovom repozitoriju** i bili su nedokazivi. Od 17. 9. se referira datum i
> oznaka runa iz [[runovi]]. Hashevi se navode samo za **tuđe** pakete, gdje su pinovi u
> `ros2.repos` i provjerivi.
