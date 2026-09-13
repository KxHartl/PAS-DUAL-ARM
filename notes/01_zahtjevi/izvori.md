---
id: IZVORI
type: izvori
updated: 2026-09-13
---
# Izvori zahtjeva: što obvezuje, a što ne

Svaka R-kartica nosi oznaku izvora. **Obvezuje samo [ZAD] i [MAIL].** Usmeni dogovori [USM]
nisu obvezujući (potvrdio korisnik 13. 9. 2026.). [VLAST] su naše dizajnerske odluke.

| Oznaka | Dokument | Datum | Obvezuje | Gdje je |
|---|---|---|---|---|
| [ZAD] | „Izrada simulacijskog modela DUAL ARM robota u Gazebo okruženju“, zadao B. Ćaran | 30. 11. 2025. | DA | `data/raw/zadatak/task_caran_2025-11-30.pdf` |
| [MAIL] | Mail asistenta B. Ćarana „Re: PAS - Dual arm robot - Podaci“ | 4. 5. 2026. | DA | lokalno `~/Downloads/email_caran_to_hartl_…pdf` (**ne commita se**: sadrži osobne podatke); citati su niže |
| [MAIL-prilog] | `dual_arm_torso-main.zip`: STL vodilica + CAD slika s mjerama | 4. 5. 2026. | DA | `src/dual_arm_torso/meshes/`, `data/raw/zadatak/torzo_cad_mjere.png` |
| [MAIL-slika] | Slika cijelog sustava „kako bi trebao izgledati“ | 4. 5. 2026. | DA (izgled) | `data/raw/zadatak/mail_img-000.png` |
| [USM] | usmeni dogovori (npr. kocka 0.3 m / 1 kg?) | — | NE | nigdje zapisano; vidi napomenu niže |
| [VLAST] | naše odluke | — | — | `04_odluke/` |

## [ZAD]: doslovno

> U sklopu ovog zadatka potrebno je izraditi simulacijski model DUAL ARM robota u Gazebo
> okruženju. Model treba sadržavati sve ključne komponente robota, uključujući senzore,
> aktuatorske mehanizme i upravljačke sustave. Cilj je omogućiti realističnu simulaciju kretanja i
> interakcije robota s okolinom. Parametre simulacije potrebno je podesiti tako da odgovaraju
> tehničkim specifikacijama stvarnog robota i aktuatora.
>
> Potrebni alati: 1. ROS2 Humble 2. Gazebo simulator: GZ Fortress (LTS) 3. ros2_control

## [MAIL]: doslovno (relevantni dijelovi)

> šaljem sve potrebno za izradu simulacijskog modela.
> - mobilna baza: https://github.com/pal-robotics/omni_base_simulation
> - ruke robota: https://github.com/Kinovarobotics/ros2_kortex
> - linearne vodilice: u prilogu.
> - pan-tilt kamera na vrhu robota: https://github.com/I-Quotient-Robotics/pan_tilt_ros
> - kamera: https://github.com/realsenseai/realsense-ros
>
> Šaljem sliku cijelog sustava kako bi trebao izgledati: *(slika: `data/raw/zadatak/mail_img-000.png`)*
>
> Potrebno je napraviti simulacijsko okruženje u Gazebo-u koje će robot mapirati, potom mu vi
> kažete otprilike regiju u koju da ode, tamo pronađe kutiju podigne ju i potom ju odnese na
> decidirano mjesto.
> Gazebo okruženje mora imat jedan prolaz, vrata standardne dimenzije recimo 80cm kroz koje
> robot mora proći s i bez kutije.
> Na kutiju slobodno stavite aruco markere (https://github.com/pal-robotics/aruco_ros) kako bi ju
> lakše detektirali.
>
> Nakon što ju detektirate morate ju podići s obje ruke.
>
> Sve mora biti napravljeno s ros2_control framework-om, obavezno koristiti omni_controller, ruke
> također moraju biti na ros2_control i upravljanje mora biti s MoveIt-om. Za mapiranje i
> navigaciju koristiti nav2_stack i slam_toolbox.

## Razlaganje na zahtjeve

| Rečenica izvora | → R-kartica |
|---|---|
| „sve ključne komponente … senzore, aktuatorske mehanizme, upravljačke sustave“ | [[R-01_omni_base]], [[R-02_kinova_arms]], [[R-03_linear_rails_torso]], [[R-04_pan_tilt_camera]] |
| „sliku cijelog sustava kako bi trebao izgledati“ | [[R-05_visual_match]] |
| „parametre … da odgovaraju tehničkim specifikacijama“ | [[R-06_realistic_parameters]] |
| „ROS2 Humble, GZ Fortress, ros2_control“ | [[R-07_ros2_humble_fortress_control]] |
| „obavezno koristiti omni_controller“ | [[R-08_omni_controller]] |
| „ruke … na ros2_control i upravljanje … s MoveIt-om“ | [[R-09_moveit_arm_control]] |
| „okruženje … koje će robot mapirati“ | [[R-10_mappable_world]], [[R-14_slam_mapping]] |
| „jedan prolaz, vrata … recimo 80cm“ | [[R-11_door_80cm]] |
| „na kutiju … aruco markere“ | [[R-12_box_with_aruco]] |
| „odnese na decidirano mjesto“ | [[R-13_destination_place]], [[R-20_place_at_destination]] |
| „kažete otprilike regiju … nav2_stack i slam_toolbox“ | [[R-15_region_goal_nav2]] |
| „tamo pronađe kutiju“ | [[R-16_find_box]] |
| „podići s obje ruke“ | [[R-17_dual_arm_lift]] |
| „proći s i bez kutije“ | [[R-18_door_pass_empty]], [[R-19_door_pass_with_box]] |
| predaja (seminar, repo, video, slajdovi) | [[R-21_deliverables]] |

## Napomene i nesigurnosti

- ✅ **Kocka 0.3 × 0.3 × 0.3 m, 1 kg NIJE ni u [ZAD] ni u [MAIL]** (potvrdio korisnik 13. 9.).
  Dimenzije i masa kutije su **slobodne**, vidi odluke korisnika niže i [[D-14_light_box_free_size]].
- „aruco_ros“ je u mailu dan kao prijedlog („slobodno“), a ne kao obveza. Vlastiti detektor je
  dopušten, vidi [[D-02_own_aruco_detector]].
- „omni_controller“ je izričito **obavezan**. PAL-ov `omni_drive_controller` nije dostupan za
  Humble apt, vidi [[P-09_omni_drive_on_fortress]].
- Formalni ([ZAD]) i proširen ([MAIL]) zadatak ne navode rok ni oblik predaje. Oblik predaje je od
  korisnika (13. 9.): seminar PDF, repo + simulacija, video, prezentacija, vidi [[R-21_deliverables]].

## Odluke korisnika (13. 9. 2026.)
Unutar slobode koju ostavljaju [ZAD] i [MAIL]:
1. **Kutija:** dimenzije i masa proizvoljne. Lagana („plastika“), odabrana da je što lakše naći i
   podignuti, s ArUco markerom → [[D-14_light_box_free_size]].
2. **Vrata:** 90 cm → [[D-13_three_room_world]].
3. **Okruženje:** tri sobe u obliku slova L. Srednja (siva) je home pozicija robota, desna (plava)
   ima stol s kutijom, a gornja (crvena) stol na koji treba donijeti kutiju. Otvori između soba su
   90 cm → [[D-13_three_room_world]].
4. **Redoslijed misije** (tumačenje korisnika, u skladu s [MAIL]): prvo mapiranje (SLAM), zatim
   odlazak po kutiju i nošenje → [[danas]].
