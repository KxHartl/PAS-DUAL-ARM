---
id: D-03
type: odluka
status: privremena
deviation: true
requirements: ["[[R-01_omni_base]]", "[[R-08_omni_controller]]"]
problems: ["[[P-03_pal_base_classic_control]]", "[[P-09_omni_drive_on_fortress]]", "[[P-10_skid_steer_cannot_turn]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-03: Baza kao skid-steer preko `diff_drive_controller` (PRIVREMENO)

## Kontekst
[MAIL] traži **obavezno omni_controller**. Na Fortressu se niti PAL-ov Classic ros2_control
([[P-03_pal_base_classic_control]]) niti Ignition `MecanumDrive` / `VelocityControl` nisu
instancirali ([[P-09_omni_drive_on_fortress]]). `ign_ros2_control` je u isto vrijeme dokazano radio.

## Opcije (23. 6.)
1. Tražiti/portati PAL `omni_drive_controller` (nije u Humble apt-u).
2. Planarni „kinematski“ pogon (PAL `planar_move`, samo Classic).
3. **`diff_drive_controller` na 4 kotača (skid-steer)**: x + yaw, odometrija, TF.

## Odluka (23. 6.)
Opcija 3, kao **privremeno** rješenje: „x + yaw je sve što zadatak treba“.

## Posljedice
- Za okret u mjestu nužan je mu2 = 0 ([[P-10_skid_steer_cannot_turn]]). To je nerealno trenje, a
  baza krabira ako se okret i vožnja rade istovremeno.
- Klizanje pri okretu razbija odometriju i SLAM ([[P-11_nav2_slam_drift]]) → povlači
  [[D-04_visual_servo_instead_nav2]].

## Odnos prema zahtjevima
🔁 **Odstupa od obaveznog [[R-08_omni_controller]].** U [[odstupanja]].
13. 9. je nađeno da je `mecanum_drive_controller` 2.53.3 instaliran, pa ova odluka treba biti
zamijenjena ako proba uspije.
