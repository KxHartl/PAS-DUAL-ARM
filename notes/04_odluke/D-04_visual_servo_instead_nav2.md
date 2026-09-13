---
id: D-04
type: odluka
status: privremena
deviation: true
requirements: ["[[R-14_slam_mapping]]", "[[R-15_region_goal_nav2]]", "[[R-16_find_box]]"]
problems: ["[[P-11_nav2_slam_drift]]", "[[P-19_aruco_foreshortening_close]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-04: Visual servo (cmd_vel na marker) umjesto Nav2/SLAM

## Kontekst
Na punom autonomnom slijedu (30. 6., `580948b`) je 360° spin za traženje markera razbio
lokalizaciju: skid-steer kliže → wheel-odom i scan-matching su krivi → robot „odluta“ ~30 m.

## Opcije
1. Popraviti lokalizaciju (IMU fuzija, bolja odometrija, pravi omni pogon).
2. **Zaobići Nav2/SLAM**: skeniranje pan-tilt kamerom uz mirnu bazu, pa izravni `cmd_vel` servo na
   TF `base_link → aruco_marker_frame`.

## Odluka (30. 6., `33bc2ac`)
Opcija 2. Servo je neovisan o SLAM-u i radi pouzdano za prilaz kutiji.

## Posljedice
- Nema karte, nema zadavanja regije, nema vožnje kroz vrata do odredišta.
- Kod za Nav2 (`nav2.launch.py`, `cmd_vel_relay`, `nav2_params.yaml`) ostaje u repou i radio je
  23. 6.

## Odnos prema zahtjevima
🔁 **Odstupa od obaveznih [[R-14_slam_mapping]] i [[R-15_region_goal_nav2]]** („Za mapiranje i
navigaciju koristiti nav2_stack i slam_toolbox“). U [[odstupanja]].
Preporuka je hibrid: Nav2 + SLAM za putovanje (regija, vrata, odredište), a visual servo samo za
finalni prilaz kutiji. Tako servo postaje dopuna Nav2, a ne zamjena.
