---
id: D-10
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-07_ros2_humble_fortress_control]]", "[[R-21_deliverables]]"]
problems: ["[[P-32_gui_starves_controllers]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-10: Headless za automatske provjere, GUI za demo i validaciju

## Kontekst
GUI renderer na opterećenom stroju izgladnjuje `gz_ros2_control` petlju, pa aktivacija kontrolera
istekne (23. 6.).

## Odluka
- `sim.launch.py headless:=true` (samo server) za brze i automatske provjere.
- **GUI** za sve što korisnik mora vidjeti. Korisnik vizualno validira svaki demo run (vidi
  memoriju „Radni stil“ i [[AGENT_GUIDE]]).
- Spawneri imaju `--controller-manager-timeout 120`.

## Posljedice
Neke „provjere“ iz lipnja su samo headless. Status u R-karticama to navodi.
