---
id: D-09
type: odluka
status: vazeca
deviation: true
requirements: ["[[R-03_linear_rails_torso]]", "[[R-06_realistic_parameters]]"]
problems: ["[[P-13_torso_prismatic_no_lift]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-09: Dizanje rukama, a ne klizačima torza

## Kontekst
Prismatic klizači torza ne dižu se pod težinom ruke u `ign_ros2_control`: lagani zglobovi rade, a
opterećeni vertikalni ostaje na donjem limitu ([[P-13_torso_prismatic_no_lift]]).

## Odluka (30. 6., `397f080`)
Kutija se diže MoveIt pokretom ruku (~15 cm ravno gore). Torzo se postavi jednom (ili ostane na
limitu), a meta se podigla na stol (niža potrebna visina dohvata).

## Posljedice
Vodilice u misiji praktički ne sudjeluju. Ruke rade bliže rubu radnog prostora.

## Odnos prema zahtjevima
Djelomično odstupa od [[R-03_linear_rails_torso]] (aktuator postoji, ali ne radi pod teretom). U
[[odstupanja]].
