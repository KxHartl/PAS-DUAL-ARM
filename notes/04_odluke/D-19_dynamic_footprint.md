---
id: D-19
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
problems: ["[[P-37_arm_position_gain_sag]]", "[[P-39_nav2_enters_doorway_at_an_angle]]", "[[P-35_arm_span_too_wide_for_door]]"]
decisions: ["[[D-18_verified_baseline_first]]"]
superseded_by: ""
updated: 2026-09-14
---
# D-19 — Robot Nav2-u govori svoj stvarni oblik

## Kontekst
Korisnik je 14. 9. opisao problem ovako: *„kontroler pokušava direktno prolaziti i onda se
zabija sa stolom, vratima, zidovima, kao da ne znamo prostor koji robot zauzima."*

To je doslovno točno. `footprint` je bio **statični poligon u YAML-u** (±0.52 × ±0.427 m),
izmjeren jednom, za jednu pozu ruku. Ruke se miču, a `ign_ros2_control` poziciju drži mekano
pa se objese — izmjereno **1.109 m u vožnji** ([[P-37_arm_position_gain_sag]]). Svaki sloj
koji odlučuje o prostoru — globalni planer, kritičari upravljača, inflacija — računao je s
robotom koji ne postoji otkad su se ruke zadnji put pomaknule.

## Odluka
Objavljivati **stvarni obris robota** na `footprint` topic oba costmapa, kontinuirano.

`Costmap2DROS` to podržava (`footprint_sub_`, `setRobotFootprintPolygon()`); dosad to nitko
nije koristio. Novi čvor `footprint_publisher` iz URDF-a i **žive TF** računa konveksni obris
cijelog robota projiciran na pod i šalje ga kao `geometry_msgs/Polygon`.

**Čvor nema nikakvu ovlast.** Opisuje, ne odlučuje: ne može zaustaviti, odbiti ni uvjetovati
vožnju. Ako TF nije spreman, šuti i costmap zadrži YAML vrijednost. To je namjerno i izravno
slijedi iz [[D-18_verified_baseline_first]] — sloj koji može odbiti vožnju ne ulazi bez runa
koji dokazuje da treba.

Uz njega ide `nav2_collision_monitor`, koji **isti taj footprint** projicira unaprijed po
zadanoj brzini i usporava prije dodira (`action_type: approach`). On je jedini sloj koji ne
ovisi o tome je li costmap vidio prepreku — bitno, jer ploču stola na 0.75 m lidar na 0.209 m
**ne može vidjeti**. `cmd_vel_relay` daje prednost njegovom izlazu, pa se ne može zaobići;
bez njega `/cmd_vel` vozi bazu kao i prije, pa teleop i mapiranje rade nepromijenjeno.

## Zašto ovo, a ne novi upravljač ili planer
Jedna izmjena popravlja **sve slojeve odjednom**, a ne mijenja arhitekturu: planer, kritičare
upravljača, inflaciju i sigurnost. MPPI (`OmniMotionModel`) i SE(2) planeri (Smac Hybrid-A*,
State Lattice) postoje na ovoj instalaciji i ostaju kao sljedeći koraci — ali tek nakon runa
koji pokaže koliko je od problema ovime nestalo.

## Posljedice
- Približavanje stolu postaje **strože** čim ruke strše: to je točno traženo ponašanje
  („dinamički mijenjati koliko udaljeno robot smije prolaziti"), ali znači i da se s raširenim
  rukama više ne može doći tako blizu. Prilaz za hvat je zaseban korak, još nije riješen.
- Redukcija poligona (kad točna ljuska ima previše vrhova) radi se **presjekom potpornih
  poluravnina**, dakle uvijek prema van. Aproksimacija footprinta sigurna je samo u jednom
  smjeru. Provjereno testom; na stvarnom URDF-u ljuska ionako ima 6 vrhova.
- Geometriju dijele `footprint_publisher` i `scripts/mesh_extent.py` (`robot_extent.py`), pa
  brojka koja se ispiše offline i poligon protiv kojeg Nav2 planira ne mogu se raziću.

## Provjera
`ros2 topic echo /local_costmap/published_footprint` mora pokazati poligon koji se mijenja
kad se ruke pomaknu; u RViz-u ga se vidi kako prati ruke. Referentna ruta iz [[runovi]]
(run 46) mora ostati vozna.
