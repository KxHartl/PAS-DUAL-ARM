---
id: P-13
type: problem
status: rijeseno
requirements: ["[[R-03_linear_rails_torso]]", "[[R-06_realistic_parameters]]"]
solutions: ["[[S-01_robot_description]]", "[[S-03_ros2_control_setup]]"]
decisions: ["[[D-09_lift_with_arms_not_torso]]"]
updated: 2026-09-15
---
# P-13: Klizači torza (prismatic) se ne dižu pod težinom ruke

## Simptom
`torso_controller` naredi visinu, a opterećeni vertikalni klizač ostaje na donjem limitu (0.05).
Lagani zglobovi (pan-tilt) rade normalno.

## Uzrok
U izoliranom headless pokusu 14. 9. potvrđeno je da postojeći `position` command interface
pod teretom ne pokreće vodilice, iako JTC vraća `SUCCEEDED`. `effort` interface uz PID u JTC-u
pomaknuo je obje vodilice i držao cilj. To je dokaz da blokada nije bila nepremostiva
geometrijska kolizija; dinamiku kontaktnog hvata s dodatnom masom još treba provjeriti.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. `397f080` | efort limit 100 → 1000 N | bez promjene | nije samo efort |
| 2 | 30. 6. `397f080` | `min`/`max` na command interfaceu | bez promjene | — |
| 3 | 30. 6. `397f080` | `position_proportional_gain` (sada 20) | bez promjene | — |
| 4 | 14. 9., `feature/cube-manipulation`, C1 | postojeći pozicijski kontroler, cilj 0.20 m s rukama u `ARM_CARRY_V2` | akcija `SUCCEEDED`, stvarno 0.050/0.050 m | nema stvarnog pomaka; status akcije nije dokaz |
| 5 | 14. 9., `feature/cube-manipulation`, C2 | `effort` interface + JTC PID: p=1500, i=500, d=100, i_clamp=300; isti cilj | nakon 2 s 0.1944/0.1944 m; nakon 10 s 0.2000/0.2000 m; povratak na 0.0500/0.0500 m | vodilice rade pod težinom ruku i drže visinu |
| 6 | 15. 9. `ee4dd4d` | **`position` sučelje na `main`-u** (Codexov `effort` odbačen), `table_ready` zada 0.200 m preko `torso_controller`, ruke u `table_arms` pozi, GUI | akcija `SUCCEEDED`, stvarno **0.0500 / 0.0500 m** — nula pomaka | potvrđen C1: pozicijsko sučelje pod teretom ne pomiče vodilice **uopće**, ne samo djelomično |
| 7 | 15. 9., radno stablo | `effort` + PID (ista pojačanja kao C2), mjereno **nakon stvarnog smirenja** umjesto nakon fiksne pauze od 3 s | **0.2000 / 0.2000 m, greška +0.0000 mm**, razmak lijevo-desno 0.0000 mm; ponovljeno u dva uzastopna diza | Vodilice trajno pogađaju cilj. Onih „3 mm“ koje je `table_ready` prijavljivao bilo je puzanje uhvaćeno prerano, ne trajna greška |
| 8 | 15. 9., radno stablo | Zadnji pokušaj da **pozicijsko** sučelje proradi: gain postavljen na sva četiri moguća načina (detalji u [[P-37_arm_position_gain_sag]] #8) | ❌ vodilice se ne pomaknu ni 0.0 mm ni kad je parametar potvrđeno 20.0. Isti čvor, ista naredba, `effort` profil ih digne na 0.2000 m | **Pozicijsko sučelje je iscrpljeno.** Vodilice pod teretom rade samo preko `effort` + PID |
| 9 | 15. 9., radno stablo | **Izmjeren sam zglob, a ne status akcije.** Naređeno 0.20 m na `torso_controller`, pa 10 s praćeno `/joint_states`. Zatim ista stvar na **rotacijskom** zglobu (`pan_tilt_pitch_joint`) u **istom** runu i profilu | Vodilica: pomak **0.0000 mm**, brzina **0.00000 m/s** — ne sporo, nego **nikakvo**; akcija javi `status 4`. Pan-tilt: pomak 0.19961 rad, brzina 0.0599 rad/s, stigao na 0.64961 (cilj 0.65). `ros2 control list_hardware_interfaces`: `torso_*_carriage_joint/position [available] [claimed]`, isto kao pan-tilt | **Uzrok nije gravitacija, ni gain, ni MoveIt.** Kontroler sučelje uredno preuzima i piše u njega, ali ga plugin ne pretvori u gibanje. Jedini prismatic zglobovi u robotu su upravo vodilice; svi rotacijski na istom profilu rade. Hipoteza: `ign_ros2_control` u ovoj verziji ne vodi **prismatic** zglobove preko `position` sučelja |
| 10 | 15. 9., radno stablo | **Uzrok nađen.** Vodilica je puštena s 0.30 m i naređena na 0.20 — pomakne se 99.95 mm, brzina **0.01499 m/s**, točno predviđenih `0.1 × 0.15`. Zatim spuštena **na** limit 0.05 (prijeđe punih 150 mm) pa naređena natrag gore | Prema limitu: **150.00 mm** ✅. S limita natrag: **0.00 mm** ❌, a akcija javi uspjeh | **Zglob se zaledi kad sjedne točno na graničnik**, i to u **oba** smjera. Ignition poziciju pretvara u brzinu zgloba, a na limitu se ta brzina ponišava. `initial_value` je bio **0.05**, što je ujedno i `lower` — robot je startao već zaglavljen. Nije gravitacija, nije gain, nije prismatic tip, nije MoveIt |
| 11 | 15. 9., radno stablo | `initial_value` 0.05 → **0.06** (1 cm iznad graničnika), **zadani pozicijski profil**, bez ijednog regulatora | ✅ `carriages before: 0.0600`, `CARRIAGE MEASURED left=0.2000 right=0.2000 m, error +0.0000 / +0.0000`, `staging done (carriages=ok)`. Dizanje pod teretom ruku: 0.20 → 0.35 → 0.20 m, po **150.00 mm**, točno na pet decimala | **Riješeno.** Vodilice rade na običnom `position` sučelju s MoveItom. `effort` profil im nije potreban |

## Zašto se činilo da rade

`scripts/joint_gui.py:170` objavljuje `sensor_msgs/JointState` **izravno na `/joint_states`** —
to je zamjena za `joint_state_publisher_gui` za RViz, bez ikakve fizike. Ondje naredba *jest*
stanje, pa se vodilice pomiču. Isti obrazac je i u referentnom projektu BATRACS
(`mock_components/GenericSystem`). Gazebo s DART fizikom je drugi slučaj.

## Rješenje (15. 9.)
`initial_value` vodilica postavljen na **0.06** umjesto 0.05
(`src/pas_dual_arm_bringup/urdf/robot.urdf.xacro`, `torso_system`). Jedina izmjena.

Sve prije toga — dizanje efort limita na 1000 N, `min`/`max` na sučelju, `position_proportional_gain`,
prelazak na `effort` + PID — liječilo je simptom. Robot je startao **zaglavljen na vlastitom
graničniku**, a ondje Ignition ne miče zglob ni u jednom smjeru jer poziciju izvodi kao brzinu.

Zato je i `effort` profil radio: on ne ide kroz taj put nego primjenjuje moment.

## Staro rješenje (povijesno)
Na radnoj grani `feature/cube-manipulation` `torso_controller` šalje `effort` kroz PID.
Pokus je proveden u zasebnoj ROS domeni 75 i Gazebo particiji; skripta
`scripts/probe_torso.py` mjeri stvarne zglobove. Misijski `main_task` još podiže rukama,
pa [[D-09_lift_with_arms_not_torso]] ostaje opis starog misijskog slijeda dok se ne
integrira i ne provjeri dizanje kocke vodilicama.

## Sljedeći korak
Provjeriti da obje vodilice drže visinu tijekom obostranog kontaktnog hvata s kockom
i da im stvarne poze ne razmiču jastučiće. Tek tada zamijeniti dizanje rukama u misiji.
Trajna točnost bez tereta više nije otvorena (pokušaj 7); otvoreno je držanje **pod kockom**.

`table_ready.py:stage_carriages` sada čeka da se vodilice stvarno zaustave (položaj miran
unutar 0.02 mm kroz 3 s, prekid nakon 40 s) umjesto fiksne pauze. Mjerenje po uzastopnim
uzorcima nije dovoljno: 1 mm kroz 10 s pomakne manje od 0.02 mm između susjednih očitanja i
prošlo bi kao mirovanje.

## Ne ponavljati
- Dalje dizati efort ili gain (probano do 1000 N).
- Tražiti krivca u MoveItu, u gainu ili u gravitaciji. Izmjereno (pokušaj 9): naredba stigne do
  sučelja, sučelje je `claimed`, a zglob se ne pomakne **ni 0.0 mm**. Rotacijski zglob u istom
  runu i profilu radi normalno. `effort` pomakne **isti** prismatic zglob na 0.2000 m, dakle DART
  ga zna micati — ne vodi ga `position` put.
