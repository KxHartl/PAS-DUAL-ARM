---
id: D-21
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-06_realistic_parameters]]", "[[R-03_linear_rails_torso]]", "[[R-17_dual_arm_lift]]"]
problems: ["[[P-13_torso_prismatic_no_lift]]", "[[P-37_arm_position_gain_sag]]", "[[P-41_effort_pid_arm_actuator_profile]]"]
superseded_by: ""
updated: 2026-09-15
---
# D-21 — `effort` + PID kao zaseban profil pokusa, `position` ostaje zadan

## Kontekst
Dotad je vrijedilo pravilo korisnika: **nema regulatora, sve na `position` sučelju**, a ako
nešto ne stigne na cilj, to se izmjeri i zapiše umjesto da se zaobiđe PID-om
([[P-37_arm_position_gain_sag]], uputa u [[hvat_kocke]]).

Mjerenja 15. 9. pokazala su granicu tog pravila:

| | `position` (zadano) | `effort` + PID |
|---|---|---|
| vodilice torza pod težinom ruku | **ne miču se**: 0.0500 m umjesto 0.200 m, akcija javi `SUCCEEDED` | 0.2000 / 0.2000 m, trajna greška 0.0000 mm |
| ruke drže naređenu pozu | ne — plugin pretvara poziciju u brzinu uz gain 0.1 | da, pred-hvat u 8 mm |
| mjereni moment zgloba (`effort` state) | **radi** — 9.384 Nm, poklapa se s KDL gravitacijom | radi jednako |
| procjena sile, slobodna ruka | ~1e-9 N | ~1e-5 N |

Bitno: `effort` **state** sučelje radi u oba slučaja, jer ga
`kortex.ros2_control.xacro:69-73` deklarira neovisno o naredbenom sučelju, a instalirani
`ign_ros2_control` ga čita iz `JointTransmittedWrench` projiciranog na os zgloba. Mjerenje
momenta na kojem počiva cijela procjena sile **ne ovisi** o ovom izboru.

## Opcije
1. Ostati na `position` posvuda — ali tada se vodilice ne dižu uopće, pa nema ni dizanja kocke.
2. Prijeći na `effort` + PID posvuda — realni limiti momenta, ali traži ugađanje i pokazalo se
   osjetljivim ([[P-41_effort_pid_arm_actuator_profile]]).
3. Hibrid: `position` ruke, `effort` vodilice.

## Odluka
**Opcija 2**, i to kao zaseban profil koji se bira pri pokretanju: `sim.launch.py
force_grasp:=true` prebacuje ruke i vodilice na `effort` i učitava
`config/force_controllers.yaml`. Bez te zastavice ponašanje ostaje nepromijenjeno.
Odabrao korisnik 15. 9. 2026., uz uvjet da se i vodilice ugode ispod 1 mm.

Razlog za `effort` i na rukama, a ne samo na vodilicama: stvarni Gen3 ima granicu momenta
39 Nm (zglobovi 1–4) i 9 Nm (5–7). `position` sučelje u Gazebu vodi zglob kinematički, bez
ikakvog ograničenja sile, pa bi stisak kocke mogao razviti moment koji stvarni robot nikad ne
bi mogao ostvariti. `effort` profil te granice postavlja izravno na naredbeno sučelje.

## Posljedice
- Pojačanja se **moraju** računati iz izmjerene efektivne inercije, ne iz KDL matrice mase
  ([[P-41_effort_pid_arm_actuator_profile]]). Za to postoji `scripts/apply_measured_arm_gains.py`.
- Integrator je jedina gravitacijska kompenzacija koju ruka ima, pa mu granica mora biti
  granica momenta aktuatora.
- Pravilo „nema regulatora" **ostaje na snazi za misijski `position` profil**. Ovaj profil je
  ograničen na pokus hvata silom i ne mijenja `config/controllers.yaml`.
- [[P-37_arm_position_gain_sag]] ostaje otvoren za zadani profil; pod `force_grasp` je zaobiđen.

## Odnos prema zahtjevima
[[R-06_realistic_parameters]]: profil **pojačava** vjernost jer po prvi put nameće stvarne
granice momenta Gen3. [[R-03_linear_rails_torso]]: bez njega vodilice ne rade uopće, pa
zahtjev ne bi bio ispunjiv. Nije odstupanje od [ZAD]/[MAIL] — zadatak traži `ros2_control`,
a ne određeno naredbeno sučelje.
