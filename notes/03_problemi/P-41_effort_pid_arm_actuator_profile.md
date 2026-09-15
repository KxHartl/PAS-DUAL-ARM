---
id: P-41
type: problem
status: djelomicno
requirements: ["[[R-06_realistic_parameters]]", "[[R-09_moveit_arm_control]]", "[[R-17_dual_arm_lift]]"]
solutions: ["[[S-03_ros2_control_setup]]", "[[S-08_grasp_squeeze_attach]]"]
decisions: []
updated: 2026-09-15
---
# P-41 — `effort` + PID profil ruku: pojačanja se ne smiju računati iz KDL matrice mase

## Simptom
Pod `force_grasp:=true` (ruke i vodilice na `effort` sučelju s PID-om u JTC-u):

1. Ruka ne drži naređenu pozu — pred-hvat promaši **0.22 m**, a greška se raspada s
   vremenskom konstantom **~12 s**. Za stisak koji radi korake od 0.25 s to je neupotrebljivo.
2. Zglobovi **1, 3, 5 i 7** titraju bang-bang: brzina mijenja predznak **svaki fizikalni
   korak (1 ms)**, do ±1.4 rad/s (granica brzine), moment ±30 Nm. Procjenitelj sile ih zbog
   toga odbacuje s `moving too fast for quasistatic estimation` — to je isti kvar koji je
   zaustavio prvi pokus procjene sile 15. 9.
3. Zglobovi nagiba (2, 4, 6) miruju pod istim pojačanjima.

## Uzrok
**Potvrđeno mjerenjem.** Dva odvojena uzroka:

**(a) `i_clamp` manji od gravitacijskog momenta.** JTC nema unaprijedno gravitacijsko
kompenziranje, pa je **integrator jedini nosilac gravitacije**. Bio je ograničen na
10 / 4 / 0.5 Nm, a granice aktuatora su 39 Nm (zglobovi 1–4) i 9 Nm (5–7). U ispruženoj
pozi zglob 2 traži ~19 Nm, pa je integrator ostao zasićen i greška trajna.

**(b) KDL matrica mase nije mjera efektivne inercije u DART-u.** Za **zakretne** zglobove
(1, 3, 5, 7 — vrte se oko vlastite osi članka) DART se ponaša kao da je ruka ~30× lakša.
Eksplicitna integracija čistog prigušenja stabilna je dok je `d < 2·I/dt`, pa granica
titranja mjeri `I` izravno (`scripts/` → `d_sweep`, dt = 1 ms):

| zglob | izmjereno `I` [kg m²] | KDL dijagonala | omjer | granica `d` = 2I/dt |
|---|---|---|---|---|
| 1 | 0.0172 | 0.491 | 0.035 | 34 |
| 2 | 0.4525 | 0.819 | 0.55 | 905 |
| 3 | 0.0118 | 0.355 | 0.033 | 24 |
| 4 | 0.2011 | 0.4395 | 0.46 | 402 |
| 5 | 0.0079 | 0.039 | 0.20 | 16 |
| 6 | 0.0177 | 0.0567 | 0.31 | 35 |
| 7 | 0.00155 | 0.00183 | 0.85 | 3.1 |

Gravitacijski momenti iz istog KDL modela **se slažu** s DART-om na 4 decimale
(9.383 vs 9.384 Nm), pa mase nisu krive — krivo je bilo uzeti dijagonalu matrice mase
kao efektivnu inerciju za projektiranje pojačanja. Zašto se zakretni zglobovi toliko
razlikuju ostaje otvoreno.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 15. 9., radno stablo | Codexova prenesena pojačanja p=80, i=5, d=2 (zglobovi 1–4) / 25, 2, 0.2 / 2, 0.2, 0.03 | zglobovi 5 i 7 titraju **0.79–0.81 rad/s** već u mirovanju; procjenitelj odbacuje sve uzorke | `d` zapešća **prenizak**, ne previsok |
| 2 | 15. 9., radno stablo | `i_clamp` 10/4/0.5 → **39/39/39/39/9/9/9** (granice momenta) | integrator prestao biti zasićen; greška pred-hvata i dalje 0.22 m, ali se raspada u ~12 s umjesto da stoji | uzrok (a) potvrđen i riješen; `p` je i dalje premalen |
| 3 | 15. 9., radno stablo | pojačanja iz **KDL** dijagonale matrice mase, ω=30/40 rad/s, ζ=1 | zglobovi 1, 3, 5, 7 u bang-bang titranju na 1 ms; `d(j1)=29.46` tik iznad izmjerene granice **34**… zapravo iznad stvarne granice jer je `I` bio precijenjen | KDL dijagonala nije upotrebljiva kao osnova |
| 4 | 15. 9., radno stablo | mjerenje `I` preko granice prigušenja (tablica gore), pa ω=**150 rad/s**, ζ=1, `i`=p/2, `i_clamp`=granica momenta | ✅ **nema titranja ni na jednom od 14 zglobova** (`\|v\|max` = 0.0000, moment p-p = 0.000); svi zglobovi točno na naredbi; staging obje ruke `Goal reached`; pred-hvat pogađa u **7 i 10 mm** (prag 20 mm) | profil je upotrebljiv; `p` i `d` oboje skaliraju s `I`, pa `d/d_max = ω·dt = 0.15` za svaki zglob |

## Trenutno rješenje
`scripts/apply_measured_arm_gains.py` računa pojačanja iz izmjerene inercije i upisuje ih u
`src/pas_dual_arm_bringup/config/force_controllers.yaml`. Projekt: kritično prigušen sustav
drugog reda na ω = 150 rad/s (ω·dt = 0.15 od granice stabilnosti), `i = p/2` (~2 s da
integrator preuzme gravitaciju), `i_clamp` = granica momenta zgloba.

## Sljedeći korak
Ponoviti mjerenje inercije u **pozi hvata**, ne samo u `ARM_HOME` — `I` ovisi o konfiguraciji,
a projekt je rađen na jednoj pozi. Kriterij: nema titranja i tijekom stiska s kockom.

## Ne ponavljati
- Računati pojačanja iz KDL matrice mase. Gravitacija se slaže, inercija ne.
- Postavljati pojačanja jedno po jedno preko `ros2 param set` na živom kontroleru: 56 uzastopnih
  poziva ostavlja ruku u nekonzistentnim međustanjima (novi `p` uz stari `d`) i trgne je.
- Tumačiti `i_clamp` kao sigurnosnu granicu. Integrator je jedina gravitacijska kompenzacija
  koju ruka ima; njegova granica mora biti granica aktuatora.
