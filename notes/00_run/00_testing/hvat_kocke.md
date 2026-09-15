---
id: RUN_TEST_GRASP
type: upute
updated: 2026-09-15
---
# Testiranje: dolazak hvataljkama u točke hvata i dizanje

Slijed testa, kako ga je korisnik zadao:

> 1. robot dođe na poziciju pred stol
> 2. digne ruke vodilicama na visinu stola
> 3. ako treba, približi se još malo stolu
> 4. svaku ruku vodimo u točku koju treba
> 5. kada ruke dođu u točke, podižemo vodilice za 10 cm

Bez pritiska, bez attacha, bez gate-ova. To je **prvi test** — gleda se u GUI-ju što se dogodi,
pa se tek onda odlučuje dalje. Vidi [[R-17_dual_arm_lift]].

> [!important] Sve ide na `position` sučelje
> Ni ruke ni vodilice nemaju regulator. Ako nešto ne stigne na cilj, to se **izmjeri i zapiše**,
> ne zaobiđe se PID-om.

> [!warning] Korak 2 trenutačno ne prolazi
> Izmjereno 15. 9. 2026.: vodilice pod težinom ruku ostaju na 0.0500 m iako je naređeno 0.200 m,
> a akcija javi `SUCCEEDED`. Vidi [[P-13_torso_prismatic_no_lift]]. U `scripts/joint_gui.py`
> (RViz, bez fizike) se pomiču jer ondje naredba izravno postaje stanje.

---

## Korak 1 — robot na poziciju pred stol

Dock poza iz `nav_zones.py`: `dock = table_half (0.40) + HALF_LENGTH (0.52) +
table_dock_safety (0.10) = 1.021 m` od centra stola. Plavi stol je u `(0, −6.5)`, prilazi se
s `+Y` strane.

| | vrijednost |
|---|---|
| dock poza | `x = 0.0`, `y = −5.479`, `yaw = −90° (−1.5708 rad)` |
| centar kocke odatle | ~0.87 m ispred `base_link` |
| ploha stola | `z = 0.75 m`, centar kocke `z = 0.90 m` |

Za sam test robot se spawna na dock pozi — ne troši se vrijeme na punu vožnju:

Svi terminali prvo:
```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
```

**Terminal 1** (simulacija + RViz; spawn na dock pozi, zapešća iznad plohe):
```bash
bash scripts/run_cube_isolated.sh ros2 launch pas_dual_arm_bringup sim.launch.py headless:=false rviz:=true table_arms:=true robot_spawn_x:=0.0 robot_spawn_y:=-5.479 robot_spawn_yaw:=-1.5708
```

`rviz:=true` otvara prikaz `rviz/cube.rviz` — robot, TF markera i obiju hvataljki, oblak dubinske
kamere i **poze hvata** (`/cube_grasp/poses`, narančaste osi) čim ih skripta iz koraka 4 izračuna.
Sve je uključeno odmah; ništa se ne dodaje rukom.

`table_arms:=true` spawna zapešća **iznad** plohe. Carry poza ih drži na 0.49 m, a ploha je na
0.75 m — zato su ruke dotad zapinjale za rub stola.

Kad je test gotov, puna vožnja do te poze ide preko `room_navigator` (`blue:dock` na
`/room_navigator/goto`), ne izravno na `/navigate_to_pose`.

**Terminal 2** (MoveIt i ArUco — **obavezno `auto_start:=false`**):
```bash
bash scripts/run_cube_isolated.sh ros2 launch pas_dual_arm_bringup task.launch.py auto_start:=false
```

> [!danger] Nikad dvije simulacije odjednom
> Oba Gazeba objavljuju `/clock` na istoj domeni, pa vrijeme skače naprijed-natrag. RViz javlja
> `Detected jump back in time` stotinama puta u sekundi, resetira se na svaki, i na kraju pukne s
> `Cannot create GL vertex buffer`. Iz simptoma se uzrok ne vidi. Prije pokretanja:
> ```bash
> bash scripts/clean_ros.sh
> ```
> `run_cube_isolated.sh` to od 15. 9. i sam odbija — javi `GRESKA: simulacija vec radi`.

> [!important] `grasp_cube.py` je jedini vlasnik gibanja
> `task.launch.py` sada ima `auto_start:=false` kao zadanu vrijednost. Stari `main_task`
> orkestrator pokreće se samo eksplicitnim `auto_start:=true`. Pričekati da T1 ispiše završetak
> `table_ready` čvora prije pokretanja T3.

**Terminal 3** ostaje slobodan za korake 2–4. Do tada se robot ne smije micati sam od sebe —
jedino što se pomakne bez tvoje naredbe su vodilice i ruke u `ARM_HOME`, iz čvora `table_ready`.

## Korak 2 — vodilice dižu ruke na visinu stola

Čvor `table_ready` krene sam čim se kontroleri aktiviraju: zada visinu vodilica kao običnu
trajektoriju na `torso_controller` (isto kao ruke, `position`, bez regulatora), pa **očita
`/joint_states`** i javi stvarnu visinu. Status akcije se ne uzima kao dokaz.

U logu terminala 1 tražiti redak:
```
CARRIAGE MEASURED left=… right=… m (commanded 0.200, error …, action reported …)
```

Ponoviti mjerenje zasebno, s drugom visinom (**Terminal 3**):
```bash
bash scripts/run_cube_isolated.sh python3 scripts/probe_torso.py --height 0.20
```

Taj broj ide u tablicu pokušaja [[P-13_torso_prismatic_no_lift]], kakav god bio.

## Korak 3 — lociranje kocke i (ako treba) primicanje

Prvo mjerenja, robot se **ne miče**.

**Terminal 3** — centar kocke iz markera + dubine, obje točke hvata, IK:
```bash
bash scripts/run_cube_isolated.sh python3 scripts/probe_cube_perception.py --ros-args -p use_sim_time:=true
```

**Terminal 3** — treba li se uopće primicati: IK na stvarnoj dock udaljenosti:
```bash
bash scripts/run_cube_isolated.sh python3 scripts/probe_cube_ik_variants.py --ros-args -p use_sim_time:=true
```

Prihvatljivo: `EXTENT` duga os **0.300 ± 0.01 m**, `AGREEMENT` `xy ≤ 0.03` i `z ≤ 0.05`.
Z se uzima **iz markera** (on je na sredini plohe); dubina vidi samo gornji dio kocke pa joj je
Z sredina ~2.7 cm previsoka.

Ako IK prolazi na ~0.87 m → baza se ne miče. Ako pada, primiče se odometrijom (`drive_distance`),
izvan Nav2. Granica je noga stola na `y = −6.125`. Nakon pomaka marker zna izaći iz vidnog polja
→ naciljati kameru i **ponovno izmjeriti** prije pomicanja ruku.

## Koraci 3–5 — jedna naredba

Sve od lociranja do dizanja je u jednoj skripti, u **jednom terminalu**:

```bash
bash scripts/run_cube_isolated.sh python3 scripts/grasp_cube.py --ros-args -p use_sim_time:=true
```

Zaustavljanje ranije: `--pregrasp-only` (ništa ne dodiruje) ili `--no-lift` (stane na kontaktu).

Slijed:

| #   | radnja                                                                     |
| --- | -------------------------------------------------------------------------- |
| 1   | glavna kamera izmjeri kocku (marker + dubina)                              |
| 2   | baza se primakne na doseg ruku, uz granicu stola                           |
| 3   | hvataljke se otvore, obje ruke **paralelno** na pred-poze (20 cm od ploha) |
| 4   | svaka ruka očita **svoj** marker na plohi koju će pritisnuti               |
| 5   | obje ruke brzinom najviše 25 mm/s na 2 cm od ploha i na **istu visinu**   |
| 6   | obje ruke prilaze 2 mm/s; svaka staje na svom prvom kontaktu              |
| 7   | obje hvataljke se zatvore; nepotpuna ruka radi korake 2 mm do 4/4         |
| 8   | nakon stabilne 1 s vodilice dignu 20 cm brzinom 10 mm/s                   |

> [!important] Visina se poravnava prije dodira, i poslije se ne dira
> Korak 5 obje ruke dovede na **istu visinu** dok još ništa ne dodiruju. U koraku 6 svaka ruka
> zadrži visinu na kojoj **stvarno jest** i miče se čisto vodoravno.
>
> Ispravljanje visine **dok se pritišće** je ono što je 15. 9. nakosilo kocku: zapešće se pomakne
> po z uz plohu koju već dodiruje, kocka se nagne, desna popusti, lijeva učini isto, i obje je
> onda drže ukoso.

> [!important] Svaka ruka staje zasebno
> Od standoffa od 2 cm obje ruke krenu zajedno, ali cilj pojedine ruke otkazuje se na njezinu prvom
> kontaktu. Zatim se samo ruka koja još nema 2/2 kontakta primiče u koracima od 2 mm;
> dovršena ruka miruje. Z ostaje trenutačni, a orijentacija se vraća na onu izvedenu iz
> markera, najviše 10 mm preko izmjerene plohe. Ako sva četiri kontakta nisu stabilna 1 s,
> dizanje ne počinje.

Ako naredba od 2 mm tri puta zaredom daje manje od 0,2 mm stvarnog pomaka,
nepotpuna ruka se odmakne 3 mm radi rasterećenja i poravnanja prije nastavka.

Tijekom friction-only dizanja nema `DetachableJoint` attacha. Gubitak kontakta zaustavlja
vodilice, vraća postupak na ograničenu finu korekciju i zatim nastavlja prema istom cilju
`početna visina + 20 cm`. Iscrpljen hod ili nemoguć povrat kontakta prekidaju run.

---

## Što zabilježiti

| # | podatak | kartica |
|---|---|---|
| 1 | naređena vs. stvarna visina vodilica | [[P-13_torso_prismatic_no_lift]] |
| 2 | centar kocke i obje točke hvata u `base_link` | [[R-17_dual_arm_lift]] |
| 3 | je li IK prošao na dock udaljenosti, ili koliko se trebalo primaknuti | [[P-25_asymmetric_arm_reach]] |
| 4 | odstupanje stvarne poze hvataljki od zadanih | [[P-37_arm_position_gain_sag]] |
| 5 | što se dogodilo pri +10 cm na vodilicama | [[R-17_dual_arm_lift]] |

Svaki run → novi redak u [[runovi]].
