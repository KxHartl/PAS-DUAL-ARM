---
id: RUNOVI
type: povijest
updated: 2026-09-13
---
# Runovi i eksperimenti

> Logovi pojedinih runova nisu sačuvani. Ovo je rekonstrukcija iz STATE.md i commit poruka.
> **Od danas svaki run upisati ovdje** (datum, commit, GUI/headless, ishod, gdje je stao, zaključak).

## Sažetak po razdobljima
| Razdoblje | Meta | Runova | Ishod | Izvor |
|---|---|---|---|---|
| 23. 6. (M6) | kocka na podu, teleport | nekoliko, headless | „uspjeh“, ali lažan hvat | `45c32f1`, [[P-16_fake_teleport_grasp]] |
| 30. 6. | šipka / ploča | više, GUI + headless | lift s attachom radi; trenje ne drži; place na isti stol | `397f080`…`5c9e201` |
| 15.–16. 7. | kocka 0.3 m | ~30 (runovi 1–31) | **3 puna ciklusa hvat + lift**; ~15 poštenih aborta; nula lažnih attacha | `c504720`, STATE.md |
| 16. 7. (b) | kocka | 29, 30, 31 | 29/30: stvaran obostrani kontakt, abort na geometriji; 29: kocka zbačena na pod; 31 prekinut | STATE.md, [[P-28_gate_too_strict]] |

**Uspješnost s kockom** (do 16. 7.): ~35 % (3/9 runova koji su došli do kocke).

## Dnevnik (od 13. 9.)
| # | datum | commit | način | cilj runa | ishod | stao na | zaključak / kartica |
|---|---|---|---|---|---|---|---|
| 31 | 16. 7. | `73617e8` | GUI | provjera popuštenog gatea | prekinut krajem sesije | — | [[P-28_gate_too_strict]] |
| 32 | 13. 9. | `2bda09e` | GUI | podizanje novog svijeta (tri sobe) | ✅ svijet valjan (`ign sdf -k`), 8/8 kontrolera aktivno, korisnik potvrdio raspored | — | [[D-13_three_room_world]] |
| 33 | 13. 9. | (necommitano) | GUI | SLAM + Nav2 + `cmd_vel_relay` podignuti | ✅ stack se digao, `/map` objavljen (158×158, 0.05 m/px); Nav2 javio da je `inflation_radius` (0.05) manji od upisanog radijusa (0.31) | nije vožen nijedan cilj | [[P-12_door_too_narrow]] |
| 34 | 13. 9. | (necommitano) | GUI | `ARM_CARRY` preko MoveIt-a + TF mjerenje širine | ⚠ MoveIt OK, ali **širina 1.26 m** (laktovi y = ±0.58) | prolaz kroz 0.9 m nije ni pokušan | **[[P-35_arm_span_too_wide_for_door]]** (novo) |
| 35 | 13. 9. | `7f24fbd` | RViz | korisnik namjestio pozu `ARM_CARRY_V2` (višekratnici 45°) | ✅ snimljena, klizači 0.8 → 0.65 m | — | [[08_poze]] |
| 36 | 13. 9. | `35565cb` | MoveIt | mjerenje širine hodnikom s prorezom + vrhovima meshova | ✅ **85.4 cm**, prva mjera (83.4) bila kriva zbog tankog zida | — | [[P-35_arm_span_too_wide_for_door]] |
| 37 | 13. 9. | `e99316e` | GUI | novi svijet 6×6 m, vrata 1.0 m | ✅ 8/8 kontrolera, korisnik potvrdio raspored | — | [[D-13_three_room_world]] |
| 38 | 13. 9. | `cae0376` | GUI | prilagodba `<gui>` (FOV 46°/60°, ViewAngle plugin) | ❌ plosnato, panel odsječen, kamera odlutala na x≈310 m | vraćeno na zadani GUI | [[S-02_world_and_sim_launch]] |
| 39 | | | | | | | |
