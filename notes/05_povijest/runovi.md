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
| 39 | 13. 9. | necommitano | headless | tura s filtriranim skenom, stara simulacija | ❌ okret −90° dao samo −28.8° | zaustavljeno prije vrata | zatvorena petlja za okret; [[P-11_nav2_slam_drift]] |
| 40 | 13. 9. | necommitano | headless | nova simulacija, tura i SLAM | ❌ okret −89.9°, ravna dionica 4.5 m → 1.89 m, `map→odom` +1.04 m; stvarna širina ruku 1.109 m | prije plave sobe | [[P-11_nav2_slam_drift]], [[P-37_arm_position_gain_sag]] |
| 41 | 13. 9. | necommitano | GUI | `mapping_tour` uz sim + SLAM + MoveIt | ❌ MoveIt/JTC javili uspjeh, ali `left_joint_6` odstupio 0.614 rad; tura nije naredila vožnju. Korisnik vidio drift baze i spontano izlijetanje kutije. Naknadna read-only Gazebo poza kutije bila je (−6.24, 3.30, 0.20) umjesto (0, −6.38, 0.25). | provjera putne poze prije prve dionice | [[P-37_arm_position_gain_sag]], [[P-38_spontaneous_box_motion]] |
| 42 | 13. 9. | necommitano | GUI | `mapping_tour` nakon multi-layer detach ispravka i `mu2=0.2` | ❌ Kutija mirna na stolu (P-38 riješen), ruke stabilne u `ARM_CARRY_V2` (0.024 rad). Okret −90° OK. Na +4.5 m ravnoj dionici open-loop odstupio ~1.2°, zapeo za štok vrata (vrata 1.0 m, robot 85.4 cm); kotači proklizali, timeout odometrije. | prolaz vrata prema plavoj sobi | Prelazak na ručno mapiranje (teleop); [[P-38_spontaneous_box_motion]] riješen; [[P-11_nav2_slam_drift]], [[P-35_arm_span_too_wide_for_door]] |
| 43 | 13. 9. | necommitano | GUI | Ručno teleop mapiranje (`teleop_twist_keyboard`) uz `ARM_CARRY_V2` | ⚠ Karta dobra, ali korisnik primijetio rotacijsko kašnjenje skena pri okretima prije prelaska na konačni mecanum model | sve tri sobe | Prijelaz na puni mecanum model |
| 44 | 13. 9. | necommitano | GUI | Teleop mapiranje s popravljenim `mecanum_drive_controller` (100 Nm, `mu1=0.80`, `mu2=0.20`, auto `ARM_CARRY_V2`) | ✅ **Validirana finalna karta triju soba** ($11.9 \times 11.8$ m, $102.3\text{ m}^2$). Zidovi ravni i ortogonalni bez drifta, oba prolaza 1.0 m potpuno čista, 4 noge stola jasno razlučene. `scripts/check_map.py` prošao. Spremljeno u `maps/seminar_map.*`. | završeno mapiranje triju soba | Spremno za Nav2 AMCL lokalizaciju i planiranje |
