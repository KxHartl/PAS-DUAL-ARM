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
| 32 | | | | | | | |
