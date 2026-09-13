---
id: P-35
type: problem
status: otvoreno
requirements: ["[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]", "[[R-11_door_80cm]]"]
solutions: ["[[S-06_navigation]]", "[[S-09_task_orchestration]]"]
decisions: ["[[D-13_three_room_world]]", "[[D-15_door_transit_behaviour]]"]
updated: 2026-09-13
---
# P-35: Širina robota s rukama — koja poza stane kroz vrata

## Simptom
Robot s rukama u `ARM_CARRY` je **~1.29 m** širok, a vrata su 0.9 m. Baza sama (0.6 m) prolazi bez
problema, ruke ne.

## Uzrok
**Potvrđeno:** ruke su na **bočnim** klizačima, pa su ramena već na y = ±0.26 m. `ARM_CARRY`
(`j2 = 0.7`, `j4 = -2.5`, `j6 = 1.2`) savija laktove **prema van**. Komentar u kodu
(`main_task.py:128-131`, „laktovi uvučeni, raspon ~0.6 m“) nikad nije bio izmjeren.
**Donja granica širine je ~0.64 m** (sama ramena ±0.26 + polumjer linka), bez obzira na pozu.

## Tri metode mjerenja i njihova pouzdanost
| Metoda | Kako | Rezultat za `ARM_CARRY_V2` | Pouzdanost |
|---|---|---|---|
| **(A) procjena** `capture_posture.py`, `measure_robot.py` | ishodišta linkova iz TF-a + **pretpostavljenih 0.06 m** | 0.87 m | gruba |
| **(B) sudar** `fit_test.py` | zid s prorezom u MoveIt sceni + binarna pretraga, prave kolizijske meshove | **0.856 m** | mjerodavna |
| **(C) vrhovi meshova** `mesh_extent.py` | svaki vrh mesha transformiran TF-om, zasebno vizualni i kolizijski | **0.854 m** | mjerodavna |

(B) i (C) su neovisne i slažu se na 2 mm. **Stvarna širina je 85.4 cm.**
Vizualni i kolizijski meshovi ruku su **identični** (razlika 0.0 cm), pa ono što RViz crta jest ono
što MoveIt računa.

### Dvije greške na putu do te brojke (13. 9.)
1. **Procjena (A)** je pretpostavila polumjer linka 6 cm. Bila je 1.5 cm preširoka.
2. **Prvi `fit_test`** je koristio zid debljine 10 cm na `x = 0`, pa su ruke koje sežu do
   `x = +0.70` **prošle pokraj zida**. Mjerio je presjek robota na `x = 0`, a ne najširu točku, i
   dao lažnih 83.4 cm. Popravak: prorez je **hodnik duljine 3 m** koji pokriva cijelog robota.

> [!tip] Zašto u RViz-u izgleda šire
> Zadani pogled je perspektivan: ruke su na z ≈ 0.4–0.8 m i bliže kameri od dna zidova, pa se čine
> šire. Za istinite omjere prebaciti pogled na **TopDownOrtho** (Views → Type) ili koristiti alat
> **Measure**.

## Mjerenje poza (13. 9. 2026., metoda A, MoveIt `/compute_fk` + `/check_state_validity`, `base_link`)
Širina = 2 × (max |y| svih linkova ruku + 0.06 m polumjera); „vrata“ = širina + 10 cm (pravilo korisnika).

| Poza | Širina | Vrata (+10 cm) | Doseg naprijed | Najširi link | Samokolizija |
|---|---|---|---|---|---|
| `ARM_HOME` | 1.41 m | 1.51 m | 0.63 m | podlaktica | OK |
| `ARM_CARRY` (trenutna) | 1.29 m | 1.39 m | 0.57 m | podlaktica | OK |
| **j2 = +90°, j4 = +90°** (tražena) | **0.85 m** | **0.95 m** | 0.48 m | podlaktica | ❌ **SUDAR** |
| j2 = +90°, j4 = 0° (ravno naprijed) | 0.90 m | 1.00 m | 1.08 m | prsti | OK |
| **j2 = +90°, j4 = +90°, j6 = −90°…−70°** | **0.85 m** | **0.95 m** | 0.74 m | podlaktica | ✅ **OK** |
| j2 = +90°, j4 = +70°, j6 = −35° | 0.85 m | 0.95 m | 0.83 m | podlaktica | ✅ OK |

**Zašto tražena poza pada:** pri j4 = 90° obje šake dođu u sredinu i dodiruju se
(`left_robotiq_85_*` ↔ `right_bracelet_link`, `right_robotiq_85_base_link` i obratno — 13 parova).
Zakret **zgloba 6 za ~−70°** razmakne šake po visini, širina ostaje 0.85 m, a poza postaje validna.
Simetrična varijanta (j2 = −90°, j4 = −90°, j6 = +70°) daje isto.

## Zaključak (metode B i C)
- **`ARM_CARRY_V2` je široka 85.4 cm.** Po pravilu „+10 cm“ → **vrata 95.4 cm**.
- Kroz vrata od **90 cm robot prolazi** s 2.3 cm po strani (tijesno, bez tražene rezerve).
  Otvor od **100 cm** daje 7.3 cm po strani — korisnik ga je 13. 9. prihvatio kao dobar.
- Najširi su redom: `spherical_wrist_2` (85.4 cm), `forearm` (85.0), `half_arm_1` (83.4),
  `shoulder` (82.5). Dakle **cijela ruka je blizu granice**, ne jedan istaknuti dio — zato nema
  lakog dobitka: ramena su fiksno na y = ±0.26 m.
- `ARM_CARRY` iz koda (1.29 m po metodi A) treba zamijeniti ovom pozom. To je izmjena koda,
  **nije napravljena**.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 23. 6. `45c32f1` | `ARM_CARRY` + vrata 1.2 m | prolaz headless uspio | prošao zbog **širokih vrata**, ne zbog uske poze |
| 2 | 13. 9. | `ARM_CARRY` izvršen preko MoveIt-a, pa izmjereni TF linkovi | MoveIt OK, širina **1.29 m** | poza neupotrebljiva za 0.9 m |
| 3 | 13. 9. | FK mjerenje 12 kandidata + provjera samokolizije | najuže = **0.85 m**; tražena j2/j4 = 90° pada na sudaru šaka | granica je ~0.85 m, ne 0.6 m |
| 4 | 13. 9. | sweep j2/j4/j6 za validnu usku pozu | **j2 = ±90°, j4 = ±90°, j6 = ∓70°** → 0.85 m, validno | kandidat za `ARM_DOOR` |
| 5 | 13. 9. | korisnik sam namjestio pozu u RViz-u (`ARM_CARRY_V2`, višekratnici 45°) | metoda A: 0.87 m | poza je i priprema za hvat i za nošenje |
| 6 | 13. 9. | `fit_test.py` sa zidom **debljine 10 cm na x = 0** | 83.4 cm | **neispravno**: ruke sežu do x = +0.70 i prošle su pokraj zida |
| 7 | 13. 9. | `fit_test.py` s **hodnikom duljine 3 m** | **85.6 cm**, prvi dodiruje `right_spherical_wrist_2_link` | ispravno |
| 8 | 13. 9. | `mesh_extent.py`: vrhovi svih meshova kroz TF, vizualni i kolizijski zasebno | **85.4 cm**, isto za obje geometrije | potvrđuje 7; vizual = kolizija |

## Sljedeći korak (NIJE rađeno: samo identificirano)
1. Odluka korisnika o širini vrata (0.95 / 1.0 m) prema izmjerenih 0.85 m.
2. Dodati `ARM_DOOR` pozu u `main_task.py` i koristiti je prije svakog prolaza.
3. Provjeriti da je iz `ARM_DOOR` moguće doći u press pozu (i s kockom u ruci).
4. Tek tada testirati prolaz ([[P-12_door_too_narrow]], [[D-15_door_transit_behaviour]]).

**Kriterij uspjeha:** validna poza ≤ 0.85 m, robot prođe vrata bez kontakta sa zidom.

## Ne ponavljati
- Vjerovati komentaru „ARM_CARRY ~0.6 m“ bez TF mjerenja.
- Poza s j4 = ±90° bez zakreta zgloba 6: šake se sudare.
- **Mjeriti širinu tankim zidom**: mjerna prepreka mora biti hodnik koji pokriva cijelu duljinu
  robota, inače istaknuti dijelovi prođu pokraj nje.
- **Procjenjivati širinu iz perspektivnog pogleda u RViz-u** ili iz ishodišta linkova s
  pretpostavljenim polumjerom.
