---
id: P-43
type: problem
status: u tijeku
requirements: ["[[R-19_door_pass_with_box]]", "[[R-17_dual_arm_lift]]", "[[R-11_door_80cm]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]"]
decisions: ["[[D-05_contact_verified_attach]]", "[[D-12_honesty_abort_over_fake]]"]
updated: 2026-09-15
---
# P-43 — Poza hvata šira od vrata

## Simptom
Korisnik, 15. 9. (GUI): *„pogledaj pozu koju smo definirali za prolaz kroz vrata/prolaze.
moramo kutiju uhvatiti s pozom koja ima istu sirinu, inace ne cemo proci kroz vrata."*

Poza za prolaz `ARM_CARRY_V2` je **0.840 m** široka ([[P-35_arm_span_too_wide_for_door]]).
Poza pritiska kojom se kocka hvatala izmjerena je na **1.531 m** (`mesh_extent.py`). Vrata
su **1.0 m**. S kockom u rukama robot ne može proći ni jedna vrata, pa [[R-19_door_pass_with_box]]
nije ostvariv tim hvatom, koliko god on sam bio dobar.

## Uzrok — izmjereno
Širina hvata ima **dva** dijela, a samo je jedan bio vidljiv.

**1. Loša IK grana (0.53 m viška).** Planer je birao granu s laktovima vani. To se da
popraviti izborom grane.

**2. Zapešće na osi prilaza (neizbježno kod vodoravnog prilaza).** Izmjereno offline preko
cijelog nul-prostora (`scripts/grasp_width.py`, 1620 IK rješenja po ruci): kod vodoravnog
pritiska **svako** rješenje ima istu polu-širinu, **0.502 m**. Najširi je
`spherical_wrist_1/2`, a on leži na samoj osi prilaza:

| dio | doprinos po strani |
|---|---|
| pola kocke | 0.150 m |
| zapešće → vrh prsta (`tip_standoff`) | 0.149 m |
| sferno zapešće izvan ishodišta EE | 0.2025 m |
| **ukupno** | **0.5015 m → 1.003 m** |

Lakat se može uvući, zapešće ne može: ono je na pravcu kojim prst ide u plohu. Nikakav izbor
IK grane ne spušta vodoravni hvat ispod 1.003 m.

## Rješenje — nagnuti prilaz
Os prilaza se nagne **prema dolje**, pa zapešće ide **gore** umjesto van. Vrh prsta i dalje
cilja **središte bočne plohe**, pa kontaktni senzori i dalje vrijede za [[R-17_dual_arm_lift]].

Izmjereno istim alatom, kocka na 0.62 m, sve visine vodilica 0.20–0.35 m daju isto (± 2 mm):

| nagib | širina | najširi dio |
|---|---|---|
| 0° | 1.003 m | sferno zapešće |
| 30° | 0.937 m | sferno zapešće |
| 40° | 0.881 m | sferno zapešće |
| 45° | 0.847 m | sferno zapešće |
| **50°** | **0.823 m** | **rame** |
| 55°, 60° | 0.823 m | rame |

Od ~50° širinu drži **nosač ramena** (0.412 m po strani), koji je donja granica ovog robota
bez obzira na pozu ruku. Dalje naginjanje ne donosi ništa. **50° je uže od `ARM_CARRY_V2`**
(0.840 m), poze koja je potvrđena kroz vrata.

Parametar `press_tilt` = 0.873 rad (50°) u `main_task.py`.

### Zamka koju treba izbjeći
Kod nagnute osi **ne smije** se odmicati od središta kocke duž osi, kako je to radila stara
(vodoravna) verzija. Vrh prsta tada sjedne `half·tan(nagib)` previsoko: kod 45° to je
0.15 m, dakle **gornji rub** kocke od 0.30 m, a ne njezina ploha. Cilja se točka na plohi, pa
se zapešće odmakne duž osi od nje. Provjereno: kod 50° vrhovi sjedaju na y = ±0.150,
z = 0.820 (središte plohe), zapešća na z = 0.934.

### Zamka 2 — rotacija šake oko osi prilaza
Pri uvođenju nagiba os x alata greškom je okrenuta (`sgn·(−vy, vx)` umjesto `sgn·(vy, −vx)`).
Šaka je time rotirana 180° oko osi prilaza: kamera na zapešću sjedne **5.6 cm ispod** osi umjesto
iznad, a marker na plohi, podignut 5.6 cm upravo za kameru iznad osi, ispadne iz kadra (run S1:
oba zapešća `marker NOT seen`). Vraćeno na staru orijentaciju. Offline brojevi širine
(`grasp_width.py`) računani su sa **starom** orijentacijom, pa vrijede i dalje; run F12 je imao
okrenutu šaku, ali je pao prije ikakvog pokreta.

## Pokušaji

| # | Datum | Što | Rezultat | Zaključak |
|---|---|---|---|---|
| 1 | 15. 9. | IK pred-poze sijan iz `ARM_CARRY_V2` umjesto iz tekuće poze (run F10) | ❌ nađena uža grana, ali plan do pred-poze iz nje pada; obje ruke **0.46 m** od kocke. Ispis „0.952 m" bio je širina **promašene** poze, ne hvata | Širina je svojstvo **ciljne poze**, ne sjemena. Vraćeno |
| 2 | 15. 9. | `measure_width()` u misiji uspoređen s `mesh_extent.py` | ⚠ misija javila 0.966 m, `mesh_extent` 1.531 m. `ExtentMeasurer.points()` **tiho preskače** svaki link koji TF ne može smjestiti | `measure_width()` sada broji smještene linkove i **odbija** dati broj ako fali ijedan (`width UNRELIABLE`) |
| 3 | 15. 9. | Offline FK + IK nad URDF-om, cijeli nul-prostor, vodoravni prilaz (`scripts/grasp_width.py`) | 1620/1632 rješenja, **sva** 0.502 m po strani | Uzrok 2 potvrđen: vodoravni hvat ne može ispod 1.003 m |
| 4 | 15. 9. | Isti alat, nagib 0–60°, vodilice 0.20–0.35 m | 50° → **0.823 m**, i dalje ravno | Nagnuti prilaz, `press_tilt` = 50° |
| 5 | 15. 9. | Misija s nagnutim prilazom 50°, headless (run F12) | ⚠ pošten abort prije pokreta ruku: MoveIt vidi start desne ruke u sudaru sa stolom | Nije problem nagiba. `ARM_CARRY_V2` stoji 2.4 cm ispod stvarne ploče, a MoveIt model stola je 10 cm debeo (stvaran 4 cm) |
| 6 | 15. 9. | Offline: postoji li visina vodilica na kojoj je `ARM_CARRY_V2` kod kocke iznad stola i uz kocku | ❌ ne postoji: 0.20 m ispod ploče (2.4 cm), 0.35–0.65 m prsti **u kocki** (0.0 cm) | Uska poza se ne smije voziti do kocke. Alternativa: pred-poza na mjestu mjerenja, pa primicanje — 17.9 cm iznad stola, 5.7 cm od kocke |

## Što još nije provjereno
- Prolazi li nagnuti pritisak ravnom Kartezijevom linijom (`press_both_linear`) — offline alat
  provjerava samo krajnju pozu, ne put do nje.
- Drži li nagnuti dodir kocku jednako dobro: sila prsta sada ima komponentu prema dolje.
  Kod krutog spoja (D-05) to ne mijenja nošenje, ali mijenja trenutak kontakta.
- **GUI potvrda korisnika.** Do nje ništa od ovoga nije riješeno.
