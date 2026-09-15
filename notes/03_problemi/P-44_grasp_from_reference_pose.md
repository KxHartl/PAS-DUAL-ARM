---
id: P-44
type: problem
status: u tijeku
requirements: ["[[R-17_dual_arm_lift]]", "[[R-19_door_pass_with_box]]", "[[R-03_linear_rails_torso]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]"]
decisions: ["[[D-05_contact_verified_attach]]", "[[D-12_honesty_abort_over_fake]]"]
updated: 2026-09-15
---
# P-44 — Hvat preko korisnikovih poza (V3 → V4)

## Zahtjev korisnika (15. 9.)
> *„napravio sam grasp_3 taj mi se najviše sviđa. ali to je okvirna poza, referenca. to neka mu
> je default poza, vodilice na 500. a kada ide detektirati kutiju neka onda prvo detektira kutiju,
> locira, definira bočne točke u koje treba doći ruka za određivanje bočnih markera. skenira
> bočne markere, dobijemo targete u koje želimo doći s vrhom alata. i onda dolazimo u te točke
> da konačna poza bude što sličnija grasp_3."*

Odgovori na pitanja: za vožnju vodilice **~200 mm**, dizanje kod stola; skeniranje kao
`GRASP_V3` ali sa **izravnanim zapešćem** (kamera vodoravno); vodilice na **500 mm**, fino
podešavanje rukama **uz zadržanu orijentaciju alata**; hvataljke **zatvorene** (jastučić).

## `GRASP_V3` — izmjereno offline (`log/analysis/grasp_v3_check.py`)
| | vrijednost |
|---|---|
| širina (collision meshevi) | **0.831 m** — uža od `ARM_CARRY_V2` (0.840 m) |
| prilaz šaka | **55° prema dolje**, simetrično |
| vrhovi prstiju (vodilice 500) | 2.1 cm od ploha, 5.6 cm ispod sredine plohe, kocka 0.58 m ispred |
| glavna kamera prema kocki s docka | zaklonjeno 0/162 (vodilice 200) i 4/162 (500) zraka |
| lidar | ruke najniže 0.49 m iznad poda (lidar 0.21 m) |

## Ograničenje: gdje se vodilice smiju dići
`GRASP_V3` seže 0.71 m naprijed. Na visini vožnje šake su **ispod** ploče stola, pa se kroz nju
ne mogu dići (`log/analysis/raise_distance.py`):

| kocka ispred robota | razmak pri dizanju 200 → 500 mm |
|---|---|
| 0.87 m (dock) | **0.0 cm** |
| 0.92 m | 1.3 cm |
| 0.97 m | 6.3 cm |
| **1.03 m** | **12.3 cm** ← `raise_distance` |

Na docku je `GRASP_V3` s vodilicama na 200 mm **u** ploči (0.0 cm). Zato i `pose_studio` sada
spawna robota 1.05 m od kocke.

## Slijed (ugrađen u `main_task`)
1. vožnja u `GRASP_V3`, vodilice 200 mm;
2. vizualni prilaz do **1.03 m**, mjerenje kocke glavnom kamerom (marker + dubina);
3. vodilice na **500 mm**;
4. **skenirajuća poza** — `GRASP_V3` s izravnanom osi prilaza, otvorene šake, vrhovi 0.20 m od
   ploha kocke kakva će biti nakon primicanja; kamera na zapešću točno na visini markera
   (0.877 m), gleda vodoravno;
5. primicanje na **0.58 m** i centriranje strafeom — skenirajuća poza ≥ 10.8 cm od stola i nogu,
   ≥ 19.7 cm od kocke cijelim putem;
6. kamere na zapešću mjere plohe → centar, razmak i zakret kocke;
7. zatvorene hvataljke; **pred-hvat i hvat računati od `GRASP_V3`**: njegova orijentacija alata
   (zakrenuta za kocku), jastučići na središtu ploha, `squeeze_interference` u plohu;
8. MoveIt plan do pred-hvata (5 cm natrag po osi prilaza), pa **ravno 5 cm** brzinom 1 cm/s;
9. dokaz: dodir na obje ruke **i** šake na cilju → attach (D-05) → dizanje vodilicama, najviše
   do **640 mm** (na 650 mm vodilica stoji na graničniku i zaledi se, P-13).

## Koliko je konačna poza slična `GRASP_V3` (`log/analysis/final_from_v3.py`)
| | najveća razlika zgloba | širina | ruka–kocka | prsti–kocka | stol/noge |
|---|---|---|---|---|---|
| pred-hvat | **9–10°** | 0.839 m | 6.5 cm | 2.4 cm | 19.3 cm |
| hvat | **7–8°** | 0.831 m | 3.4 cm | dodir | 15.2 cm |

Prvi pokušaj (put izračunat iz skenirajuće poze) davao je razlike do **44°** — skenirajuća poza
je daleko od `GRASP_V3` u zglobovima. Ciljevi se zato računaju **polazeći od `GRASP_V3`**.

## Pokušaji
| # | Datum | Što | Rezultat | Zaključak |
|---|---|---|---|---|
| 1 | 15. 9. | Offline geometrija slijeda | izvedivo; dizanje na ≥ 1.03 m | vidi tablice gore |
| 2 | 15. 9. | Misija headless, spawn 1.05 m od kocke (run V1) | ❌ `GRASP_V3` samosudar u MoveIt-u | V3 napušten |
| 3 | 16. 9. | Korisnikove V4 poze, offline + MoveIt provjera | sve tri bez samosudara; putevi slobodni | slijed V4 |
| 4 | 16. 9. | Misija V4 headless (run V2) | ✅ `TASK COMPLETE`; `tool_tip` 1.1/2.6 mm od cilja; dodir L/R; Gazebo: kocka +15.2 cm i 0.40 m s robotom, nagib 0.3° | radi headless; čeka GUI potvrdu korisnika |
| 5 | 16. 9. | Korisnik u GUI-ju (U2) | ✅ `TASK COMPLETE`, `tool_tip` 2.2/2.0 mm; „mislim da je dobro" | prijedlozi → V3 |
| 6 | 16. 9. | Vodilice + ruke istodobno, DETECTION +5 cm, odmak 0.50 m pa vodilice 200 mm (run V3) | ✅ `TASK COMPLETE`; Gazebo: kocka na z 0.707 (u rukama, na visini vožnje) | čeka GUI korisnika |
| 7 | 16. 9. | Kraj na 100 mm; privlačenje kocke 15 cm + laktovi 20° od torza (runovi V4, V5) | V4: povlačenje preskočeno (spušteno stanje provjeravano uz stol); **V5 ✅**: privučeno 15 cm, 4.1 cm do torza, širina 0.821 m, vodilice 0.10; Gazebo z 0.605 | radi headless; bez zakreta laktova granica je 6 cm (MoveIt) |

## V4: tri poze umjesto jedne (16. 9.)
Korisnik: *„spremio sam novu verziju 4. imamo drive, detection, grasp. vozimo se u drive, ispred
stola kad lociramo kutiju idemo u detection ... kada detektiramo bočne markere, dobijemo ciljeve i
odlazimo u grasp ... pokušavao sam čisto na stisak stisnuti kutiju, ali ne ide, ispadne. tako da
dodaj da se ruke samo dođu u traženu točku ... kada dođu u točku, spajamo kutiju i dižemo ju."*

**`GRASP_V3` otpada:** MoveIt ga vidi kao samosudar (nadlaktica 1 mm u kutiji torza) na svakoj
visini vodilica — run V1 je na tome pao. Zakretanje lakta uz zamrznutu šaku nije pomoglo (8–10°:
i dalje 3 mm).

| | DRIVE_V4 | DETECTION_V4 | GRASP_V4 |
|---|---|---|---|
| vodilice / hvataljke | 200 mm / 0.791 | 400 mm / 0.791 | 400 mm / 0.791 |
| MoveIt samosudar | nema | nema | nema |
| širina | 0.821 m | 1.335 m | 0.827 m |
| prilaz šake | 53° dolje, ruke uvučene (0.27 m naprijed) | 5° dolje, kamere 0.30 m od markera, 1.8° od osi | 55° dolje, jastučići 1.4 cm od ploha |

- **Vrh alata:** `end_effector_link` je u bazi hvataljke. Središte zatvorenih jastučića je 12.7 cm
  duž osi alata → novi URDF okvir `{side}_tool_tip`. Prednji rub jastučića je 16.3 cm.
- **Slijed** (`main_task`): DRIVE_V4 do docka (0.87 m) → lociranje glavnom kamerom → vodilice
  400 mm (46.6 cm od stola) → DETECTION_V4 → primicanje na 0.63 m (≥ 11.8 cm od stola) → kamere na
  zapešću → **ciljevi = središta markera** → GRASP_V4 (MoveIt, inače interpolacija zglobova; 1.4 cm
  od kocke na kraju) → vrhovi ravno u ciljeve, **2.2 cm, bez stiska** (`touch_depth` 2 mm), zglobovi
  se mijenjaju ≤ 1.8° → dodir na obje ruke **i** `tool_tip` ≤ 1 cm od cilja → attach → +0.15 m na
  vodilicama → **unatrag 0.40 m i stop**.
- Kad jastučić dodirne plohu na visini markera, središte jastučića (`tool_tip`) je 2.6 cm izvan
  plohe (debljina jastučića pod nagibom 55°): u plohu ulazi površina jastučića, ne središte.

## Otvoreno
- Navigacija s `DRIVE_V4` kao pozom vožnje nije ponovno provjerena (Nav2 statički footprint je
  mjeren za `ARM_CARRY_V2`; dinamički footprint, [[D-19_dynamic_footprint]], prati stvarni obris).
- GUI potvrda korisnika.
