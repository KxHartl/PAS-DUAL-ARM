---
id: P-52
type: problem
status: otvoreno
verified: "4 serije, 50 runova, 18. 9. 2026."
updated: 2026-09-18
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]", "[[R-20_place_at_destination]]", "[[R-21_deliverables]]"]
solutions: ["[[S-06_navigation]]", "[[S-08_grasp_squeeze_attach]]"]
decisions: ["[[D-24_measure_from_recordings_not_logs]]", "[[D-12_honesty_abort_over_fake]]"]
---
# P-52: sustav radi na 90–98 % svake granice koju ima

## Simptom
Misija ne pada uvijek na istom mjestu. Kroz 50 odvoženih runova padovi se sele —
poravnanje pred vratima, prolaz kroz vrata, poza ruke pri nošenju, marker za odlaganje —
i nijedan uzrok nije dominantan. Uspješnost se drži oko **65–80 %**.

## Uzrok
**Potvrđeno.** Nema jednog pokvarenog dijela. Sustav svaku svoju granicu koristi gotovo do kraja:

| veličina | tipično | granica | iskorišteno |
|---|---|---|---|
| dolazak na navigacijsku točku | 4,1–4,9 cm | 5,0 cm | **90 %** |
| zakret pri dolasku | 1,3–1,4° | 1,43° | **98 %** |
| poza ruke pri nošenju | 0,140–0,141 rad | 0,15 rad | **94 %** |
| bočni razmak u prolazu | ~6,5 cm | 0,5 cm (gate) | — |

Zato svaka sitna varijacija prebaci **neku** granicu, a koja — ovisi o runu. Pojedini pad zato
izgleda kao zaseban kvar, a nije: to je ista nestabilnost viđena s različitih strana.

Primjeri promašaja za dlaku, svi iz odvoženih serija:
- `right_joint_4 is 0.154 rad off CARRY_V4 (limit 0.15)` — promašaj **0,004 rad**
- `only 0.5 cm beside the robot ... limit 0.5 cm` — promašaj **milimetri**
- dolazak `5,5 cm` uz toleranciju `5,0 cm` — promašaj **5 mm**, pa 227 s mrtve petlje

## Ono što ispod svega stoji: bočno gibanje se ne mjeri
Izmjereno iz bagova, između pamćenja markera i završne provjere, prema ground truthu (n=13):

| | naprijed | bočno |
|---|---|---|
| odometrija kotača | **0,8 mm** (max 2,0) | **19,5 mm** (max 22,0) |

Kotači mjere ono što se **otkotrlja**. Naprijed se kotrljaju, pa je odometrija točna na pola
milimetra. Bočno ovaj pogon **kliže** — tako mecanum ovdje uopće radi, `mu2 = 0,20` je namjeran
([[P-09_omni_drive_on_fortress]]) — a klizanje kotačima ne ostavlja zakret za brojanje.

Posljedice te jedne činjenice, sve izmjerene:
- **odlaganje promaši ~20 mm** (median `place_err_truth_mm` 20,9 mm, n=10)
- **robot to ne vidi**: tvrdi 3–7 mm, razlika **+15,4 mm** sa sd **0,2** (n=13) — ista referenca
  kojom se giba ujedno mu je i mjerilo
- **u vrata uđe do 6 cm od osi**, pa gate prolaza odbije run

Trenje kotača je provjereno i **ispravno**: `fdir1` daje X-raspored (`FR`/`RL` +45°, `FL`/`RR` −45°),
dijagonalni parovi se poklapaju. Klizanje nije posljedica krivog postava nego same metode.

## Pokušaji
| # | datum | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 18. 9. | `required_movement_radius` 0.05 → 0.02 (bio jednak toleranciji cilja) | ⚠ djelomično | pravilo iz komentara je vraćeno, ali dionica duguje **zaokret**, ne put |
| 2 | 18. 9. | `SimpleProgressChecker` → `PoseProgressChecker` (+`required_movement_angle`) | ⚠ 4/20 → 1/10 | zaokret se sad broji kao napredak; robot se prestao ledit (0 % → 9,6 % naredbi ≠ 0) |
| 3 | 18. 9. | hibridna provjera odlaganja: naprijed iz odometrije, bočno iz karte | ❌ **bez učinka** | pristranost ostala +15,4 mm. **Mjerenje je bilo krivo**, vidi niže |
| 4 | 18. 9. | `GoalAlign` u zasebnom profilu `FollowPathAlign`, po dionici preko `behavior_tree` | ✅ **zastoj nestao** (0/20 u dvije serije), run **13 % brži** | cijena: zazor u vratima −1,2 cm |
| 5 | 18. 9. | zamah glavom (3 nagiba) prije odustajanja od markera | ⚠ i dalje ~1/10 pada | nije samo kut nagiba |
| 6 | 18. 9. | korak fizike 1 ms → 2 ms (radi brzine) | ❌ **vraćeno** | ruka pod teretom s 0,141 prešla na **0,150 rad** = granica; RTF 0,494 → 0,614 nije vrijedan toga |

## Četiri serije
| serija | uspješnost | zastoji `no-progress` | `door_clear_min_m` | trajanje |
|---|---|---|---|---|
| izvorna (n=20) | 65 % | **4** | 0,475 | ~640 s |
| popravci 1–2 (n=10) | 70 % | 1 | 0,479 | ~646 s |
| align zaprljan (n=10) | 80 % | **0** | 0,450 | 607–626 s |
| align čist (n=10) | 70 % | **0** | 0,463 | **551–579 s** |

> [!warning] Uspješnost se NE smije navoditi kao poboljšana
> 65 / 70 / 80 / 70 % na uzorcima od 10–20 je šum. Ono što **jest** dokazano je nestanak zastoja
> (4 → 0 kroz 20 uzastopnih runova) i **13 % kraći run**.

## Dvije moje greške, obje vrijedne zapisa
**1. Rijetko uzorkovana tema pročitana kao stabilnost.** Zaključio sam da karta bočno drži 1,3 mm
gdje odometrija luta 19,5. `/amcl_pose` u tom prozoru izlazi **0,08 Hz** (12 poruka u 152 s), jer
se AMCL osvježava na gibanje a robot pri odlaganju stoji. „Najbliža poruka" u dva trenutka bile su
dvije poze stare desecima sekundi, i ta zastarjelost je izgledala kao mirnoća. Uz zamrznut
`map → odom`, TF `map → base_link` je samo živi `odom → base_link` s konstantom ispred — luta
jednako. Hibrid je zato promijenio 2 mm umjesto 15, i vraćen je.

**2. Pokus s tri promjene odjednom.** Prvi `FollowPathAlign` je uz `GoalAlign` nosio i `min_vel_x`
−0,26 (misija ima −0,30) i `vx_samples` 25 (misija 15), prepisano rukom. Serija je izmjerila vrata
2,7 cm tješnja i zaključio sam da to košta `GoalAlign`. Nakon ispravka: **1,2 cm** je stvarna cijena,
1,4 cm je bila moja nepažnja.

## Sljedeći korak
Granice se **ne smiju** popuštati da run prođe ([[AGENT_GUIDE]] §5). Ono što se smije je dati
robotu referencu koja bočno gibanje **vidi**. AMCL je već ima — lidar — ali se osvježava tek nakon
`update_min_d` (3 cm) prijeđenog puta, pa pri finom prilazu i odlaganju praktički spava.
To je prvo mjesto koje treba izmjeriti prije bilo kakve izmjene.
