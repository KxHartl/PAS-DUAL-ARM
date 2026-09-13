---
id: P-36
type: problem
status: rijeseno
requirements: ["[[R-10_mappable_world]]", "[[R-16_find_box]]", "[[R-15_region_goal_nav2]]"]
solutions: ["[[S-02_world_and_sim_launch]]"]
decisions: ["[[D-13_three_room_world]]"]
updated: 2026-09-13
---
# P-36: Zidovi (1.2 m) bili su niži od kamere (1.45 m)

## Simptom
Kamera na pan-tiltu je na **z = 1.39 m** (`camera_color_frame`, s rubom 1.45 m), a zidovi soba bili
su **1.2 m** visoki. Robot je iz sive sobe gledao **preko** zidova u plavu i crvenu sobu.

## Uzrok
**Potvrđeno mjerenjem** (`scripts/measure_robot.py`, 13. 9.): najviša točka robota je 1.45 m, dok
su zidovi u `seminar_world.sdf` bili postavljeni na 1.2 m.

Obrazloženje zapisano u [[D-13_three_room_world]] — „zidovi 1.2 m, pa kamera iz HOME sobe ne vidi
kutiju preko zida“ — **bilo je netočno**: visina je odabrana bez provjere visine kamere.

## Zašto je bilo važno
Misija iz [MAIL] ima smisla samo ako se kutija **ne vidi** s početne poze. Da se marker vidio preko
zida, `main_task` bi mogao odmah krenuti u vizualni servo i **preskočiti** mapiranje i Nav2
([[R-14_slam_mapping]], [[R-15_region_goal_nav2]]) — dakle upravo ono što [MAIL] traži da postoji.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 13. 9. `2bda09e` | zidovi 1.2 m uz pretpostavku da blokiraju pogled | — | pretpostavka nije bila provjerena |
| 2 | 13. 9. | izmjerena visina robota u korisnikovoj pozi | kamera 1.39–1.45 m, **iznad zidova** | pretpostavka pala |
| 3 | 13. 9. | **zidovi podignuti na 3.0 m** (odluka korisnika) | SDF valjan (`ign sdf -k`) | kamera je 1.6 m ispod vrha zida → pogled preko zida nije moguć |

## Trenutno rješenje
Zidovi 3.0 m (`seminar_world.sdf`, model `rooms`: pozicija z = 1.5, visina 3.0). Otvori za vrata su
pune visine (0.9 m široki prorez od poda do 3 m) — nema nadvoja.

## Otvoreno (kozmetika)
Ako se želi „prava“ vrata, dodati nadvoj iznad otvora (npr. od 2.1 m do 3.0 m). Robot je visok
1.45 m, pa nadvoj ne smeta prolazu, a scena izgleda urednije na videu.
