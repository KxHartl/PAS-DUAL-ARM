---
id: D-26
type: decision
status: vazeca
verified: "3 čista runa kalibracijske vožnje, 18. 9. 2026."
updated: 2026-09-18
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
problems: ["[[P-53_imu_declared_never_published]]", "[[P-52_no_margin_anywhere]]"]
decisions: ["[[D-25_laser_odometry_like_pal]]", "[[D-18_measure_do_not_argue]]"]
---
# D-26: IMU ulazi u fuziju, ali ne zato što je izmjeren dobitak

## Odluka
Zakret u `odom` uzima se od **žiroskopa**, pomak od **kotača**, a laser i dalje ispravlja
cjelinu na svakom prihvaćenom podudaranju. Prekidač `yaw_source:={imu,wheels}` ostaje, jer je
usporedba morala biti moguća i mora ostati ponovljiva.

**Dobitak nije izmjeren.** To je zapisano ovdje jednako glasno kao i da jest.

## Što je izmjereno
Kalibracijska vožnja (`scripts/yaw_drive.py`), 90 s, 175° ukupnog zakreta, uključuje 0,8 m
čisto bočnog gibanja. Ocjena protiv Gazebove istine, `validation/evaluate_imu_yaw.py`.

Zakret, srednja apsolutna greška kroz cijelu vožnju:

| izvor | sred. | na kraju |
|---|---|---|
| IMU (žiroskop, integriran) | **0,02–0,03°** | 0,00–0,03° |
| fuzija `/laser_odom` | 0,06° | 0,01–0,03° |
| kotači sami | 7,3–7,5° | **34,4–34,7°** |

Položaj, ista vožnja:

| izvor | sred. | na kraju |
|---|---|---|
| fuzija `/laser_odom` | **1,8–1,9 mm** | 0,1–0,6 mm |
| kotači sami | 268–293 mm | **1039–1094 mm** |

A/B po izvoru zakreta, tri čista runa:

| `yaw_source` | zakret fuzije | položaj fuzije | odbijenih podudaranja |
|---|---|---|---|
| `wheels` | 0,06° | 1,8 mm | **0** |
| `imu` | 0,06° | 1,8 mm | **0** |
| `imu` | 0,06° | 1,9 mm | **0** |

## Zašto razlike nema
Laser već ispravlja zakret kotača na svakom podudaranju, a u ovoj vožnji **nijedno podudaranje
nije odbijeno**. Dok laser radi, svejedno je kakav mu je smjer ušao u početnu pretpostavku —
ispravak ga poništi. IMU zato nije dobio priliku išta promijeniti.

To ne znači da je nepotreban, nego da ova vožnja ne mjeri ono zbog čega ga vrijedi imati.
Vrijedi ga imati za intervale u kojima laser **ne** radi: odbijeno podudaranje (`max_fitness`),
scena bez značajki, prolaz kroz vrata gdje se vidi malo zidova. Tada mrtvi račun nosi sam, i
tada je razlika između 0,03° i 7,4° cijela razlika.

Zato `imu` ostaje zadano: cijena je jedna pretplata i zbrajanje po poruci, a jedini scenarij u
kojem se izbor uopće očituje je onaj u kojem su kotači dokazano neupotrebljivi.

## Što kotači i dalje rade
Daju **pomak u vlastitom okviru između dvije poruke** i ništa više. Kroz 1/50 sekunde baza se
jedva pomakne i bočnoj se sljepoći nema u što nakupiti. Taj se korak zatim zakrene IMU-ovim
smjerom, pa greška smjera nikad ne uđe u položaj.

PAL-ov `enable_odom_tf: false` kaže da kotači ne smiju **objavljivati transformaciju**. Ne kaže
da su beskoristan ulaz, a naše mjerenje razlikuje to dvoje: greška im je u zakretu (34°), ne u
kratkom pomaku. Bez njih ostajemo bez nosača na 50 Hz, a laser stiže na 13 — za regulator koji
radi na 20 Hz to je premalo.

## Dopuna, ista večer: matrica od osam vožnji

Prekidač `translation_source: {wheels, laser}` omogućio je da se provjeri i ono što je dotad
bilo samo tvrdnja — da kotači trebaju nositi mrtvi račun jer laser stiže na 13 Hz, a regulator
radi na 20. Osam vožnji, sve četiri kombinacije, po dvije svaka, protiv istine:

| smjer | pomak | zakret sred. | zakret max | položaj sred. | položaj max |
|---|---|---|---|---|---|
| IMU | kotači | 0,060° | 1,50° | **1,85 mm** | 22,9 mm |
| IMU | laser | **0,040°** | **0,86°** | 3,55 mm | 19,6 mm |
| kotači | kotači | 0,060° | 1,65° | **1,85 mm** | 17,8 mm |
| kotači | laser | 0,060° | 1,42° | 3,75 mm | 19,9 mm |

*(kotači sami, neispravljeni, za mjerilo: 7,4° i 276 mm)*

**Razlike su sitne, i to je nalaz.** Sva četiri postava su dobra jer **laser radi gotovo sav
posao**; izbor smjera i nosača je na rubu onoga što ova vožnja razlučuje. Dva učinka prežive
oba kruga: kotači kao nosač **prepolove** srednju grešku položaja, a IMU daje **bolji vršni
zakret** u režimu u kojem između podudaranja ništa drugo ne miče pozu.

### Što ovo poništava od gornjeg teksta
Tvrdnja da je IMU popravio odometriju **ne stoji**. Popravak sa 7,4° na 0,06° je laserov.
Gornja tablica uspoređuje IMU s **neispravljenim** kotačima, što je pošteno za senzor po sebi,
ali navodi na krivi zaključak o tome što je u sustavu promijenilo ishod.

`imu` + `wheels` ostaje zadano — najbolji srednji položaj i jedini postav koji ne ovisi o tome
da svako podudaranje uspije — ali zbog **izmjerene razlike**, ne zbog priče koju sam imao.

### Redak koji nedostaje
U svakoj od osam vožnji laser ispravlja. Zato nigdje ne vidimo **koliko laser doprinosi** odvojeno
od žiroskopa. Nedostaje `IMU + kotači, bez lasera`: mrtvi račun s dobrim smjerom, nikad ispravljen.
Traži objavu `_dead` na zasebnoj temi i dvije nove vožnje.

## Što ovo poništava
Serija od 10 runova s laserskom odometrijom (`laserodom-18-09`) **nije mjerila ono što tvrdi**.
`clean_ros.sh` nije gasio `laser_odometry`, pa se čvor gomilao kroz pokušaje — izbrojano ih je
**sedam živih odjednom**, svi objavljuju na `/laser_odom`. Isti pokus s jednim čvorom daje 0,06°
i 1,8 mm; s gomilom je davao 34° i 3,1 m.

Popravljeno u `clean_ros.sh`. Seriju treba ponoviti prije ijednog zaključka o laserskoj
odometriji u misiji.
