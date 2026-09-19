---
id: P-55
type: problem
status: rijeseno
verified: "37 prolaza kroz vrata iz 2 serije, protiv Gazebove istine, 19. 9. 2026."
updated: 2026-09-19
requirements: ["[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
solutions: ["[[S-06_navigation]]"]
decisions: ["[[D-18_measure_do_not_argue]]", "[[D-24_measure_from_recordings_not_logs]]"]
---
# P-55: robot uđe u vrata ravno, a prođe ukoso

## Simptom
Nakon što su odometrija i AMCL popravljeni (vidi [[D-26_imu_yaw_in_the_fusion]] i podešene alfe),
padovi na vratima se i dalje događaju — otprilike 1 od 3 do 1 od 5 runova:

```
only 0.1 cm beside the robot (left 0.428 m, right 0.567 m), limit 0.5 cm
```

## Što NIJE uzrok — oboje isključeno mjerenjem
**1. Bočni položaj.** U runu koji je pao robot je bio **2,1 cm** od osi otvora. To je unutar svega;
uz širinu od 83 cm i otvor od 99,5 cm trebalo mu je ostati ~6 cm sa svake strane.

**2. Lidar kao bolji izvor od AMCL-a.** `validation/doorway_offset.py` uspoređuje oba s istinom
**baš u vratima**, 37 prolaza (`P-52` je to mjerio kroz cijele vožnje, što je druga stvar):

| | medijan | najgore | bliži istini |
|---|---|---|---|
| lidar | 1,25 cm | 5,14 cm | 17/37 |
| AMCL | **1,08 cm** | **5,00 cm** | 20/37 |

Izjednačeni. A u runu koji je pao: **lidar +7,0 cm, AMCL +1,2 cm, istina +2,1 cm** — lidar je
promašio za 4,9 cm. Bočna korekcija po lidaru odgurala bi robota u **krivu** stranu.

## Uzrok
**Zakret, i nastaje tijekom prolaza.** Ista mjerenja, 37 prolaza:

| | medijan | najgore |
|---|---|---|
| \|zakret\| od osi vrata | 1,32° | **6,05°** |
| zazor po strani | 7,14 cm | **2,61 cm** |

Najveći zakret i najmanji zazor su **isti prolaz** — onaj koji je pao. Zakret od 6° robota
širokog 83 cm i dugog 72 cm kroz otvor provlači efektivnom širinom
`83·cos θ + 72·sin θ` = **90,1 cm**, što od 99,5 cm ostavlja 2,4 cm po strani.

A gate poravnanja je neposredno prije ulaska javio:

```
alignment: offset 0.008 m, heading +1.3 deg, needs 0.877 m of doorway
```

**Ulazi ravno. Krivi se u prolazu.** Gate mjeri jednom, prije ulaska, i nakon toga kroz 2,3 m
nitko ne gleda kurs.

## Kako nastaje, iz istine i `/cmd_vel` u istom bagu
Zakret raste **u koracima, i to na Nav2-ovu vlastitu naredbu**:

| t | kurs | `wz` |
|---|---|---|
| 0–2,5 s | 3,7° → 1,1° (poravnavanje u mjestu) | −0,07 → −0,03 |
| 3,0 s | 1,1° | **+0,10** |
| 4,0 s | **3,65°** | 0 |
| 4–8,5 s | 3,65° drži | 0 |
| 8,5 s | 3,65° | **+0,10** |
| 9,0 s | **4,92°** | 0 |
| 9,5 s | 4,92° | **+0,10** |
| 10,0 s | **6,05°** | 0 |

Tri naleta `wz = +0,10 rad/s`, svaki doda 1,2–2,5°, svi u istu stranu. Između njih kurs miruje.
**Nav2 sam zakreće robota u prolazu.** Bočna brzina `vy` je pritom −0,01 do −0,03 — praktički
nekorištena, iako baza to može.

## Zašto Nav2 to radi
Profil za prolaze (`FollowPath`, `nav2_params.yaml`) ima kritičare:

```
["RotateToGoal", "Oscillation", "ObstacleFootprint", "PathDist", "GoalDist", "Twirling"]
PathDist 96,  GoalDist 24,  RotateToGoal 64,  Twirling 2
```

`PathDist` i `GoalDist` ocjenjuju **položaj**. `RotateToGoal` po Nav2 dokumentaciji vraća 0
izvan `xy_goal_tolerance`, dakle u prolazu ne djeluje. **`PathAlign` nema u popisu.**

Znači da kurs u prolazu **ne drži nitko** — jedina kazna za zakretanje je `Twirling` sa
skalom 2 naspram 96 za položaj. Rotacija je slobodna, a za mecanum bazu u vratima je najgori
mogući način ispravljanja bočne greške: `vy` postoji (`vy_samples: 25`, ±0,15 m/s) i ne koristi se.

## Što je pokušano

### `PathAlign` u kritičare — ODBIJENO, pogoršava
Očit odgovor: jedini kritičar koji ocjenjuje kurs prema smjeru putanje, Nav2-ov default 32.
Tri runa: **jedan prošao, dva ISTEKLA** u vratima nakon 240 s, naspram 2 od 3 prije toga.
Robot se nije ni ogrebao ni zaglavio o dovratnik — DWB naprosto nije našao putanju koja mu
odgovara dok poravnanje vuče protiv `PathDist` od 96. Maknuto; razlog i rezultat zapisani u
samom `nav2_params.yaml`, jer je ideja previše očita da je netko ne bi ponovio.

### `Twirling` veći — NIJE ni pokušano, namjerno
Bio bi kraći put i krivi. Baza je predviđena da kasnije vozi blizu svojih ograničenja,
**istovremeno se okrećući i translatirajući**, kako ne bi morala stajati da se okrene prema
svakoj točki. Velika cijena rotacije to onemogućuje. `PathAlign` bi vezao kurs uz putanju;
`Twirling` kažnjava okretanje kao takvo.

### Nadzor kursa kroz cijeli prolaz — RADI
`room_navigator` gleda kurs cijelim prolazom, ne samo prije ulaska. Preko **0,05 rad (2,9°)**
otkazuje Nav2 goal, okrene robota u mjestu dok nije ravan, i vozi dionicu ponovno. Jednom po
dionici, pa robot koji ne može držati kurs padne umjesto da kruži.

Rezultat, 3 runa i 8 prolaza (`kurs2-19-09`) protiv 37 prolaza polazišta:

| | polazište | sa zaštitom |
|---|---|---|
| zakret, medijan | 1,32° | 1,30° |
| **zakret, najgori** | **6,05°** | **1,44°** |
| zazor, medijan | 7,14 cm | 7,20 cm |
| **zazor, najmanji** | **2,61 cm** | **5,30 cm** |
| uspjeh serije | 2/3 | **3/3** |

Medijan se nije pomaknuo i nije trebao: zaštita ne popravlja tipičan prolaz nego **odsijeca
rep**, a rep je bio uzrok padova. Okinula se 4 puta kroz 3 runa — zanošenje i dalje nastaje,
samo više ne naraste.

## Što ostaje
Uzrok zanošenja **nije uklonjen**, samo presretnut. Nav2 i dalje zakreće robota u prolazu jer
mu nijedan kritičar to ne brani, a jedini kritičar koji bi to radio ruši prolaznost. Prava bi
meta bio lokalni planer koji za mecanum bazu bočnu grešku ispravlja `vy`-jem umjesto rotacijom
— `vy_samples: 25` i ±0,15 m/s stoje neiskorišteni.
