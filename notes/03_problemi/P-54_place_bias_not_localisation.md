---
id: P-54
type: problem
status: otvoreno
verified: "4 serije, 18. 9. 2026., mjereno protiv Gazebove istine."
updated: 2026-09-18
requirements: ["[[R-20_place_at_destination]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]"]
decisions: ["[[D-24_measure_from_recordings_not_logs]]", "[[D-18_measure_do_not_argue]]"]
---
# P-54: kocka sleti 2 cm ustranu, i lokalizacija s tim nema veze

## Simptom
`place_err_truth_mm` = **20,4–21,1 mm** u svakoj seriji. Od toga je **19,2–20,0 mm bočno**
(`place_dy`), a naprijed samo 5–8 mm. Predznak je uvijek isti.

Robot pritom tvrdi da je pogodio: njegova vlastita provjera javlja **4–7 mm**. Razlika od ~15 mm
nije u izvedbi nego u **njegovoj slici svijeta**.

## Što je isključeno — mjerenjem, ne rasuđivanjem
Kroz jednu večer su popravljene tri nezavisne stvari i **nijedna nije pomaknula ovaj broj**:

| popravak | što je dao | odlaganje |
|---|---|---|
| laserska odometrija | 1070 → 1,8 mm zanošenja | 21,0 → 20,9 mm |
| IMU u fuziji | zakret 34,6° → 0,06° | 20,9 → 20,4 mm |
| AMCL `alpha 0.2 → 0.02` | AMCL 4,1 → 2,9 cm | 20,4 → 21,1 mm |

**Greška odlaganja nije u procjeni gdje je robot.** To je sada izmjereno, a ne pretpostavljeno.

Nije ni šum: isti predznak i isti iznos kroz ~30 runova. **Sustavna je**, dakle kalibrabilna —
čim se locira.

## Gdje je smjer greške
Dok pred crvenim stolom je na kursu **−0,0°**, dakle robot gleda u +x karte. Zato je
`place_dy` (world y) upravo **robotova lijeva strana** — i to je ujedno **os stiska**, jer ruke
kocku hvataju s bočnih strana.

Greška je dakle **duž osi kojom je kocka stisnuta**. To sužava krug osumnjičenih na geometriju
hvata i percepcije, ne na navigaciju.

## Osumnjičeni
1. **Kocka se pomakne tijekom stiska.** Vrhovi alata ne stignu do svojih ciljeva jednako:
   lijevi 4,4–5,4 mm, desni 0,3–0,8 mm — **uvijek ista strana**. Što je od te asimetrije kocka
   progutala, nitko ne primijeti: `_hold` pamti središte koje su kamere vidjele **prije** stiska.
2. **Kamera vidi marker pomaknuto.** Konstantna greška ekstrinzike glave dala bi točno ovakvu
   konstantu.

`span` je isključen: kamere mjere 0,299 m naspram stvarnih 0,30.

## Prvi pokušaj popravka i zašto nije prošao
Ideja: pločice naliježu na suprotna lica, pa je točka na pola puta između vrhova alata središte
onoga što je među njima — mjerenje umjesto pretpostavke, i neovisno o dimenziji kocke.

Izmjereno uživo: ta je sredina **55 mm** od kamerinog središta. Pretpostavka je kriva — vrh
alata nije dodirna ploha nego točka ~3,8 cm **izvan** lica (log: „centre 0.299 m square, 18,8 cm
from the tool tip", a pola kocke je 15 cm), i oba vrha očito nisu jednako udaljena od svojih lica.

Zaštita od 5 cm je to odbila, zadržala kamerin broj i zapisala upozorenje, pa **nijedan run nije
pokvaren**. Ispravan put je projicirati svaki vrh na svoje lice **duž normale**, koju već imamo —
ali tek nakon što se sirovi položaji oba vrha i kamerino središte ispišu u log i pročitaju.

## Što dalje
1. ispisati sirovu geometriju hvata (oba vrha, normala, kamerino središte)
2. popraviti projekciju i ponoviti
3. `where_is_the_marker.py` — drugi osumnjičeni; alat postoji, poravnanje po vremenu mu je krivo
4. izolirani test hvat–odlaganje bez vožnje: ovaj ciklus traje 10 min po runu, a vožnja u njemu
   ne sudjeluje
