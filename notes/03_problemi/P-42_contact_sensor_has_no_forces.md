---
id: P-42
type: problem
status: otvoreno
requirements: ["[[R-17_dual_arm_lift]]", "[[R-06_realistic_parameters]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]"]
decisions: ["[[D-21_effort_pid_actuator_profile]]"]
updated: 2026-09-15
---
# P-42 — Fortress kontaktni senzor ne daje sile, samo točke dodira

## Simptom
Prvi potpuni pokus karakterizacije (`characterization_complete`, 15. 9.) zabilježio je
**49 035 kontaktnih zapisa**. Od toga:

| polje `ros_gz_interfaces/Contacts` | popunjeno |
|---|---|
| `positions` | **49 035** |
| `wrenches` (sile) | **0** |
| `normals` | **0** |
| `depths` | **0** |

Kontaktni senzor javlja **gdje** se dodir dogodio, ali ne i **kolikom silom**.

## Uzrok
**Potvrđeno mjerenjem, uzrok hipoteza.** Instalirani
`libignition-gazebo6-contact-system.so` (6.18.0) popunjava samo točke dodira; polja za
silu, normalu i dubinu penetracije ostaju prazna. Vjerojatno je riječ o ograničenju te
verzije, a ne o konfiguraciji — senzori su uredno premošteni i javljaju se čim prst dotakne
kutiju.

## Zašto ovo ruši korak 1 plana
Plan traži da se procjena sile iz momenata zglobova **usporedi s nezavisnim kontaktnim
silama**, uz prolazni kriterij „pogreška procjene stiska < 20 % cilja". `qualify_force_log.py`
je za to i napisan. Bez sile u kontaktnoj poruci **ne postoji nezavisna referenca** i taj se
kriterij ne može ocijeniti — ni pozitivno ni negativno. Skripta se ispravno zatvara u
`passed: false` umjesto da propusti nedokazanu procjenu.

Sama procjena pritom izgleda zdravo: u slobodnom gibanju 0.02–0.26 N (cilj je 5 N),
uvjetovanost poze 7.12, rezidual modela ~0.001 Nm. Ali „izgleda zdravo" nije dokaz.

## Mogući izvori nezavisne reference
| Izvor | Daje | Dostupnost |
|---|---|---|
| `ForceTorque` senzor na zapešću | apsolutnu silu i moment, nezavisno od modela ruke | `libignition-gazebo6-forcetorque-system.so` **jest instaliran** |
| težina kocke pri dizanju (0.3 kg → 2.94 N) | apsolutnu provjeru, ali tek **nakon** dizanja | ne može biti uvjet za dizanje |
| slaganje lijeve i desne ruke | da su sile jednake i suprotne u ravnoteži | odmah; provjerava dosljednost, ne iznos |

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 15. 9., radno stablo | Pun pokus karakterizacije, snimanje kontakata svih 4 jastučića | 49 035 zapisa, **0 sa silom** | Referenca za kvalifikaciju ne postoji u ovoj verziji |
| 2 | 15. 9., radno stablo | Provjera je li `ForceTorque` sustav uopće instaliran | `libignition-gazebo6-forcetorque-system.so` 6.18.0 + `libignition-sensors6-force_torque.so` prisutni | Alternativna referenca je izvediva |

## Rješenje (15. 9., odluka korisnika)
Obje reference, ne jedna:

1. **Apsolutna** — `ForceTorque` senzor na zglobu 7 svake ruke, **isključivo kao dokaz za
   ocjenu**; upravljanje ga ne smije čitati, jednako kao kontaktne senzore (stvarni Gen3 ima
   senzore momenta u zglobovima, ne FT na zapešću). Senzor mjeri cijeli prijenos kroz zapešće,
   uključujući težinu hvataljke, pa ocjena uspoređuje **promjenu** od osnovice uzete neposredno
   prije prvog dodira — tijekom stiska se ruka pomakne milimetrima, pa se težina pokrati.
   Uspoređuju se iznosi, što ne traži rotaciju između dvaju koordinatnih sustava.
2. **Dosljednost** — obje ruke pritišću istu kocku, pa im normalne sile u ravnoteži moraju biti
   jednake. To provjerava dvije kinematičke grane jednu o drugu, ali **ne može** uhvatiti
   pogrešku zajedničku objema; zato apsolutna referenca ostaje obavezna. Za to postoji i test
   (`test_an_estimate_that_disagrees_with_the_sensor_fails`).

Kontaktni senzori i dalje određuju **kada** je jastučić opterećen; FT senzor kaže **koliko**.

> [!note] FT sustav poštuje vlastiti `<topic>`, Contact sustav ne
> Kontaktni senzori objavljuju na dugoj zadanoj putanji i `bridge.yaml` ih tako mapira. FT
> sustav, suprotno tome, objavljuje točno na `/ft/left_wrist` i `/ft/right_wrist`. Prvi pokušaj
> premošćivanja gledao je u dugu putanju i tema je bila prazna.

Prvo očitanje potvrđuje da senzor radi: u mirovanju daje 13.92 N, što je težina hvataljke.

## Nuspojava: „samo 2 od 4 jastučića" je bilo pogrešno očitanje
U zapisu F5 dodir s kutijom javljaju samo `left_right` i `right_left`. To **nije** dokaz da
druga dva jastučića promašuju. Izmjereno projekcijom kolizijske mreže svakog vrha kroz njegovu
stvarnu rotaciju, u trenutku stiska:

| vrh prsta | zazor do plohe |
|---|---|
| `left_left` | +0.2 mm |
| `left_right` | +0.1 mm |
| `right_left` | −0.1 mm |
| `right_right` | +0.3 mm |

Sva četiri su na plohi unutar 0.3 mm; koji će javiti dodir odlučuje razlika od desetinke
milimetra, tj. zakret šake od ~0.0007 rad. Tihi senzori rade — `left_left` je u zapisu F4
uredno javljao dodire koji nisu kutija.

Usput provjereno i odbačeno kao uzrok: imena kolizija (sve četiri su neimenovane, pa sdformat
svima daje isto zadano ime), negativne skale mreža (`sim.launch.py` ih briše zbog PAL baze, ali
hvataljke ih uopće nemaju) i doseg mreže (`left_finger_tip.stl` i `right_finger_tip.stl` imaju
**isti** z-doseg 0.05102 m, točno `TIP_COLLISION_FORWARD`).

## Ne ponavljati
- Brojati dodire kao dokaz nosivosti. Četiri dodira ne znače nikakvu silu, a ovdje se pokazalo
  da poruka o dodiru doslovno **ne sadrži** silu.
