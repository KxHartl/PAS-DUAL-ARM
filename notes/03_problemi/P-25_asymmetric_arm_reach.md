---
id: P-25
type: problem
status: rijeseno
requirements: ["[[R-17_dual_arm_lift]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]", "[[S-09_task_orchestration]]"]
decisions: []
updated: 2026-09-15
---
# P-25: Zone dosega lijeve i desne ruke se ne preklapaju

## Simptom
S kockom na y ≈ -0.13 desna ruka je savršena, a lijeva kronično ne može ravnu liniju (preko
središta torza). S kockom na y ≈ -0.02 je obrnuto.

## Uzrok
**Potvrđeno empirijski:** asimetrična montaža (desni klizač rotiran 180°, klinovi pod 45°) daje
različite radne prostore za press.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 15. 7. | kocka gdje je prilaz ostavi (y ≈ -0.13) | lijeva pada | — |
| 2 | 16. 7. | centrirati kocku na y = 0 | desna pada | — |
| 3 | 16. 7. | **centrirajući okret na y ≈ -0.075** (sredina), kut iz odometrije | obje ruke izvedive | rješenje |
| — | 15. 9., radno stablo | Traženo IK rješenje za pomake 0.5 / 5 / 10 mm od **trenutne** poze svake ruke, pa izmjerena i točnost rješenja (kroz `/compute_fk`) i koliko se zglobovi moraju pomaknuti | **IK je točan: pogreška poze 0.00 mm za sve slučajeve, obje ruke.** Ali za pomak od **0.5 mm** lijeva ruka vrati rješenje **3.04 rad** od trenutne konfiguracije (preskok grane), a desna **0.0036 rad**. Isto vrijedi za 5 i 10 mm: lijeva 2.84–3.04 rad, desna 0.016–0.037 rad | Asimetrija **nije** u dosegu ni u točnosti IK-a nego u **izboru grane**. Zbog toga stisak silom povremeno pukne na `left IK branch/tracking jump`: zaštita ispravno odbije izvesti zamah od 3 rad za pomak od pola milimetra. Sjeme (`seed`) iz trenutnog stanja to ne spriječi |

## Izmjereno 15. 9.: lijeva ruka je sustavno lošija, desna je izvrsna
Preko svih današnjih runova (headless i GUI), odstupanje izmjerene poze od naređene:

| mjera | lijeva | desna |
|---|---|---|
| pred-hvat, median | **15.0 mm** | 10.0 mm |
| pred-hvat, preko praga 20 mm | **4 od 17** | **0 od 13** |
| standoff, raspon | 20.0 – **56.9** mm | 20.2 – **20.9** mm |
| standoff izvan 20 ± 5 mm | 1 od 10 | 0 od 9 |

Desna ruka drži standoff u rasponu od **0.7 mm** kroz devet runova — dakle cjevovod (percepcija,
planiranje, kontrola) je sposoban biti tijesan. Lijeva povremeno promaši grubo.

Isti obrazac vidi se i drugdje: lijeva ruka titra **53.7 N** na zapešću tijekom pred-hvata naspram
**21.8 N** desne ([[P-41_effort_pid_arm_actuator_profile]]), lijeva je preskakala IK granu za pomak
od 0.5 mm dok desna nije, i lijeva je bila ta koja nije bila smirena pri nuliranju.

**Hipoteza (nije potvrđena):** u ovoj konfiguraciji lijeva ruka radi blizu granice grane —
`left_joint_3` i `left_joint_5` stoje oko ±3.14 rad, dakle na samoj točki 2π omota kontinuiranih
zglobova. Treba provjeriti mijenja li se ponašanje ako se pred-hvat za lijevu ruku bira tako da
zglobovi ne leže na ±π.

> [!warning] Ne popuštati prag pred-hvata dok se ovo ne razjasni
> Prag od 20 mm pada u sredinu raspodjele lijeve ruke, pa bi ga bilo lako „popraviti"
> podizanjem. To bi sakrilo stvarni kvar. Desna ruka pokazuje da 10 mm i standoff unutar
> 0.7 mm jesu dostižni.

## Rješenje za korak stiska (15. 9.)
Korak stiska **više ne zove IK**. Za pomak od 0.5 mm koristi se diferencijalni korak preko
Jacobiana (`force_model.resolved_rate`): prigušeni najmanji kvadrati na `J Δq = Δx`, što je
lokalno po konstrukciji pa promjena grane nije moguća.

Zašto IK nije bio dobar alat: MoveIt-ov solver je **globalan**. Kad mu seedani Newton ne
konvergira, restarta iz slučajnog sjemena i vrati drugu konfiguraciju — izmjereno je da je
pogreška poze **0.00 mm** (rješenje je ispravno!), ali 3.04 rad od trenutnog stanja. Zaštita
`IK branch/tracking jump` ga je ispravno odbijala, pa je run padao na prvom koraku stiska.

Prigušenje je odabrano mjerenjem, ne napamet. Preko 200 slučajnih Jacobiana, najgori slučaj
izgubljenog dijela traženog pomaka:

| prigušenje | izgubljeno | max \|Δq\| |
|---|---|---|
| 0.002 | 1.0 % | 0.0089 rad |
| **0.005** | **5.6 %** | **0.0086 rad** |
| 0.01 | 16.7 % | 0.0079 rad |
| 0.05 | 54.0 % | 0.0047 rad |

Uzeto je 0.005. Veličinu koraka ionako određuje koliko je zahtjev malen, a ne prigušenje.
Provjera sudara ostaje — `GetStateValidity` za obje ruke, na polovini i na kraju koraka.

## Trenutno rješenje
`main_task.py:1525` (`center.y + 0.075`). Vrijednost je u [[06_parametri]].
