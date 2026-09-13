---
id: R-18
type: zahtjev
status: djelomicno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-06_navigation]]", "[[S-09_task_orchestration]]"]
problems: ["[[P-12_door_too_narrow]]"]
decisions: ["[[D-08_door_widened]]"]
updated: 2026-09-13
---
# R-18: Prolaz kroz vrata BEZ kutije

## Izvor (doslovno)
> „vrata … kroz koje robot mora proći s i bez kutije.“ [MAIL]

## Tehnički znači
Prazan robot (ruke uvučene) prođe kroz otvor od ~0.8 m bez sudara sa zidom, autonomno (Nav2).

**Kriterij prihvaćanja:**
- [ ] vrata 0.8 m ([[R-11_door_80cm]])
- [ ] baza prođe iz sobe A u sobu B bez kontakta sa zidom (GUI + `/scan`)
- [ ] ruke u `ARM_CARRY` ili tuck pozi tijekom prolaza

## Trenutno stanje
⚠ Jednom provjereno **kroz 1.2 m**, headless, 23. 6. (`45c32f1`): baza je išla x 0.45 → 3.07
preko Nav2, uz staging poziciju ispred vrata. Kroz 0.8 m nikad. Trenutni `main_task` uopće ne
vozi kroz vrata.

## Kako se rješava
- [[S-06_navigation]]: staging ispred vrata (`door_xy` = (1.5, 0)), inflation (sada 0.05)
- [[S-09_task_orchestration]]: gdje u slijedu
