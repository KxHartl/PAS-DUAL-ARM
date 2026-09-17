---
id: P-28
type: problem
status: rijeseno
verified: "gate prolazi u M4 bez popustanja fizickog dokaza"
requirements: ["[[R-17_dual_arm_lift]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]"]
decisions: ["[[D-12_honesty_abort_over_fake]]"]
updated: 2026-09-17
---
# P-28: Gate hvata je prestrog (odbija stvarne hvatove)

## Simptom
Runovi 29 i 30 imali su **stvaran obostrani kontakt s kutijom**, a abortirali su na geometriji vrhova
prstiju: ruke su bile 6–13 cm od idealne poze zbog IK/tracking lutrije. Uspješnost s kockom je ~35 %
(3/9).

## Uzrok
**Potvrđeno:** gate je tražio strogu geometriju (`fingertips_on_box`) **I** kontakt. Geometrija
ovisi o tracking grešci koja je u simu 4–13 cm.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. | gate = placement (reached + geom) I (senzor ILI stall) | — | stall izbačen: ravan press nema stalla |
| 2 | 16. 7. | placement (reached ili pad kontakt + geom) I box-only kontakt L + D | 3/9 uspjeha, aborti pošteni | prestrogo |
| 3 | 16. 7. | **primarno:** svjež box-only kontakt na OBA jastučića; placement = geom **ILI** oba EE < 0.12 m od press ciljeva | 🧪 **nije pokrenuto** (runovi 29/30 pali prije; 31 prekinut) | očekivano blizu 100 % |

## Sljedeći korak
1. Runovi 31+ u GUI-ju (3–5 runova) → upisati ishode u [[runovi]].
2. Ako 5c nudge i dalje ruši: dodati settle + recheck petlju u 5c.
3. **Ne** popuštati fizički dokaz ([[D-12_honesty_abort_over_fake]]).

**Kriterij uspjeha:** ≥ 3/5 runova do attacha, nula lažnih attacha.

## Napomena
Promašeni press zna **zbaciti kocku sa stola** pri povlačenju (run 29: kocka na podu metar dalje).
Nakon takvog aborta treba restartati sim.
