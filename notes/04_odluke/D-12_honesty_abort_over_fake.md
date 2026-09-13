---
id: D-12
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-16_find_box]]", "[[R-17_dual_arm_lift]]", "[[R-06_realistic_parameters]]"]
problems: ["[[P-16_fake_teleport_grasp]]", "[[P-28_gate_too_strict]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-12: Politika poštenja: abort je bolji od lažnog uspjeha

## Kontekst
Kroz lipanj „hvat“ je izgledao uspješno, a bio je lažan:
- teleport `set_pose`;
- cirkularna provjera (vrhovi prstiju mjereni prema istom naređenom centru);
- tihi fallback na x = 0.55 kad mjerenje padne;
- zatvaranje hvataljki ~50 cm ispred kutije + attach iz daljine.

## Odluka (30. 6., plan „ne-radi-hvatanje-kutije“; ojačano 16. 7.)
1. **Nikad attach bez neovisnog fizičkog dokaza** (box-only kontakt, imena kolizija iz poruke).
2. **Nijedan tihi fallback na izmišljenu pozu.** Ako percepcija padne, slijedi `_fail`.
3. Provjere moraju biti **nezavisne** od naredbe (TF readback, senzori), nikad naredba vs naredba.
4. Neuspjeh završava povlačenjem ruku + jasnom porukom `Task aborted during: …`.
5. Gate se smije popustiti samo **pošteno**: primarni dokaz ostaje fizički kontakt.

## Posljedice
Niža prividna uspješnost (~35 %), ali svaki uspjeh je stvaran. Svih ~15 neuspjeha su pošteni
aborti.

## Za agente
Ovo je **invarijanta**: vidi [[AGENT_GUIDE]]. Svaka izmjena gatea mora zadržati točke 1–3.
