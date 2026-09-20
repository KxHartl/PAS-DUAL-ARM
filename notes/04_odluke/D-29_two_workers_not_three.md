---
id: D-29
type: decision
status: vazeca
verified: "tri-probe (3 radnika, n=3) protiv cam-par (2 radnika, n=12), 20. 9. 2026."
updated: 2026-09-20
decisions: ["[[D-24_measure_from_recordings_not_logs]]", "[[D-28_fastest_setup_that_works]]"]
problems: ["[[P-54_place_bias_not_localisation]]"]
---
# D-29: serije idu na dva radnika, ne tri

## Pitanje
`run_parallel.py` ima definirane odsječke stroja za tri radnika. Isplati li se treći?

## Izmjereno
Ista konfiguracija, isti kod (`b106a9b`), isti flagovi. Tri radnika vrtjela su jedan krug
(`tri-probe`), dva radnika dvanaest (`cam-par`).

| | 2 radnika | 3 radnika |
|---|---|---|
| uspjeh | 12/12 | 3/3 |
| **misija, simulirano vrijeme** | 175,7 s | **169,5 s** |
| misija, stvarno vrijeme | 545 s | **733 s** |
| RTF | 0,39 | **0,28** |
| promašaji roka kontrolera | 575–683 | **856–949** |
| greška odlaganja (istina) | 7,95 mm | 7,20 mm |
| **AMCL vršna greška** | **2,95 cm** | **4,60 cm** |
| AMCL zakret | 1,10° | 2,50° |

## Zaključak
**Propusnost jedva raste, a jedna mjera se pokvari.**

Treći radnik produljuje run za 36 % stvarnog vremena, pa je dobitak u 2 sata samo
26 → 29 runova (+10 %), a ne +50 %.

Važnije: **lokalizacija se mjerljivo pogoršava**. AMCL pod gladovanjem za procesorom kasni sa
skenovima i vršna greška raste za ~55 %. Odlaganje, hvat i zazor u prolazu ostaju isti, kao i
trajanje u simuliranom vremenu — robot se ponaša jednako, gubi se samo stvarno vrijeme i
pravovremenost lokalizacije.

Posljedica za mjerenje: runovi s tri radnika **nisu usporedivi** s dvoradničkim za AMCL, RTF i
stvarno trajanje, iako jesu za uspješnost i točnost odlaganja. Miješanje bi dalo različit $N$ po
veličini, što se u radu ne smije pojaviti.

## Odluka
Serije se voze na **dva radnika**. Treći odsječak (`SLICES`) ostaje definiran za slučaj kad
propusnost bude važnija od lokalizacijskih brojki, uz obveznu napomenu uz podatke.

Ako treba više propusnosti, jeftinije je osloboditi stroj nego dodati radnika: pri mjerenju su
MissionCenter, `flatpak-session-helper` i preglednik trošili ~1,5 jezgre i 4,4 GB, a nisu
prikvačeni pa kradu upravo pinovanim radnicima.
