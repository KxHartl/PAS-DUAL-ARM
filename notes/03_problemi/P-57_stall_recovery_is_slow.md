---
id: P-57
type: problem
status: otvoreno
verified: "n40/w1 run 009, 20. 9. 2026., jedini slučaj u 40 runova."
updated: 2026-09-20
requirements: ["[[R-19_door_pass_with_box]]", "[[R-20_place_at_destination]]"]
problems: ["[[P-55_doorway_skew_is_control]]"]
decisions: ["[[D-28_fastest_setup_that_works]]"]
---
# P-57: zaštita od zapinjanja radi, ali oporavak traje 5,5 minuta

## Što se dogodilo
U seriji `n40` (28 runova) jedan je run trajao **915 s** umjesto uobičajenih ~550. Misija je
**uspjela**, 5 mm od markera.

Sve dionice osim jedne su normalne. Noga 1/6 — *odmicanje od stola prema pozi prilaza, s kockom
u rukama* — potrošila je **385 s** umjesto ~30.

```
[room_navigator] leg 1/6 - blue: back off the table to approach pose ...:
    nothing has moved for 25 s, 5.4 cm and -25.9 deg from the last pose;
    asking Nav2 for the route again
[bt_navigator] Failed to get result for follow_path in node halt!
```

## Dobra vijest
**Zaštita je prvi put viđena kako djeluje.** Napisana je 19. 9. i triput popravljana (mjerena u
`map` umjesto u `odom`, krivi sat, kriva metoda), ali do sada nijedan run nije zapeo pa se nije
imala prilike dokazati. Sada jest: prepoznala je zastoj i zatražila novu rutu, i time spasila run
koji bi inače istekao.

## Loša vijest
Od okidanja do dolaska prošlo je još **329 s**. Dakle prag od 25 s nije problem — problem je što
nakon ponovnog traženja rute vožnja i dalje dugo ne napreduje.

Zakret od **−25,9°** od zadnje poze govori da se robot u trenutku zastoja vrtio u mjestu, a ne da
je stajao. To je dionica na kojoj je zakret u mjestu inače zabranjen (prilaz stolu), pa je
sumnja na prijelaz iz te zabrane u slobodnu vožnju.

## Što nije napravljeno i zašto
Ništa. Nalaz je pao **usred serije od 40 runova za predaju**; izmjena ponašanja u tom trenutku
poništila bi uzorak. Zapisano je i ostavljeno.

## Što dalje
1. iz snimke tog runa izvući putanju i `cmd_vel` u prozoru 1789931666–1789932051 i vidjeti je li
   robot davao naredbe koje ne izvršava (proklizavanje, [[P-56_speed_is_capped_by_slip]]) ili
   nije davao ništa
2. provjeriti prelazi li zabrana zakreta u mjestu ispravno na sljedeću dionicu
3. ako se ponovi: nakon drugog uzastopnog zastoja ne tražiti opet istu rutu, nego se odmaknuti
   unatrag pa planirati iznova

## Učestalost
**1 od 40** izvođenja (2,5 %). U seminaru je naveden kao objašnjenje gornjeg kraja raspona
trajanja misije (\MisijaMax), a ne prešućen.
