---
id: D-18
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
problems: ["[[P-39_nav2_enters_doorway_at_an_angle]]", "[[P-37_arm_position_gain_sag]]"]
supersedes: "[[D-17_closed_loop_door_transit]]"
superseded_by: ""
updated: 2026-09-14
---
# D-18 — Provjereno stanje je polazište, a inkrement ima svoj run

## Kontekst
14. 9. je navigacija radila end-to-end: ručni RViz cilj → plava soba za **33.1 s** (Vrata 0
prijeđena s 1.8 cm i **4.2°**), i `goto red` → pred crvenim stolom za **60.6 s**. Nav2 je
vozio **svaku** dionicu; kod vrata su postojale samo portalne poze i preduvjet
`arms_ok()` + `aligned_with()`.

Taj run nije bio commitan. Na njega su istog dana naslagana dva sloja, **nijedan odvožen**:
zatvorena petlja kroz vrata s vlastitim gateom ([[D-17_closed_loop_door_transit]]), pa
mjerenje širine koje po defaultu odbija vožnju. Rezultat: sustav je prestao voziti, a
korisnik je stanje opisao kao „najgore ikad".

Mjerljivo: novi gate `doorway_margin` za prolaz od 4.2° daje `0.95 − 0.925 − 0.05 = −2.5 cm`,
dakle **odbija prolaz koji je dokazano uspio**. Stari gate `aligned_with()` ostao je u kodu
neprozvan.

## Odluka
1. `main` se vraća na stanje provjerenog runa. Kod povučenih slojeva čuva se na grani
   `wip/door-transit-closed-loop` i ne briše se.
2. **Arhitektura ostaje ona koja je radila** (i koju je korisnik tražio): sustav se vozi
   normalno preko Nav2; kod **značajke** se na dionicu prikvači posebno ponašanje — kod
   vrata portalne poze i preduvjet, kod stola halo i prilazna poza.
3. **Ništa što može odbiti vožnju ne ulazi u kod bez runa koji dokazuje da je to odbijanje
   potrebno.** Novi gate koji je stroži od postojećeg mora najprije pokazati da postojeći
   propušta nešto što se stvarno dogodilo.
4. Jedan inkrement = jedna izmjena + jedan run + jedan red u [[runovi]] i u pripadnoj
   P-kartici. Ne slaže se drugi sloj dok prvi nije odvožen.
5. Stanje koje je odvoženo **se commita istog dana**. Run A je izgubljen kao referenca upravo
   zato što je postojao samo kao tekst u `STATE.md`.

## Posljedice
- Povučeni su: ekskluzivno poravnanje, zatvoreni tranzit, `doorway_margin` kao gate, prekid
  kod ploče stola, `envelope_monitor` i `require_envelope`, mjerenje iz dovratnika, kutni
  `scan_filter`. Nijedan nije bio loša ideja sam po sebi — problem je što ih je pet stiglo
  odjednom, bez ijednog metra vožnje između.
- Nalazi izračunati pri njihovoj analizi **ostaju vrijediti** i zapisani su u
  [[P-39_nav2_enters_doorway_at_an_angle]] („Nalazi koji vrijede neovisno o sloju").
- Sljedeći inkrement je **obilazak stola**, jer je to problem koji je korisnik opisao;
  mjerenje širine i kutni filtar dolaze poslije, i to mjerenje **bez** prava da zaustavi.

## Odnos prema zahtjevima
Ne mijenja nijedan zahtjev. [MAIL] traži Nav2 do regije — Nav2 i dalje vozi. Ovo je odluka o
**načinu rada**, ne o rješenju: kako se dodaje ponašanje, a ne koje ponašanje.
