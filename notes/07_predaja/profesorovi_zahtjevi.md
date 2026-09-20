---
id: PREDAJA-ZAHTJEVI
type: plan
updated: 2026-09-21
---
# Profesorovi zahtjevi: što je riješeno i čime

Popis svih smjernica koje je profesor dao na objavljeni repozitorij i na seminar, sa stanjem na
dan predaje. Uz svaku stavku stoji **čime se dokazuje**, a ne samo tvrdnja da je gotova.

## A. Komentari na seminar (doslovno, [[danas]])

### A1. „provjeriti tvrdnju o udaljenosti 0,835 m od prepreka jer iz trenutnog opisa nije jasno na što se točno odnosi" ✅

**Što je bio problem.** Brojka je bila točna, ali nerečena do kraja: 0,835 m je razmak koji
**planer izračuna** na spremljenoj karti, izveden bez pokretanja simulacije. Koliko se robot
stvarno drži od stola **u vožnji** nije bilo izmjereno, pa se iz opisa nije vidjelo na koju se od
te dvije veličine brojka odnosi.

**Kako je riješeno.** `validation/analyze_runs.py` mjeri razmak iz snimke, protiv Gazebovih
vlastitih poza, i pritom **razdvaja prelazak sobe od prilaza stolu** — robot se stolu mora
primaknuti da bi kutiju dohvatio, pa bi zajednički minimum bio besmislen:

```
The report states 0.835 m from the table edge in open room space.
Driven, crossing a room: min 0.703 m, median 0.705 m, over 40 run(s).
The approach to a table is excluded and reported separately.
```

U radu sada stoje **obje** veličine i razlog zašto se razlikuju: planirano 0,835 m, izvezeno
0,704 m (raspon 0,698–0,707 m kroz 40 izvođenja). Poglavlje „Planiranje puta jedinstvenim poljem
cijene".

### A2. „rezultati bi bili uvjerljiviji kada bi se prikazalo nekoliko ponavljanja kompletne završne misije i osnovna statistika uspješnosti" ✅

**Serija od 40 cjelovitih izvođenja**, svako u vlastitoj simulaciji i svako u malo drukčijem
svijetu (kutija pomaknuta za nekoliko centimetara, da se ne mjeri ista stvar 40 puta).

Uzorak je **jedinstven**: 12 (`cam-par`) + 28 (`n40`), isti kod, isti postav, dva paralelna
radnika u oba slučaja, RTF 0,39 u oba — pa sve veličine imaju isti $N$. Trokradnička proba
(`tri-probe`) namjerno je **izostavljena** iz uzorka jer gušenje kvari lokalizaciju
(→ [[D-29_two_workers_not_three]]).

| | medijan | raspon |
|---|---|---|
| dovršene misije | **40 / 40** | — |
| prolazaka kroz vrata | **80 / 80** | — |
| odstupanje odlaganja | 8,0 mm | 6,2–9,3 |
| pogreška lokalizacije (vršna) | 3,1 cm | 2,3–5,1 |
| bočni zazor u prolazu | 4,8 cm | 3,6–6,1 |
| razmak od stola u vožnji | 0,704 m | 0,698–0,707 |
| trajanje misije | 173,0 s | 165,4–312,3 |

**Brojke nisu prepisane rukom.** `validation/seminar_numbers.py` čita snimke i piše
`seminar/mjerenja.tex` kao LaTeX makroe, koje rad uvlači. Osvježavanje nakon nove serije je jedna
naredba, pa se tekst i podaci ne mogu razići.

**Mjereno protiv istine, ne protiv vlastite tvrdnje.** Položaji robota i kutije uzimaju se iz
simulatora. Zato je u radu navedeno i to da robot **sam** javlja 4,0 mm ondje gdje neovisno
mjerenje daje 8,0 mm — ta razlika **jest** iznos pogreške njegove procjene.

### A3. „ublažiti izraze poput geometrijski egzaktna karta, milimetarska točnost i determinističko i robusno izvršavanje ako nisu potkrijepljeni dovoljnim brojem ponavljanja" ✅

Svi navedeni izrazi uklonjeni su iz teksta (provjereno: 0 pojavljivanja).

Umjesto blažeg izražavanja uvedena je **brojka koja govori točno koliko uzorak nosi**. Uz nula
neuspjeha u $N$ pokušaja, donja granica stvarne stope uspješnosti uz 95 % povjerenja iznosi
$1 - 3/N$:

| $N$ | smije se tvrditi |
|---|---|
| 12 | ≥ 75 % |
| **40** | **≥ 92,5 %** |

Rad izriče **92,5 %**, a ne „uvijek radi", i izrijekom kaže da bi tvrdnja o potpunoj pouzdanosti
tražila uzorak za red veličine veći.

### A4. „smanjiti količinu detalja i opisa pojedinih pokretanja, fokusirati tekst na konačno rješenje, algoritam i rezultate" ✅

Uklonjeno:

- **tablica redaka zapisa iz jednog izvođenja** s prijevodima (31 redak) — najizravniji primjer
  onoga na što se primjedba odnosila; uz 40 mjerenih izvođenja nije govorila ništa što statistika
  ne kaže. Zamijenjena s 11 redaka koji definiraju kriterij uspjeha;
- **anegdota o prvom pokretanju ispitnog sklopa** — bila je o alatu, ne o robotu;
- **trostruko ponavljanje kriterija uspjeha** u tri pododjeljka → sada stoji jednom, u „Načinu
  ispitivanja";
- **slika 4.2** (nacrt torza iz zadatka) — tuđa slika, a dimenzije koje je nosila već su u tablici;
- **slika 6.1** (nečitka snimka mapiranja) → zamijenjena prikazom sustava u radu.

### A5. „provjeriti terminologiju i literaturu te ujednačiti stil rada; izraze poput „no fake grasp" i „pravilo poštenja" zamijeniti formalnijim izrazima" ✅

- oba imenovana izraza uklonjena (0 pojavljivanja);
- formalizirano još šest kolokvijalnih mjesta: „golim okom", „čest je izvor zabune", „nitko ga ne
  čita", „radije stane nego da okrzne dovratnik", „čovjek ili program", „dok čovjek pritisne gumb";
- literatura: **15 jedinica, sve citirane u tekstu** (provjereno automatski, nema neupotrijebljenih
  referenci).

## B. Smjernice na objavljeni repozitorij

### B1. Video cijelog sustava ✅
`docs/demo.mp4` — jedan neprekinut prolaz, 5 min 58 s, bez rezova. Od čekanja naredbe do dovršene
misije; završava robotovim vlastitim ispisom „Kocka je točno na markeru! Odstupanje od centra:
3 mm". Uvršten u README s posterom. Izvornik (1080p60 sa zvukom) u `dist/`, izvan gita.

### B2. README na engleskom, samostalan, sa slikama ✅
Upute i slike u `docs/`, nijedna poveznica ne vodi u `notes/`. Nosi video, tablicu iz 40 izvođenja
i postupak kojim se serija reproducira.

### B3. Razdvajanje radnog i release repozitorija ⚠ **otvoreno**
`.gitignore` drži radni materijal vani (LaTeX izvor, `dist/`, `data/`), ali **`notes/` je i dalje u
indeksu — 148 datoteka**. Drugi repozitorij ne postoji. Ovo je jedina profesorova stavka koja nije
zatvorena.

### B4. Usklađivanje seminara sa zadatkom ✅
20 zahtjeva dosljedno kroz rad (17 ispunjeno, 3 uz obrazloženo odstupanje), tablica triju veličina
zazora, ručno mapiranje navedeno izrijekom, kontaktni senzor opisan kao **uvjet**, a ne kao
mjerenje sile.

## C. Greške nađene vlastitom provjerom (21. 9.)

Nisu bile na profesorovu popisu, ali bi ih pregled otkrio:

- **tlocrt (slika 4.2) tvrdio je „otvor 0,98 m"** — to je širina koju *karta* javlja, a tlocrt je
  plan *svijeta*, gdje su otvori 1,00 m. Potvrđeno alatom: `width 0.980 m vs 1.00 m true`.
  Ispravljeno, i dva mjesta u tekstu koja su istu razliku brisala sada kažu koja je koja;
- **zastarjele brojke iz jednog izvođenja**: zaključak („5 mm"), lokalizacija (3,0 / 2,9 cm /
  0,3°), tablica zazora („kroz pet prolazaka"), točnost hvata (0,9–4,3 mm). Sve sada iz serije;
- **preostala pogreška odlaganja objašnjena**: uzdužnih 7,0 mm nije šum nego precjenjivanje
  udaljenosti markera za 0,8 % (kamera javlja 0,932 m gdje je istina 0,924 m, n=12).
  → [[P-54_place_bias_not_localisation]];
- **jedno izvođenje od 40 trebalo je oporavak** od zapinjanja i navedeno je u radu kao objašnjenje
  gornjeg kraja raspona trajanja, a ne prešućeno. → [[P-57_stall_recovery_is_slow]].
