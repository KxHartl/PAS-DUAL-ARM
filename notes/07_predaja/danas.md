---
id: DANAS
type: plan
updated: 2026-09-18
---
# Plan predaje

> [!warning] Gap analiza ispod je **arhiva od 13. 9. 2026.**
> Sve što je ondje označeno kao ❌ ili ⚠ zatvoreno je 15.–16. 9. Aktualno stanje je u
> [[00_MAPA]], a preostala odstupanja u [[odstupanja]]. Tablica se čuva jer pokazuje
> redoslijed rada, ne trenutno stanje.

## Riješeno 18. 9. (jutro): misija opet vozi
Dva dana je izgledalo da se „projekt sam prepravio". **Git je bio netaknut** — radno stablo čisto,
`git fsck` bez izgubljenih commitova, `main` čisti potomak `origin/main`, ništa prepisano.
Uzrok je bila **`apt` nadogradnja 439 `ros-humble-*` paketa 17. 9. u 18:09**, koja zaustavlja AMCL
usred vožnje ([[P-51_apt_upgrade_stops_amcl]]). Nakon vraćanja stacka na snapshot `2026-08-07`
misija vozi **`MISSION COMPLETE`, 5 mm od centra markera**, s nepromijenjenim kodom ([[runovi]] B1).

- Sav noćni rad (automatsko mapiranje, ispitni sklop, docs) ostaje na `main`; ništa nije vraćeno
  unatrag. Sigurnosna kopija prije zahvata: grana `wip/automated-mapping-18-09` + tag
  `backup-18-09-prije-povratka`.
- **ROS stack je zaključan** (`apt-mark hold`, 543 paketa; živi repo isključen). Ne puštati
  automatske nadogradnje do predaje ([[S-10_build_run_environment]]).

## Stanje predaje 20. 9. 2026. (navečer)
1. **Video cijelog sustava** — ✅ snimljen 21. 9. (korisnik, OBS). Jedan neprekinut prolaz,
   5 min 58 s, od čekanja naredbe do dovršene misije uz 3 mm od markera. U repozitoriju kao
   `docs/demo.mp4` (1080p30, bez zvuka, 13 MB — ispod GitHubove granice od 100 MB), uvršten u
   README s posterom. Izvornik (1080p60 sa zvukom, 105 MB) stoji u `dist/`, izvan gita.
2. **README na engleskom** — ✅, i više ne citira jedan run nego seriju.
3. **Seminar usklađen sa serijom** — ✅. Brojke generira `validation/seminar_numbers.py` u
   `seminar/mjerenja.tex`; tekst ih samo uvlači. Dodano: tablica statistike, donja granica
   uspješnosti po pravilu $1-3/N$, potpoglavlje o sustavnoj pogrešci procjene udaljenosti.
4. **Statistika** — ✅ **40 od 40**. `cam-par` (12) + `n40` (28), isti kod, isti postav, dva
   radnika u oba slučaja, RTF 0,39 u oba — dakle jedan uzorak za sve veličine. Donja granica
   uspješnosti uz 95 % povjerenja: **92,5 %**. Medijan odlaganja 8,0 mm, misija 173,0 s.
   Jedan run (1/40) pokazao je zastoj i oporavak → [[P-57_stall_recovery_is_slow]].
5. **Slajdovi** — otpada (korisnik, 20. 9.).
6. **`git status` čist** — ⚠ `build.pre-18-09/` i `install.pre-18-09/` (253 MB) stoje nepraćeni
   od oporavka 18. 9.; sustav je otad prebuildan i odvozio 40 runova, pa više ne služe ničemu.
   Brisanje čeka odluku korisnika.
7. **Push na `origin`** — ❌ ništa nije gurnuto; grane i tagovi su lokalni.

## Aktualno otvoreno (stanje 18. 9. 2026.)
Nakon profesorovih smjernica na objavljeni repo i seminar:
1. **Video cijelog sustava** — ❌ ne postoji. `scripts/record_demo.sh` snima GUI run.
2. **README na engleskom, samostalan, sa slikama** — ✅ 17. 9.; upute u `docs/`, slike u
   `docs/img/`, nijedne poveznice u `notes/`.
3. **Razdvajanje radnog i release repozitorija** — ⚠ `.gitignore` drži radni materijal vani, ali
   `notes/` je i dalje u indeksu; drugi repozitorij ne postoji.
4. **Uskladiti seminar** — ✅ 17. 9.: 20 zahtjeva dosljedno, tablica triju zazora, 0,835 m
   objašnjeno, ručno mapiranje izrijekom, kontaktni senzor kao uvjet a ne mjerenje sile,
   literatura s citatima.
5. **Ponavljanja misije i statistika** — ⚠ sklop postoji; **podaci od 17. 9. su nevažeći** jer su
   snimljeni na slomljenom stacku ([[P-51_apt_upgrade_stops_amcl]]). Seriju treba **ponoviti od
   nule** na vraćenom stacku; prvi ispravan run je B1 ([[runovi]]).
6. Slajdovi — **otpada** (korisnik, 20. 9.: prezentacije nema, predaje se rad).

### Profesorovi komentari, doslovno (seminar)
> - provjeriti tvrdnju o udaljenosti 0,835 m od prepreka jer iz trenutnog opisa nije jasno na što
>   se točno odnosi
> - rezultati bi bili uvjerljiviji kada bi se prikazalo nekoliko ponavljanja kompletne završne
>   misije i osnovna statistika uspješnosti
> - ublažiti izraze poput geometrijski egzaktna karta, milimetarska točnost i determinističko i
>   robusno izvršavanje ako nisu potkrijepljeni dovoljnim brojem ponavljanja
> - smanjiti količinu detalja i opisa pojedinih pokretanja, fokusirati tekst na konačno rješenje,
>   algoritam i rezultate
> - provjeriti terminologiju i literaturu te ujednačiti stil rada; izraze poput „no fake grasp“ i
>   „pravilo poštenja“ u tekstu zamijeniti formalnijim izrazima

Prva tri se **zatvaraju istim podacima**: serijom runova sa snimljenim bagovima
([[D-24_measure_from_recordings_not_logs]]). 0,835 m postaje izmjerena veličina umjesto offline
izračuna, statistika dolazi iz `metrics.csv`, a jačina izraza se veže uz N koji stvarno imamo.

## Gap analiza (13. 9. 2026., arhiva)

> [!important] Redoslijed misije je iz [MAIL]
> **Mapiranje (SLAM) → korisnik zada regiju (Nav2 u plavu sobu) → pronađi kutiju → podigni je
> objema rukama → nosi je kroz vrata (HOME → CRVENA) → odloži je na `place_table`.**
> Svijet s tri sobe ([[D-13_three_room_world]]) taj redoslijed i nameće: s početne poze kutija se
> ne vidi.

## Gap analiza (obavezno iz [ZAD]/[MAIL] vs stanje)
| Zahtjev | Stanje | Težina popravka | Vrijednost za ocjenu |
|---|---|---|---|
| [[R-10_mappable_world]] + [[R-11_door_80cm]] + [[R-13_destination_place]] svijet | ✅ tri sobe, vrata 0.9 m (13. 9.), čeka GUI | — | visoka |
| [[R-14_slam_mapping]] mapiranje | ❌ (radilo 23. 6.) | srednji (kod postoji; rizik [[P-11_nav2_slam_drift]]) | visoka, „koristiti“ |
| [[R-15_region_goal_nav2]] regija → Nav2 | ❌ | srednji | visoka |
| [[R-17_dual_arm_lift]] hvat objema rukama | ✅ (🧪 zadnje izmjene) | nizak (run u plavoj sobi) | jezgra demoa |
| [[R-18_door_pass_empty]] / [[R-19_door_pass_with_box]] prolaz | ⚠ / ❌ | srednji / visok ([[P-18_transport_drops_box]]) | visoka |
| [[R-20_place_at_destination]] odlaganje | ⚠ | nizak nakon R-19 (isti stol, ista visina) | visoka |
| [[R-08_omni_controller]] omni_controller | ❌ | srednji–visok ([[P-09_omni_drive_on_fortress]]) | visoka, „obavezno“ |
| [[R-21_deliverables]] seminar, video, slajdovi | ❌ | **siguran trošak ~5–6 h** | nužno |

## Nalazi sesije 13. 9. (faza identifikacije, bez rješavanja)
| Nalaz | Posljedica | Kartica |
|---|---|---|
| Svijet s tri sobe radi, 8/8 kontrolera | temelj je spreman | [[D-13_three_room_world]] |
| SLAM + Nav2 se dižu, `/map` postoji | stack je upotrebljiv | [[S-06_navigation]] |
| Nav2: `inflation_radius` 0.05 < upisani radijus 0.31 | putovi ljube zidove → 0.40 | [[P-12_door_too_narrow]] |
| **Robot u carry pozi je 1.26 m širok, vrata su 0.9 m** | **prolaz je blokiran prije bilo kojeg Nav2 testa** | [[P-35_arm_span_too_wide_for_door]] |
| Komentar „ARM_CARRY ~0.6 m“ u kodu je netočan | lipanjski prolaz je uspio samo zbog vrata od 1.2 m | [[P-35_arm_span_too_wide_for_door]] |

**Kritični put se time promijenio:** uska poza za vrata je preduvjet za R-18, R-19 i R-20. Bez nje
nema ni prolaza ni dostave, bez obzira na Nav2 i transport-probu.

## Redoslijed (prijedlog, potvrđuje korisnik)
| # | Stavka | Time-box | Dovoljno dobro | Plan B |
|---|---|---|---|---|
| 0 | GUI provjera novog svijeta + sanity okoliša ([[S-10_build_run_environment]]) | 20 min | 3 sobe, vrata, stolovi, kutija, 8 kontrolera | popraviti SDF |
| 1 | **SLAM mapiranje** 3 sobe: `nav2.launch.py` (slam_toolbox) + spora vožnja kroz sobe, spremiti kartu ([[P-11_nav2_slam_drift]]). **Preduvjet:** ruke u `ARM_CARRY` (nakon spawna strše u stranu, [[P-12_door_too_narrow]]) | 60 min | karta sve 3 sobe u RViz-u, spremljena | teleop mapiranje, bez autonomije |
| 2 | **Nav2 do regije** (RViz „2D Goal Pose“ u plavoj sobi) → postojeći find/grasp (`main_task` od koraka SCAN) | 60 min | robot u plavoj sobi pred kutijom; run 31+ hvat ([[P-28_gate_too_strict]]) | ručni dovoz + hvat |
| 3 | **Video** hvata u plavoj sobi | 15 min | 1 čist ciklus snimljen | najbolji dostupni run |
| 4 | **Nošenje**: `ARM_CARRY` s kutijom → Nav2 kroz vrata u crvenu sobu → odlaganje na `place_table` ([[P-18_transport_drops_box]], [[P-12_door_too_narrow]]) | 90 min | kutija na stolu u crvenoj sobi | proba prijevoza + iskreno odstupanje |
| 5 | `mecanum_drive_controller` ([[P-09_omni_drive_on_fortress]]) | 60 min | kontroler aktivan, x + yaw rade | diff_drive → odstupanje |
| ∥ | **Seminar** ([[seminar_mapa]]): pisanje od **najkasnije** sredine dana, usporedno s runovima | 4 h | sva poglavlja + slike | skraćena poglavlja 7–9 |
| ∥ | **Slajdovi** (iz seminara) | 1 h | 10–12 slajdova | — |
| end | commit, tag predaje, `dist/` | 20 min | čist repo | — |

**Obrazloženje:** redoslijed prati misiju iz [MAIL] (SLAM prvo, kako je korisnik rekao). Svaki
korak ostavlja nešto što se može pokazati: kartu, dolazak u regiju, hvat, prijenos. Omni je
time-boxan, a neuspjeh se piše kao iskreno odstupanje.

## Checklist predaje ([[R-21_deliverables]])
- [x] `seminar.tex` → PDF (16. 9.; FSB predložak i `build-docs.sh` su **lokalno**, u `.ai/`,
      izvan repozitorija; vidi [[vanjski_paketi]])
- [x] slike: 13 rasterskih + 5 TikZ dijagrama u `seminar/slike/` (16. 9.)
- [ ] video (GUI run) → `dist/`
- [ ] slajdovi → `dist/`
- [ ] `README.md` **na engleskom**, samostalan, sa slikama; `RUNNING.md` ažuran
- [x] [[odstupanja]] prenesena u seminar (poglavlje „Ograničenja“)
- [ ] `git tag` predaje, `git status` čist
