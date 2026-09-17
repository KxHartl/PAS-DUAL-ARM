---
id: D-24
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-21_deliverables]]", "[[R-15_region_goal_nav2]]", "[[R-20_place_at_destination]]"]
problems: []
superseded_by: ""
updated: 2026-09-18
---
# D-24 — Runovi se snimaju kao podaci, ne samo kao ispis

## Kontekst
Prva verzija ispitnog sklopa (17. 9.) sudila je run **isključivo iz njegova loga**. Korisnik je
18. 9. prigovorio točno na to: ispis je pisan da ga čovjek čita u hodu, a ne da se obrađuje, i u
njemu može biti samo ono što je netko unaprijed odlučio ispisati.

Prigovor drži uz profesorove komentare, jer se **nijedan** od njih ne može zatvoriti iz loga:

| Komentar | Što traži | Ima li toga u logu |
|---|---|---|
| provjeriti tvrdnju 0,835 m od prepreka | izmjerenu udaljenost vožene putanje od ruba ploče stola | ne |
| nekoliko ponavljanja + statistika uspješnosti | N runova i raspodjela | djelomično (ishod da, mjere ne) |
| ublažiti „milimetarska točnost", „determinističko i robusno" | nezavisnu mjeru odlaganja i lokalizacije | ne — postoji samo robotova vlastita tvrdnja `PLACE VERIFIED` |
| manje opisa pojedinih pokretanja | tablicu umjesto pripovijedanja o runovima | ne |

## Opcije
1. **Samo log** (dosadašnje) — jeftino, ali svako novo pitanje traži novi run.
2. **Log + rosbag + naknadna obrada** — run se snimi jednom, pitanja se postavljaju poslije.
3. Namjenski čvor koji piše sažete mjere uživo — malen zapis, ali opet mjeri samo ono što smo se
   unaprijed sjetili mjeriti; to je ista greška u manjem pakiranju.

## Odluka
**Opcija 2**, 18. 9. Svaki run ostavlja dva zapisa koji odgovaraju na različita pitanja:

- **log** → *je li misija uspjela*. Uspjeh mora biti nešto što sustav **sam tvrdi**
  (`PLACE VERIFIED` + `MISSION COMPLETE`), a ne što mu analiza naknadno prizna
  ([[D-12_honesty_abort_over_fake]]).
- **rosbag** → *što se stvarno dogodilo*. Snima se i ground truth iz Gazeba
  (`/debug/gz_dynamic_pose`, `debug_truth:=true`, nitko na njemu ne upravlja), pa se run može
  mjeriti protiv toga **gdje je robot bio**, a ne protiv onoga što je mislio da jest.

`validation/analyze_runs.py` mjeri svaki bag protiv **svijeta tog runa** (`worlds/run_NNN.sdf`),
ne protiv konstanti u skripti, i piše `validation/results/metrics.csv`.

Mjeri se, među ostalim: najmanji razmak do **ruba ploče** stola pri prelasku sobe, isti razmak na
namjernom prilazu stolu (odvojeno), razmak do zidova, prolaz kroz svaka vrata, greška AMCL-a
prema ground truthu, te **gdje je kutija fizički stala** u odnosu na marker iz opisa svijeta.

Podjela „prelazak sobe" / „prilaz stolu" reže se na zadnjem trenutku u kojem je robot još bio
**1,20 m** od stola na koji se potom dokira. To je iznad brojke koja se provjerava (0,835 m), pa
provjera pada na prelasku sobe i ne može biti uljepšana prilazom.

## Posljedice
- Cijena: bag je nekoliko stotina MB po runu; serija od 20 mjeri se u gigabajtima. Ne commita se
  (`validation/results/` je u `.gitignore`), a nakon `metrics.csv` treba samo za pitanja koja
  metrike još ne pokrivaju. `--no-bag` i `--no-truth` gase snimanje.
- `mission.launch.py` i `scenario_mission.launch.py` dobili su prolaz za `debug_truth` do
  `sim.launch.py`; podrazumijevano je `false`, pa se demo ponaša kao fizički robot.
- Tvrdnja **0,835 m** prestaje biti offline izračun i postaje izmjerena veličina s rasponom preko
  N runova — ili se, ako mjerenje to ne potvrdi, u seminaru mijenja u ono što mjerenje kaže.

## Odnos prema zahtjevima
Ispunjava [[R-21_deliverables]] (statistika uspješnosti) i daje nezavisnu potvrdu za
[[R-15_region_goal_nav2]] i [[R-20_place_at_destination]], koje su dosad stajale na robotovoj
vlastitoj tvrdnji iz jednog runa.
