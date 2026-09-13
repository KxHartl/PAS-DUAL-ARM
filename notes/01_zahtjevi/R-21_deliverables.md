---
id: R-21
type: zahtjev
status: otvoreno
source: "korisnik (13. 9. 2026.)"
parent: "[[00_MAPA]]"
solutions: []
problems: []
decisions: []
updated: 2026-09-13
---
# R-21: Predaja: seminar, repo + simulacija, video, prezentacija

## Izvor
Korisnik, 13. 9. 2026. (zadnji dan). [ZAD] i [MAIL] ne propisuju oblik predaje.

## Tehnički znači
| Predmet | Oblik | Gdje | Kriterij |
|---|---|---|---|
| Seminarski rad | PDF po FSB predlošku (`latex_format: fsb-seminar`) | `docs/` → `dist/` | sva poglavlja, slike, iskrena odstupanja ([[odstupanja]]) |
| Repo + simulacija | GitHub repo, `README` + `RUNNING.md` | root | čisti build iz checkouta, jedna naredba do demoa |
| Video | snimka Gazebo GUI runa | `dist/` | cijela misija ili najdalji stabilni dio, s naslovima koraka |
| Prezentacija | slajdovi | `dist/` | cilj → arhitektura → rezultati → problemi → odstupanja |

**Kriterij prihvaćanja:**
- [ ] `dist/` sadrži PDF seminara, video i slajdove
- [ ] `docs/` sadrži `.tex` izvor, a PDF je generiran (`.ai/scripts/helpers/build-docs.sh`)
- [ ] repo čist, zadnji commit označen (tag predaje)

## Trenutno stanje
❌ `docs/` je prazan (samo README), `dist/` prazan. Mapa sadržaja: [[seminar_mapa]]. Redoslijed
dana: [[danas]].
