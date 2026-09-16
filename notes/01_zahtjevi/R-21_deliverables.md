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
| Seminarski rad | PDF po FSB predlošku (`latex_format: fsb-seminar`) | lokalno (izvan repoa) → predaja | sva poglavlja, slike, iskrena odstupanja ([[odstupanja]]) |
| Repo + simulacija | GitHub repo, `README.md` + `MAPPING.md` + `RUNNING.md` | root | čisti build iz checkouta, jedna naredba do demoa |
| Video | snimka Gazebo GUI runa | `dist/` | cijela misija ili najdalji stabilni dio, s naslovima koraka |
| Prezentacija | slajdovi | `dist/` | cilj → arhitektura → rezultati → problemi → odstupanja |

**Kriterij prihvaćanja:**
- [ ] PDF seminara, video i slajdovi spremni za predaju
- [ ] `.tex` izvor napisan, PDF generiran (LaTeX predložak i `build-docs.sh` su **lokalno**,
      u `.ai/`, izvan repozitorija od 16. 9.)
- [x] repo čist (16. 9.: README/MAPPING/RUNNING prepisani, scaffolding i zastarjeli dokumenti
      izašli iz indeksa, vanjski paketi samo preko `ros2.repos` — [[vanjski_paketi]])
- [ ] zadnji commit označen (tag predaje)

## Trenutno stanje
⚠ Repozitorij je pripremljen za predaju (16. 9.). Seminar, video i slajdovi još nisu napravljeni.
Mapa sadržaja: [[seminar_mapa]]. Redoslijed dana: [[danas]].
