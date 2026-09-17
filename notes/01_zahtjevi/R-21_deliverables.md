---
id: R-21
type: zahtjev
status: djelomicno
verified: "seminar i repo gotovi 16. 9.; video i slajdovi otvoreni"
source: "korisnik (13. 9. 2026.)"
parent: "[[00_MAPA]]"
solutions: []
problems: []
decisions: []
updated: 2026-09-17
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
- [x] `.tex` izvor napisan, PDF generiran 16. 9. (LaTeX predložak i `build-docs.sh` su
      **lokalno**, u `.ai/`, izvan repozitorija)
- [ ] seminar usklađen s profesorovim smjernicama (17. 9. — vidi [[danas]])
- [ ] **video** snimljen
- [ ] slajdovi
- [x] repo čist (16. 9.: README/MAPPING/RUNNING prepisani, scaffolding i zastarjeli dokumenti
      izašli iz indeksa, vanjski paketi samo preko `ros2.repos` — [[vanjski_paketi]])
- [ ] zadnji commit označen (tag predaje)

## Trenutno stanje
⚠ **Seminar je napisan i PDF generiran (16. 9.)**, repozitorij je pripremljen za predaju.

Otvoreno nakon profesorovih smjernica (17. 9.):
- **video** — nužno, ne postoji nijedna snimka ni skripta za snimanje;
- **slajdovi**;
- usklađivanje seminara i README-a (README mora biti **na engleskom**, samostalan i sa slikama);
- razdvajanje radnog i release repozitorija;
- tag predaje.

Mapa sadržaja: [[seminar_mapa]]. Redoslijed: [[danas]].
