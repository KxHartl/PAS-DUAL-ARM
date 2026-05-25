# LiteRealm — Pravila za AI Agente

Ovo je jednostavan workspace za pisanje seminara, zadaća i akademskih radova uz pomoć AI agenata.

## Direktoriji (strogo poštivati)

| Direktorij | Što ide ovdje |
|---|---|
| `docs/` | `.tex` fajlovi, generirani `.pdf`-ovi, slike i LaTeX resursi. |
| `src/` | Programski kod (`.py`, `.cpp`, `.js`…) ako zadatak to zahtijeva. |
| `dist/` | Konačne verzije za predaju (finalni PDF, zip). |
| `data/raw/` | Izvorni, neizmijenjeni podaci i literatura. Ne mijenjati. |
| `data/processed/` | Obrađeni podaci (grafikoni, tablice, filtrirani dataseti). |
| `data/rag/sources/` | PDF izvori za RAG citiranje (ako je RAG uključen). |

## LaTeX

1. Provjeri `project.yaml` za odabrani LaTeX format (`latex_format` polje).
2. Predlošci su u `.ai/templates/` — kopiraj odgovarajući u `docs/` i prilagodi.
3. **Uvijek** generiraj `.pdf` nakon pisanja.
4. Za kompilaciju koristi: `.ai/scripts/helpers/build-docs.ps1` (ili `.sh`).

## RAG — Citiranje iz izvora

Ako je RAG uključen (`rag_mode` u `project.yaml`), agent može pretraživati korisnikove PDF izvore:

1. **Ingestija**: `python .ai/rag/ingest.py` — parsira PDF-ove iz `data/rag/sources/`.
2. **Pretraga**: `python .ai/rag/query.py "pitanje"` — vraća relevantne odlomke s izvorom i stranicom.
3. **Citiranje**: Koristi dobivene reference za precizno citiranje u seminaru (npr. `[Izvor, str. X]`).

## Global Brain

Ako `brain_mode: global` u `project.yaml`, agent ima pristup dijeljenom znanju u `~/.agentbrain`:
- Čitaj naučene lekcije i obrasce iz prethodnih projekata.
- Piši nove lekcije nakon završetka zadatka.

## Komunikacija

- **Chat**: Hrvatski jezik.
- **Kod i commit poruke**: Engleski jezik.
- **Commit format**: Conventional Commits (`feat:`, `fix:`, `docs:`).

## Workflow

1. Pročitaj `STATE.md` za kontekst trenutnog zadatka.
2. Pročitaj `project.yaml` za LaTeX format i RAG/Brain konfiguraciju.
3. Radi u `docs/` (tekst) ili `src/` (kod).
4. Ako je RAG uključen, koristi `query.py` za pronalaženje relevantnih izvora.
5. Generiraj PDF. Commitaj na `main`.
