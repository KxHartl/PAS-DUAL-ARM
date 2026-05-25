# LaTeX Predlošci

Tri akademska predloška za FSB (Fakultet strojarstva i brodogradnje), Sveučilište u Zagrebu.

## Dostupni predlošci

| Predložak | Direktorij | Format | Namjena |
|---|---|---|---|
| **Seminar** | `fsb-seminar/` | 12pt, A4 | Seminarski radovi |
| **Diplomski rad** | `fsb-thesis/` | 12pt, A4 | Diplomski/završni rad |
| **Paper** | `kx-paper/` | 10pt, dva stupca | Znanstveni/konferencijski rad |

## Kako koristiti

1. Odaberi predložak prema tipu rada.
2. Kopiraj sadržaj iz `<predložak>/latex/` u `docs/` direktorij projekta.
3. Prilagodi naslov, autora i sadržaj.
4. Kompajliraj s `build-docs` skriptom.

## Sadržaj svakog predloška

```
predložak/
├── latex/          # LaTeX izvorni fajlovi (.tex)
├── word/           # Word verzija (.docx) + referentni PDF
├── demo.pdf        # Primjer kompajliranog dokumenta
├── instructions.md # Upute za AI agenta
└── structure.md    # Skeleton strukture dokumenta
```

## LaTeX paketi

Potrebni paketi su navedeni u `latex-requirements.txt`.
