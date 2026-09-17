---
id: R-05
type: zahtjev
status: ispunjeno
source: "[MAIL-slika]"
parent: "[[00_MAPA]]"
solutions: ["[[S-01_robot_description]]"]
problems: ["[[P-04_mesh_uri_not_found]]"]
decisions: []
updated: 2026-09-13
---
# R-05: Izgled kao na slici sustava

## Izvor (doslovno)
> „Šaljem sliku cijelog sustava kako bi trebao izgledati“ [MAIL]
> Slika: `data/raw/zadatak/mail_img-000.png`

![Ciljani izgled sustava (B. Ćaran, prilog uz [MAIL])](../../data/raw/zadatak/mail_img-000.png)

## Tehnički znači
Raspored komponenti odgovara slici:
- omni baza dolje;
- torzo od aluminijskih profila po sredini;
- ruke na bočnim klizačima, okrenute prema van;
- pan-tilt s kamerom na vrhu.

**Kriterij prihvaćanja:**
- [x] korisnik vizualno potvrdio model u RViz-u / Gazebu
- [x] cijeli robot se renderira u Gazebu (bez nevidljivih dijelova)

## Trenutno stanje
✅ Geometrija je kalibrirana 12. 6. uz vizualne provjere korisnika ( …;
protokol GUI checkpointa je danas [[AGENT_GUIDE]] §6). Mjere su u [[S-01_robot_description]]
§ „Geometrija montaže": torzo rpy 0, klizači na ±Y, desni rotiran 180°.
Izvor mjera je CAD slika `data/raw/zadatak/torzo_cad_mjere.png`. Mesh URI popravak
[[P-04_mesh_uri_not_found]].

## Kako se rješava
- [[S-01_robot_description]]
