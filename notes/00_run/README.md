---
id: RUN_START
type: upute
updated: 2026-09-13
---
# Pokretanje i rad na projektu

Ovo je operativna ulazna točka. Status zahtjeva i razlozi odluka su u [[00_MAPA]],
a pravila rada u [[AGENT_GUIDE]].

- [[01_pokretanje]] — build, simulacija, SLAM, lokalizacija i zadatak.
- [[02_testiranje]] — što provjeriti i kako razlikovati prolaz od lažnog uspjeha.
- [[03_izmjene]] — gdje mijenjati parametre/kod i kako zabilježiti pokus.

Projektni okoliš je `scripts/run_native.sh`, a brza dijagnostika
`bash scripts/verify_environment.sh` (ili `--live` uz pokrenutu simulaciju).

**Stanje 14. 9. 2026.:** karta je prihvaćena (`maps/seminar_map.*`, run 44). Navigacija
radi na **zonama izvedenim iz detektiranih vrata i stolova** ([[D-16_zones_from_detected_features]]):
provjereno headless da se stog diže, da je keepout filter aktivan na oba costmapa i da
planirane putanje sijeku prag vrata pod < 1.1°. **Nijedna dionica još nije odvožena** — to
je sljedeći korak (runovi 45+), prazan robot u `ARM_CARRY_V2`, bez kutije.
Prolaz kroz vrata, hvat u novom svijetu i transport nisu potvrđeni.

Bez simulatora se detekcija i zone provjere u sekundi:
`python3 scripts/check_doors.py` i `python3 scripts/check_zones.py`.
