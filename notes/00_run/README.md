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

**Stanje 13. 9. 2026.:** nema prihvaćene karte u `src/pas_dual_arm_bringup/maps/`.
SLAM/AMCL/Nav2 konfiguracija i registar značajki postoje, ali prolaz kroz vrata,
lokalizacija na spremljenoj karti i navigacija nisu potvrđeni. Simulacija se diže;
automatska tura ima sigurnosni prekid ako stvarni zglobovi ne drže putnu pozu.
Hvat u novom svijetu nije ponovno potvrđen.
