---
id: RUN_START
type: upute
updated: 2026-09-15
---
# Pokretanje i rad na projektu

Ovo je operativna ulazna točka. Status zahtjeva i razlozi odluka su u [[00_MAPA]],
a pravila rada u [[AGENT_GUIDE]].

- [[01_pokretanje]] — build, simulacija, SLAM, lokalizacija i zadatak.
- [[02_testiranje]] — što provjeriti i kako razlikovati prolaz od lažnog uspjeha.
- [[03_izmjene]] — gdje mijenjati parametre/kod i kako zabilježiti pokus.
- `00_testing/` — **postupci testiranja po podsustavu**: koje naredbe, u kojem terminalu,
  što gledati i što zabilježiti: [[navigacija]], [[hvat_kocke]],
  [[definiranje_poza_hvata]] (robot pred stolom, ruke u skenirajućoj pozi, poze se
  zadaju u `joint_gui.py --sim` i šalju simulaciji).

Projektni okoliš je `scripts/run_native.sh`, a brza dijagnostika
`bash scripts/verify_environment.sh` (ili `--live` uz pokrenutu simulaciju).

**Stanje 14. 9. 2026.:** karta je prihvaćena (`maps/seminar_map.*`, run 44). Navigacija
radi na **zonama izvedenim iz detektiranih vrata i stolova** ([[D-16_zones_from_detected_features]]),
a Nav2 vozi svaku dionicu — kod vrata su samo portalne poze i preduvjet. Referentna vožnja
je run 46 ([[runovi]]): plava soba za 33.1 s, crveni stol za 60.6 s.

Novo: robot Nav2-u objavljuje **stvarni obris** koji prati ruke, uz `collision_monitor`
kao sloj koji ne ovisi o costmapu ([[D-19_dynamic_footprint]]). Još nije odvoženo —
postupak je u [[navigacija]]. Hvat u novom svijetu i transport nisu potvrđeni.

Bez simulatora se detekcija i zone provjere u sekundi:
`python3 scripts/check_doors.py` i `python3 scripts/check_zones.py`.
