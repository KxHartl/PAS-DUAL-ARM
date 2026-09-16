---
id: DANAS
type: plan
updated: 2026-09-13
---
# Zadnji dan (13. 9. 2026.): gap analiza i redoslijed

> [!important] Redoslijed misije je iz [MAIL]
> **Mapiranje (SLAM) → korisnik zada regiju (Nav2 u plavu sobu) → pronađi kutiju → podigni je
> objema rukama → nosi je kroz vrata (HOME → CRVENA) → odloži je na `place_table`.**
> Svijet s tri sobe ([[D-13_three_room_world]]) taj redoslijed i nameće: s početne poze kutija se
> ne vidi.

## Gap analiza (obavezno iz [ZAD]/[MAIL] vs stanje)
| Zahtjev | Stanje | Težina popravka | Vrijednost za ocjenu |
|---|---|---|---|
| [[R-10_mappable_world]] + [[R-11_door_80cm]] + [[R-13_destination_place]] svijet | ✅ tri sobe, vrata 0.9 m (13. 9.), čeka GUI | — | visoka |
| [[R-14_slam_mapping]] mapiranje | ❌ (radilo 23. 6.) | srednji (kod postoji; rizik [[P-11_nav2_slam_drift]]) | visoka, „koristiti“ |
| [[R-15_region_goal_nav2]] regija → Nav2 | ❌ | srednji | visoka |
| [[R-17_dual_arm_lift]] hvat objema rukama | ✅ (🧪 zadnje izmjene) | nizak (run u plavoj sobi) | jezgra demoa |
| [[R-18_door_pass_empty]] / [[R-19_door_pass_with_box]] prolaz | ⚠ / ❌ | srednji / visok ([[P-18_transport_drops_box]]) | visoka |
| [[R-20_place_at_destination]] odlaganje | ⚠ | nizak nakon R-19 (isti stol, ista visina) | visoka |
| [[R-08_omni_controller]] omni_controller | ❌ | srednji–visok ([[P-09_omni_drive_on_fortress]]) | visoka, „obavezno“ |
| [[R-21_deliverables]] seminar, video, slajdovi | ❌ | **siguran trošak ~5–6 h** | nužno |

## Nalazi sesije 13. 9. (faza identifikacije, bez rješavanja)
| Nalaz | Posljedica | Kartica |
|---|---|---|
| Svijet s tri sobe radi, 8/8 kontrolera | temelj je spreman | [[D-13_three_room_world]] |
| SLAM + Nav2 se dižu, `/map` postoji | stack je upotrebljiv | [[S-06_navigation]] |
| Nav2: `inflation_radius` 0.05 < upisani radijus 0.31 | putovi ljube zidove → 0.40 | [[P-12_door_too_narrow]] |
| **Robot u carry pozi je 1.26 m širok, vrata su 0.9 m** | **prolaz je blokiran prije bilo kojeg Nav2 testa** | [[P-35_arm_span_too_wide_for_door]] |
| Komentar „ARM_CARRY ~0.6 m“ u kodu je netočan | lipanjski prolaz je uspio samo zbog vrata od 1.2 m | [[P-35_arm_span_too_wide_for_door]] |

**Kritični put se time promijenio:** uska poza za vrata je preduvjet za R-18, R-19 i R-20. Bez nje
nema ni prolaza ni dostave, bez obzira na Nav2 i transport-probu.

## Redoslijed (prijedlog, potvrđuje korisnik)
| # | Stavka | Time-box | Dovoljno dobro | Plan B |
|---|---|---|---|---|
| 0 | GUI provjera novog svijeta + sanity okoliša ([[S-10_build_run_environment]]) | 20 min | 3 sobe, vrata, stolovi, kutija, 8 kontrolera | popraviti SDF |
| 1 | **SLAM mapiranje** 3 sobe: `nav2.launch.py` (slam_toolbox) + spora vožnja kroz sobe, spremiti kartu ([[P-11_nav2_slam_drift]]). **Preduvjet:** ruke u `ARM_CARRY` (nakon spawna strše u stranu, [[P-12_door_too_narrow]]) | 60 min | karta sve 3 sobe u RViz-u, spremljena | teleop mapiranje, bez autonomije |
| 2 | **Nav2 do regije** (RViz „2D Goal Pose“ u plavoj sobi) → postojeći find/grasp (`main_task` od koraka SCAN) | 60 min | robot u plavoj sobi pred kutijom; run 31+ hvat ([[P-28_gate_too_strict]]) | ručni dovoz + hvat |
| 3 | **Video** hvata u plavoj sobi | 15 min | 1 čist ciklus snimljen | najbolji dostupni run |
| 4 | **Nošenje**: `ARM_CARRY` s kutijom → Nav2 kroz vrata u crvenu sobu → odlaganje na `place_table` ([[P-18_transport_drops_box]], [[P-12_door_too_narrow]]) | 90 min | kutija na stolu u crvenoj sobi | proba prijevoza + iskreno odstupanje |
| 5 | `mecanum_drive_controller` ([[P-09_omni_drive_on_fortress]]) | 60 min | kontroler aktivan, x + yaw rade | diff_drive → odstupanje |
| ∥ | **Seminar** ([[seminar_mapa]]): pisanje od **najkasnije** sredine dana, usporedno s runovima | 4 h | sva poglavlja + slike | skraćena poglavlja 7–9 |
| ∥ | **Slajdovi** (iz seminara) | 1 h | 10–12 slajdova | — |
| end | commit, tag predaje, `dist/` | 20 min | čist repo | — |

**Obrazloženje:** redoslijed prati misiju iz [MAIL] (SLAM prvo, kako je korisnik rekao). Svaki
korak ostavlja nešto što se može pokazati: kartu, dolazak u regiju, hvat, prijenos. Omni je
time-boxan, a neuspjeh se piše kao iskreno odstupanje.

## Checklist predaje ([[R-21_deliverables]])
- [ ] `seminar.tex` → PDF (FSB predložak i `build-docs.sh` su **lokalno**, u `.ai/`, izvan
      repozitorija od 16. 9.; vidi [[vanjski_paketi]])
- [ ] slike: robot (Gazebo), svijet s tri sobe, **SLAM karta u RViz-u**, TF stablo, graf čvorova,
      hvat (sekvenca), dijagram slijeda misije
- [ ] video (GUI run) → `dist/`
- [ ] slajdovi → `dist/`
- [ ] `README.md` + `RUNNING.md` ažurni (jedna naredba do demoa)
- [ ] [[odstupanja]] prenesena u seminar (poglavlje „Ograničenja“)
- [ ] `git tag` predaje, `git status` čist
