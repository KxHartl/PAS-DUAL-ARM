---
id: DANAS
type: plan
updated: 2026-09-13
---
# Zadnji dan (13. 9. 2026.): gap analiza i predloženi redoslijed

> [!important] Ovo je PRIJEDLOG
> Redoslijed i time-boxove potvrđuje korisnik. Svaka stavka ima kriterij „dovoljno dobro“ i plan B
> (ako ne uspije → iskreno u [[odstupanja]]).

## Gap analiza (obavezno iz [ZAD]/[MAIL] vs stanje)
| Zahtjev | Stanje | Težina popravka | Vrijednost za ocjenu |
|---|---|---|---|
| [[R-17_dual_arm_lift]] hvat objema rukama | ✅ (🧪 zadnje izmjene) | nizak (samo run) | visoka, jezgra demoa |
| [[R-11_door_80cm]] vrata 80 cm | 🔁 2.0 m | **nizak** (SDF) | visoka, eksplicitno traženo |
| [[R-18_door_pass_empty]] prolaz prazan | ⚠ | nizak–srednji | visoka |
| [[R-19_door_pass_with_box]] prolaz s kutijom | ❌ | **visok** ([[P-18_transport_drops_box]]) | visoka |
| [[R-20_place_at_destination]] odlaganje na odredište | ⚠ | srednji (ovisi o R-19) | visoka |
| [[R-08_omni_controller]] omni_controller | ❌ | srednji–visok ([[P-09_omni_drive_on_fortress]]) | visoka, „obavezno“ |
| [[R-14_slam_mapping]] + [[R-15_region_goal_nav2]] SLAM + Nav2 | ❌ | srednji (kod postoji) | visoka, „koristiti“ |
| [[R-21_deliverables]] seminar, video, slajdovi | ❌ | **siguran trošak ~5–6 h** | nužno |

## Predloženi redoslijed
| # | Stavka | Time-box | Dovoljno dobro | Plan B |
|---|---|---|---|---|
| 0 | Sanity: build + GUI sim u novom okolišu ([[S-10_build_run_environment]]) | 20 min | 8 kontrolera aktivno | popraviti okoliš, sve drugo čeka |
| 1 | **Run 31+** hvat + kontaktni place, **snimiti video** | 45 min | ≥ 1 čist ciklus snimljen | video najboljeg dostupnog runa |
| 2 | Vrata → 0.8 m + prolaz **bez** kutije ([[P-12_door_too_narrow]]) | 30 min | GUI prolaz bez kontakta | vožnja odometrijom umjesto Nav2 |
| 3 | Transport-proba → carry kroz vrata → place na `target_table` ([[P-18_transport_drops_box]]) | 90 min | kocka preživi 0.4 m + 60° | matrica B; ako ne → odstupanje |
| 4 | `mecanum_drive_controller` ([[P-09_omni_drive_on_fortress]]) | 60 min | kontroler aktivan, x + yaw rade | ostati na diff_drive → odstupanje |
| 5 | SLAM + Nav2 hibrid ([[P-11_nav2_slam_drift]]) | 60 min | karta + 1 cilj koji zada čovjek | karta iz ranijeg testa + odstupanje |
| ∥ | **Seminar** ([[seminar_mapa]]): pisanje od **najkasnije** sredine dana, usporedno s runovima | 4 h | sva poglavlja + slike | skraćena poglavlja 7–9 |
| ∥ | **Slajdovi** (iz seminara) | 1 h | 10–12 slajdova | — |
| end | commit, tag predaje, `dist/` | 20 min | čist repo | — |

**Obrazloženje redoslijeda:**
- Prvo osigurati ono što se sigurno može predati (demo hvata + video).
- Zatim najjeftinije zatvaranje obaveznog zahtjeva (vrata).
- Potom najvrjedniji otvoreni dio misije (transport).
- Omni i Nav2 su skuplji, pa su time-boxani. Neuspjeh se piše kao iskreno odstupanje.

## Checklist predaje ([[R-21_deliverables]])
- [ ] `docs/seminar.tex` (FSB predložak iz `.ai/templates/fsb-seminar/latex/`) → PDF
      (`.ai/scripts/helpers/build-docs.sh`)
- [ ] slike: robot (Gazebo), svijet, RViz karta, TF stablo, graf čvorova, hvat (sekvenca), dijagram
      slijeda misije
- [ ] video (GUI run) → `dist/`
- [ ] slajdovi → `dist/`
- [ ] `README.md` + `RUNNING.md` ažurni (jedna naredba do demoa)
- [ ] [[odstupanja]] prenesena u seminar (poglavlje „Ograničenja“)
- [ ] `git tag` predaje, `git status` čist
