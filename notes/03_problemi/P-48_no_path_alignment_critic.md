---
id: P-48
type: problem
status: u_tijeku
verified: "izmjereno iz runa 17. 9. 22:49 (28 s mirovanja, nula grešaka u logu)"
requirements: ["[[R-15_region_goal_nav2]]", "[[R-14_slam_mapping]]"]
solutions: ["[[S-06_navigation]]"]
decisions: ["[[D-22_mapping_motion_profile]]"]
updated: 2026-09-17
---
# P-48: Nijedna kritika ne nagrađuje okretanje prema putanji

## Simptom
Dvije naizgled nepovezane pojave, isti uzrok:

1. **Robot vozi unatrag i bočno** i to izgleda nasumično (opažanje korisnika, 17. 9.).
2. Čim je vožnja unatrag zabranjena (`min_vel_x: 0.0` u profilu mapiranja), **robot stoji 28 s bez
   ijedne greške** u ijednom logu: cilj prihvaćen, `follow_path` se izvodi, `controller_server` bez
   greške, TF uredan, skenovi teku.

## Uzrok
Popis DWB kritika u `nav2_params.yaml` je:

```
["RotateToGoal", "Oscillation", "ObstacleFootprint", "PathDist", "GoalDist", "Twirling"]
```

U njemu **nema `PathAlign` ni `GoalAlign`** — dakle ničega što boduje *kurs* robota. `PathDist` i
`GoalDist` boduju samo **gdje trajektorija završi**. Na omni bazi to znači: do cilja iza sebe robot
dođe tako da vozi unatrag, jer je to jedino gibanje koje popravlja bodove. Okretanje prema cilju ne
donosi nijedan bod jer ne mijenja krajnju točku.

Kad se unatrag zabrani, a `RotateToGoal` ugasi:

| Opcija | Bodovi |
|---|---|
| naprijed | **gore** (cilj je iza, udaljava se) |
| okret u mjestu | **isti** (krajnja točka nepromijenjena) |
| mirovanje | isti, bez rizika |

…pa mirovanje pobjeđuje. Prvi cilj istraživanja je (-0.60, 0.58), a robot se pojavi u (0,0) okrenut
prema +x — cilj mu je na 136°, iza i lijevo. Zato je zastoj bio odmah, na prvoj dionici.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 17. 9. | profil mapiranja s `min_vel_x: 0.0` i `RotateToGoal.scale: 0.0` | **robot stoji 28 s**, nula grešaka | zabrana gibanja bez zamjenske ideje = zastoj |
| 2 | 17. 9. | dodane `PathAlign` (32) i `GoalAlign` (8) u profil mapiranja; `min_vel_x: -0.10` | 🧪 čeka vožnju | okretanje prema putanji se boduje, pa naprijed postaje jeftinije od unatrag |

## Ne ponavljati
- **Zabranjivati gibanje koje planer koristi, bez kritike koja nudi zamjensko gibanje.** Ako
  mirovanje ostane najbolje ocijenjeno, robot će stajati — i to bez ijedne poruke u logu, što je
  najgora vrsta kvara.
- Tumačiti „missed its desired rate of 20 Hz" kao uzrok zastoja: run u kojem je robot **vozio** imao
  je 451 takvu poruku, a ovaj u kojem je stajao 75. To je mjera opterećenja stroja, ne kvara.

## Otvoreno
Misijski profil i dalje nema `PathAlign`/`GoalAlign` i i dalje vozi unatrag kad mu tako ispadne.
**Ne dira se** dok je vožnja kroz vrata odvožena 5/5 ([[D-18_verified_baseline_first]]); ako se
poslije pokaže vrijednim, ide kao zaseban pokušaj s vlastitom vožnjom.
