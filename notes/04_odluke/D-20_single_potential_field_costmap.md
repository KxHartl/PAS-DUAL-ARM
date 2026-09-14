---
id: D-20
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-14_slam_mapping]]", "[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
problems: ["[[P-39_nav2_enters_doorway_at_an_angle]]", "[[P-40_amcl_pose_disagrees_with_lidar]]"]
superseded_by: ""
updated: 2026-09-14
---
# D-20 — Jedinstveno potencijalno polje i brazde za Nav2 navigaciju

## Kontekst
U runovima 63–68 dolazilo je do zapinjanja i konflikata oko stolova i prolaza. Analiza je otkrila tri neovisna izvora odbijanja s oprečnom semantikom:
1. Nav2 `inflation_layer` (izotropna tvrda zabrana `INSCRIBED 253` na 0.427 m oko svake laserske točke — generirala je cyan djetelinu oko nogu stola do 0.80 m te u vratima širine 0.98 m zasićivala NavFn potencijal).
2. Binarni keepout pravokutnici lijevaka kod vrata.
3. Graduirana cijena maske oko stola (`table_field`).

Robot je izrazito anizotropan (1.04 m duljina × 0.854 m širina u `ARM_CARRY_V2`), pa je izotropni radijus čeono bio preblag (dozvoljavao ulaz 12 cm u ploču stola), a bočno prestrog. Korisnik je zatražio jedinstvenu metodu potencijalnih polja: cilj privlači, prepreke (zidovi, stolovi) odbijaju, slobodan prostor je čist, s privlačnom „brazdom" nulte cijene kroz sredinu prolaza i prilaza.

## Odluka
1. **Jedinstvena ploha cijene u `nav_zones` (`Zones.field`)**:
   - Polje se računa izravno nad kartom pomoću `scipy.ndimage.distance_transform_edt`.
   - **Udaljenost se mjeri od stvarnog oboda ploče stola (0.80 m), a ne od nogu.** Lidar na 0.2086 m vidi samo noge, koje su 5 cm uvučene, pa bi polje građeno na njima bilo 5 cm preblizu po strani. Ploče se zato dodaju u skup prepreka prije transformacije udaljenosti.
   - Zauzete ćelije i pojas do `core` (0.427 m za planera) čine lethal jezgru (100).
   - Od `core` do `core + field_width` (**0.40 m**) pruža se glatka rampa s vrhom **`field_peak = 50`** (daje cost 127 u Costmap2D i NavFn cijenu 152 naspram 50 za slobodan pod — daleko ispod zasićenja 253).
   - **Obje vrijednosti su odabrane mjerenjem, ne procjenom.** `check_costmap_path.py --width/--peak` pokazuje da razmak putanje određuje *širina* rampe (putanja sjedne tik izvan nje), a vrh samo koliko čvrsto: 0.25/0.30/0.35/0.40 m dalo je 0.659/0.715/0.756/0.800 m, a s uključenim obodom ploče konačnih 0.40/50 daje **0.835 m**.
2. **Gašenje globalnog `inflation_layer`**:
   - Postavljeno `enabled: false` u `nav2_params.yaml`. Uklonjena je djetelina oko nogu i dvostruko brojanje prepreka.
3. **Brazde nulte cijene (`_furrows`)**:
   - Kroz prolaze vrata (širina `door.width - 2*lane_margin`, doseg ±1.45 m) i prilaznu traku stolu cijena rampe se reže na 0. Dovratnici i jezgre ostaju netaknuti, osiguravajući prohodan gradijentni spust NavFn planera (sprječava grešku runa 65: *Failed to create a plan from potential*).
4. **Docking i undocking geometrija**:
   - Definirana `dock` poza na stolu: `pola_ploče (0.401) + pola_duljine (0.52) + table_dock_safety (0.10) = 1.021 m` od centra stola.
   - Na dock pozi zabranjena je rotacija u mjestu (radijus rotacije 0.673 m bi zakačio stol).
   - `room_navigator` implementira sekvencu prilaza i obaveznog izlaska unatrag (`_undock_leg`) po istoj osi do `approach` poze (1.55 m) prije bilo kakvog manevra.
5. **Dva keepout filtera**:
   - `/keepout_filter_mask_planner` (s jezgrom proširenom za inscribed radijus 0.427 m za točkasti planer NavFn).
   - `/keepout_filter_mask` (bez jezgre, samo rampa i rubovi, za DWB koji sam rasterizira točan poligon otiska).

## Posljedice
- Eliminirane oprečne zone i višestruke boje u RViz-u oko stola.
- Putanje automatski drže **0.835 m** odmaka od stvarnih prepreka — iznad praga 0.52 m na kojem `collision_monitor` zaglavi (njegova projekcija pri `v → 0` i dalje seže pola duljine robota, run 64), a ispod 0.90 m na kojem putanje počinju ljubiti suprotni zid (run 68).
- Vrata i prilazi stolovima ostaju stabilni i prohodni.

## Što ovo NE jamči
Polje kaže gdje robot **voli** biti; ono ne može jamčiti da se granice robota i prepreke neće
preklopiti. To jamstvo i dalje daju **provjera otiska** (`ObstacleFootprint` u DWB-u nad
`voxel_layer`-om) i `collision_monitor`. Zato polje mora imati vlastitu lethal jezgru — NavFn
planira **točku** — a jezgra kod stola je postavljena anizotropno (bočno pola širine, čeono do
dock poze), jer izotropni radijus za robota 1.04 × 0.854 m ne može biti točan ni u jednom smjeru.

## Žive prepreke
Polje se gradi **iz karte**. Globalni costmap nema `obstacle` sloj, pa nemapirana prepreka ne
utječe na planiranje — hvata je samo točna provjera otiska u DWB-u i `collision_monitor`.
Proširenje polja na žive prepreke (dodati `nav2_costmap_2d::ObstacleLayer` i osvježavati polje u
petlji) je svjesno ostavljeno za kasnije (odluka korisnika, 14. 9.).
