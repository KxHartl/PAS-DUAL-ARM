---
id: RUN_GRASP_POSE_DESIGN
type: upute
updated: 2026-09-15
---
# Definiranje poza hvata na simuliranom robotu

Korisnik, 15. 9.: *„napraviti launch, skriptu da se robot nalazi pred stolom, kutija na stolu,
ruke se nalaze u pripremnoj pozi gdje skeniraju kutiju sa strane. i onda cu ja definirati poze
za hvat i probamo ih."*

Jedan launch postavi scenu, a poze se zadaju u stupnjevima i šalju **pravim** kontrolerima
simulacije. Veza: [[P-43_grasp_pose_wider_than_door]], [[R-17_dual_arm_lift]],
[[R-19_door_pass_with_box]].

> [!warning] Alati iz ovog postupka nisu više u repozitoriju (16. 9. 2026.)
> `scripts/joint_gui.py`, `pose_studio.launch.py`, `grasp_stage.launch.py` i čvor `grasp_stage`
> ostali su **lokalno na disku**, ali su izašli iz repoa jer nisu dio gotove simulacije.
> Ovaj zapis ostaje kao dokumentacija **kako su nastale poze V4** koje misija danas koristi;
> same poze žive u `postures.py` i [[08_poze]]. Za pokretanje misije vidi `README.md`.

| što | vrijednost |
|---|---|
| spawn | dock poza `(0, −5.479)`, `yaw −90°` — ista kao misija i mapiranje, kocka 0.87 m ispred |
| ruke na spawnu | **`ARM_CARRY_V2`**, poza vožnje, mapiranja i lokalizacije |
| redoslijed | vodilice gore → ruke u skeniranje **na docku** → primicanje na 0.62 m. Obrnuto ne ide: uska poza bi kod kocke bila u ploči stola |
| vodilice | 0.200 m (mjereno, ne status akcije) |
| skenirajuća poza | otvorene šake, vodoravno, vrhovi prstiju **0.20 m** od bočnih ploha — poza iz runa 72 u kojoj obje kamere na zapešću vide svoj marker |
| kocka | otpuštena sa zapešća (`DetachableJoint`), slobodna na stolu |

> [!important] Najjednostavnije — samo simulacija i prozori, ništa se ne miče samo
> Korisnik, 15. 9.: *„pokrenuti simulaciju, gazebo prozor, rviz prozor, gui prozor za zglobove,
> gui prozor za spremljene poze, i sam otvaram teleop."* Robot se pojavi na dock pozi u
> `DRIVE_V4` (poza vožnje) i **stoji**. Jedino automatsko je jednokratno otpuštanje kocke sa zapešća.
>
> Terminal 1:
> ```bash
> bash scripts/run_cube_isolated.sh ros2 launch pas_dual_arm_bringup pose_studio.launch.py
> ```
> Terminal 2 (teleop, kad želiš):
> ```bash
> bash scripts/run_cube_isolated.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.3
> ```
> Prozor „spremljene poze": dvoklik ili **Ucitaj u klizace**, pa **Posalji** u prozoru sa
> zglobovima; **Spremi trenutnu pozu** upisuje pozu robota u [[08_poze]].
>
> MoveIt ovdje ne zna za stol ni kocku (nitko ih ne objavljuje) — **Posalji (MoveIt)** gleda samo
> sudare robota sa samim sobom.

> [!note] Svi terminali prvo
> ```bash
> cd /home/khartl/FSB/PAS-DUAL-ARM
> ```

> [!note] 1. Build (samo ako se mijenjao kod)
> ```bash
> ./scripts/run_native.sh colcon build --packages-select pas_dual_arm_scripts pas_dual_arm_bringup --symlink-install
> ```

> [!warning] Nikad dvije simulacije odjednom
> `run_cube_isolated.sh` sada odbija i drugi `grasp_stage.launch.py`, ne samo `sim.launch.py`.
> Ako javi da simulacija već radi, ugasi je u njezinom terminalu (Ctrl-C).

> [!note] 2. Terminal 1 — scena: Gazebo + RViz + MoveIt + skenirajuća poza
> ```bash
> bash scripts/run_cube_isolated.sh ros2 launch pas_dual_arm_bringup grasp_stage.launch.py |& tee log/stage-t1.log
> ```
> Čekaj redak **`STAGE_READY`** (3–4 min). Prije njega, izmjereno u headless runu S6:
>
> | redak | očekivano |
> |---|---|
> | `CARRIAGE MEASURED` | `0.2000 / 0.2000 m` |
> | `cube: ...` | duljina `0.300`, centar `(0.870, 0.000, 0.821)` — s dock poze |
> | `approach: ... advancing 0.250 m` | koliko će se primaknuti, **nakon** što ruke odu u skeniranje |
> | `left/right scan pose: ... REACHED` | ~7 mm od cilja |
> | `drove in 0.254 m with the hands in the scan pose` | primicanje |
> | `SYMMETRIC: right arm is the mirror of the left` | `0.00 mm` — desna ruka je zrcalo lijeve |
> | `WRIST left/right: face centre` | y **±0.150**, z ~0.824 — kamera na zapešću vidi svoju plohu |
> | `WRIST cameras: faces ... apart` | **0.300 m** (kocka je 0.30), centar iz kamera na zapešću ~1–3 mm od odometrije |
> | `SCAN_LEFT / SCAN_RIGHT` | zglobovi skenirajuće poze, radijani i stupnjevi |
>
> `WIDTH MEASURED [scan pose]` javlja ~1.4 m, `TOO WIDE` — to je poza **skeniranja**, ne hvata;
> ruke su vodoravno i 0.20 m od ploha. Robot se u njoj ne vozi.
>
> Ako umjesto toga piše **`STAGE FAILED: ...`**, poruka kaže koji korak je pao; ruke se tada ne
> miču dalje.

> [!note] 3. Terminal 2 — prozor sa zglobovima, upravlja simulacijom
> ```bash
> bash scripts/run_cube_isolated.sh python3 scripts/joint_gui.py --sim |& tee log/stage-t2-gui.log
> ```
> Klizači se pune **stvarnom** pozom robota. Upisuješ stupnjeve (vodilice u mm), pa šalješ:
>
> | gumb | što radi |
> |---|---|
> | **Posalji (MoveIt)** | vodilice → hvataljke → ruke, putanja **s provjerom sudara** |
> | **Posalji direktno** | isto, ali ruke idu ravno na kontrolere — **bez provjere sudara** |
> | **Preuzmi s robota** | klizači na trenutačnu pozu robota |
> | **Zrcali lijevu -> desnu** | desna ruka = **zrcalna slika** lijeve (vidi niže) |
> | panel **Šaka po osima** | šaka ±X/±Y/±Z i zakretanja, lakat uz zamrznutu šaku (vidi niže) |
>
> Kod „Obje simetrično" prvi klik može pomaknuti desnu šaku više od koraka: ona se postavlja u
> točno zrcalo lijeve, a u simulaciji je nakon postava ~9 mm od njega (praćenje). Status tada piše
> „desna poravnata u zrcalo lijeve"; svaki sljedeći korak je točan (G2: +5.0 / +5.0 mm).
> | **Provjeri** | u terminal: koji jastučić dira kocku, širina robota prema vratima, položaj šaka, vodilice, zglobovi |
> | **Ispisi pozu** | zglobovi u terminal |

> [!note] Šaka po osima (panel ispod gumba)
> - **Lijeva / Desna / Obje simetrično** — kod „obje" se pomiče lijeva, a desna postaje njeno
>   zrcalo: `+Y` = obje van, `−Y` = obje prema sredini (prema kocki).
> - **osi baze** (X naprijed, Y lijevo, Z gore) ili **osi alata** (Z = os prilaza, prema plohi).
> - **korak** u mm za pomak, u ° za zakretanje i lakat.
> - `±X ±Y ±Z` pomiču šaku, `±RX ±RY ±RZ` je zakreću; ostatak ruke slijedi.
> - **Lakat prema unutra / van (šaka stoji)** — šaka je zamrznuta u prostoru, a ruka se zakrene
>   oko pravca rame–zapešće. S 7 zglobova i potpuno zamrznutom šakom to je **jedini** slobodni
>   pomak. „Unutra" znači prema sredini robota, što je smjer koji pomaže kroz vrata.
> - **odmah pošalji (direktno)** — svaki klik ide ravno na kontrolere (~1 s). Isključi ga pa
>   pošalji tek na kraju, preko MoveIt-a.
>
> Pomaci idu malim koracima po Jacobianu ruke, pa ruka ne može preskočiti u drugu granu. Korak
> koji bi izveo zglob iz granica ili preokrenuo ruku bude **odbijen** i to piše u status traci.
> Zglobovi 2, 4 i 6 drže se barem 4° od graničnika pomoću sedmog zgloba. Zato se ruka na
> **prvom** kliku iz poze tik uz graničnik jednom odmakne (~10° po zglobovima), a svaki sljedeći
> korak je mali. Ako lakat ne može dalje, status kaže i koji zglob je na graničniku — npr. u
> skenirajućoj pozi „lakat ne može dalje prema unutra – zglob 6 (−116°)"; prema van ide.

> [!warning] Poza u dodiru s kockom ide samo „direktno"
> MoveIt odbija cilj koji je u sudaru s kockom u sceni, a poza hvata je upravo to. Prilaz do
> blizu kocke šalji preko MoveIt-a, a zadnji korak na plohu **direktno**. Direktno ne gleda ni
> stol ni kocku: ruka ide ravno u zadane kutove.

> [!important] Zrcaljenje je sada točno — i kamera ostaje gore
> Stari gumb je kopirao iste kutove na desnu ruku. Na ovom robotu to **nije** zrcalo: desna ruka
> je montirana zrcalno, pa isti kutovi šalju šaku drugamo. Ni sama promjena predznaka nije
> dovoljna: orijentacija šake tada štima, ali položaj promaši **20–45 mm** (Gen3 ima male bočne
> pomake u zglobovima). Gumb zato zrcali **pozu lijeve šake** preko sredine robota i za nju riješi
> desnu ruku: **0.000 mm**.
>
> Čisto zrcalo bi desnu **kameru na zapešću** stavilo s druge strane osi prilaza (kamera nije
> zrcalno montirana), pa ne bi vidjela marker (run S3). Desna šaka se zato još okrene za **180°
> oko osi prilaza** — to je samo zglob 7, pa j1–j6 ostaju zrcalni i ruka izgleda simetrično, a
> Robotiq s dva jednaka prsta hvata jednako. Ako IK ne konvergira ili neki zglob izađe iz granica,
> status traka to kaže.

> [!important] Širina: poza hvata mora proći vrata
> **Provjeri** ispisuje `sirina robota ... (vrata 1.00 m -> PROLAZI / NE PROLAZI; ARM_CARRY_V2
> 0.840 m)`. Vodoravni hvat ne može ispod **1.003 m** — sferno zapešće leži na osi prilaza.
> Nagib prilaza prema dolje to rješava: 45° → 0.847 m, 50° → 0.823 m
> ([[P-43_grasp_pose_wider_than_door]]). Širina je `NEPOUZDANO` ako TF nije smjestio sve linkove —
> tada ponovi Provjeri.

> [!note] Zglobovi 1, 3, 5, 7 okreću se bez graničnika
> Robot zna javiti npr. 294° (5.128 rad). Prozor ga prikazuje kao −66°, a šalje **najbliži**
> ekvivalent trenutnom kutu — nikad puni krug. Prije ovog popravka isti bi zglob bio odrezan na
> 180° i ruka bi se okrenula za 114°.

> [!note] Vožnja za vrijeme definiranja poza — Terminal 4
> ```bash
> bash scripts/run_cube_isolated.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.3
> ```
> Radi tek **nakon `STAGE_READY`** — prije toga bazu vozi `grasp_stage`.
>
> | tipka | pomak |
> |---|---|
> | `i` / `,` | naprijed / nazad |
> | `j` / `l` | okret lijevo / desno |
> | **Shift + `J` / `L`** | **bočno** lijevo / desno, bez okreta (mecanum) |
> | Shift + `U` `O` `M` `>` | dijagonalno |
> | `k` | stop |
> | `q`/`z` | brzina ×1.1 / ×0.9 |
>
> `speed:=0.1` (m/s) i `turn:=0.3` (rad/s) su namjerno sporo: ruke su izvan baze i blizu stola.

> [!warning] Ruke se voze zajedno s bazom
> U skenirajućoj pozi šake strše ~0.5 m bočno, a prsti su 0.20 m od kocke. Ništa ne zaustavlja
> robota pred stolom ili kockom — gledaj Gazebo. Ako šake guraju kocku, `grasp_stage` to ne zna.

> [!important] MoveIt scena prati vožnju
> MoveIt planira u koordinatama samog robota, pa bi stol i kocka u sceni inače putovali s robotom.
> Nakon `STAGE_READY` `grasp_stage` ih drži na mjestu preko **odometrije**: u terminalu 1 piše
> `SCENE FOLLOWS THE BASE`, a za vožnje `scene moved with the base: cube now (...)`. Odometrija
> kliže s kotačima, a pomaknuta kocka se ne prati — nakon dulje vožnje ili guranja kocke ponovi
> postav (Ctrl-C u terminalu 1 i isti launch).

> [!note] 4. Terminal 3 — spremi pozu u registar
> ```bash
> bash scripts/run_cube_isolated.sh python3 scripts/capture_posture.py GRASP_V1 "hvat kocke, opis"
> ```
> Upisuje `GRASP_V1_LEFT` / `GRASP_V1_RIGHT` i visinu vodilica u [[08_poze]].

> [!note] 5. Isprobaj spremljenu pozu
> ```bash
> bash scripts/run_cube_isolated.sh python3 scripts/joint_gui.py --sim --poza GRASP_V1 |& tee log/stage-t2-gui.log
> ```
> Poza se učita u klizače, a robot se miče tek na gumb **Posalji**.

> [!note] 6. Gašenje
> Ctrl-C u terminalu 2, pa u terminalu 1.

## Što zabilježiti po pozi
- ime poze i `Provjeri` ispis (jastučići, širina, šake, vodilice);
- je li kocka **ostala na mjestu** ili ju je ruka gurnula (vidi se u Gazebu);
- red u [[runovi]].
