# `dual_arm_torso`

Opis torza i vertikalnih linearnih vodilica (mesh + xacro) na kojima su montirane obje ruke.

## Porijeklo STL-ova

**Autor STL modela vodilica i torza: Branimir Ćaran.** Datoteke su stigle kao prilog
(`dual_arm_torso-main.zip`) uz mail od 4. 5. 2026. u kojem je zadan ovaj projekt, zajedno s CAD
slikom s mjerama (`data/raw/zadatak/torzo_cad_mjere.png`). Korišteno uz dopuštenje autora zadatka.

Ovaj paket je time jedini dio repozitorija koji **isporučuje tuđi materijal**; svi ostali vanjski
paketi skidaju se od autora preko `ros2.repos` (vidi [`README.md`](../../README.md) §9).

| Datoteka | Što je |
|---|---|
| `meshes/assembly_main_simplified.STL` | torzo s vodilicama (pojednostavljeni sklop) |
| `meshes/mts_carriage.STL` | klizač po vodilici |
| `meshes/wedge.stl` | podložni klin |
| `urdf/dual_arm_torso.urdf.xacro` | naš opis: zglobovi, granice, inercije |

Mase u xacro-u (12 kg vodilica, 2 kg klizač) su **procjena** — proizvođačevi podaci nisu dostupni;
vodi se kao svjesno odstupanje u `notes/07_predaja/odstupanja.md`.
