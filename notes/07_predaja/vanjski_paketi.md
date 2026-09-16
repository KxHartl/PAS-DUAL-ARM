---
id: VANJSKI_PAKETI
type: registar
updated: 2026-09-16
---
# Vanjski paketi i tuđi materijal (izvor za seminar)

> Za poglavlje „Korišteni resursi". Pravilo: tuđe se **citira u radu** *i* **referencira u gitu**;
> jedno ne zamjenjuje drugo. Sve je iz [[izvori]] ([MAIL], 4. 5. 2026.).

## 1. Paketi koji se skidaju od autora (nisu u repou)

U gitu postoji samo manifest `ros2.repos`; `vcs import src < ros2.repos` klonira ih s GitHuba
autora na **pinani commit**. Repozitorij ne redistribuira ni bajt njihovog koda.

| Paket | Autor | Licenca | Commit | Čemu služi | Izvor |
|---|---|---|---|---|---|
| `omni_base_simulation` | PAL Robotics | Apache-2.0 | `77248ac` | mobilna baza: geometrija, kotači, lidar | [MAIL] |
| `ros2_kortex` | Kinova | BSD | `116d87a` | Kinova Gen3 ruke + Robotiq 2F-85 | [MAIL] |
| `pan_tilt_ros` | I-Quotient-Robotics | MIT | `b0f6534` | pan-tilt mehanizam | [MAIL] |
| `realsense-ros` | Intel RealSense | Apache-2.0 | `6d87b07` | RealSense D435 | [MAIL] |
| `aruco_ros` | PAL Robotics | MIT | `86a0bbb` | ArUco; koristi se **vlastiti** detektor ([[D-02_own_aruco_detector]]) | [MAIL] (prijedlog) |

Naš repozitorij je Apache-2.0 → sve navedene licence su kompatibilne.

## 2. Tuđi materijal koji se isporučuje s repoom

| Što | Autor | Gdje | Napomena |
|---|---|---|---|
| STL vodilica i torza (`dual_arm_torso-main.zip`) | **Branimir Ćaran** | `src/dual_arm_torso/meshes/` | prilog uz [MAIL]; korišteno uz dopuštenje autora zadatka |
| CAD slika s mjerama torza | **Branimir Ćaran** | `data/raw/zadatak/torzo_cad_mjere.png` | izvor za dimenzije u xacro-u |
| Slika ciljanog izgleda sustava | **Branimir Ćaran** | `data/raw/zadatak/mail_img-000.png` | [MAIL-slika], obvezuje po izgledu |
| Tekst zadatka | **Branimir Ćaran** | `data/raw/zadatak/task_caran_2025-11-30.pdf` | [ZAD] |

Ovo je jedina kategorija gdje repo **redistribuira** tuđe datoteke, pa atribucija stoji i u
`src/dual_arm_torso/README.md`, `src/dual_arm_torso/package.xml` i glavnom `README.md` §9.

## 3. Izmjene tuđeg koda

Jedna, i dokumentirana je:

| Zakrpa | Meta | Što radi | Zašto |
|---|---|---|---|
| `patches/ros2_kortex-robotiq_2f_85-drop-isaac-args.patch` | `kortex_description/grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro` | miče `sim_isaac`, `isaac_joint_commands`, `isaac_joint_states` | ti argumenti ne postoje na pinanoj Humble grani pa xacro puca |

Primjenjuje je `scripts/apply_patches.sh` (idempotentno, provjerava je li već primijenjena).

## 4. Kako to napisati u seminaru

- Navesti paket, autora, licencu i **pinani commit** (reproducibilnost je argument, ne detalj).
- Jasno razdvojiti: *što je preuzeto* (baza, ruke, pan-tilt, kamera, STL vodilica) od *što je naše*
  (integracija u jedan URDF, svijet, kontroleri, zone, `main_task`, percepcija, misija).
- Spomenuti zakrpu — pokazuje da je preuzeto provjereno, a ne slijepo uključeno.
