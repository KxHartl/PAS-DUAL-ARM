# Dual Arm Torso - Geometrijski Odnosi i Skale (Ažurirano)

Nakon analize fotografije, jasno je da je torzo postavljen tako da mu je **najdulja stranica usmjerena prema naprijed (X-os)**, a ruke su montirane na **lijevu i desnu stranu (široke plohe)**. 

## 1. Orijentacija Torza
- **Dimenzije STL-a:** X=566.6mm, Y=284.0mm, Z=956.0mm.
- **Korekcija:** Torzo mora biti na `rpy="0 0 0"`. Na taj način njegova X-os (566mm) gleda naprijed, a Y-os (284mm) ide lijevo-desno.

## 2. Pozicioniranje Klizača (Carriages)
Klizači se nalaze na centralnim stupovima širokih ploha (Lijevo = +Y, Desno = -Y).
Dimenzije klizača: Širina X=120mm, Dubina Y=53mm (od 20.5 do 73.5), Visina Z=214mm.

**Lijevi klizač (+Y strana):**
- Montira se na Y=285.4mm plohu.
- Njegova ravna strana je na lokalnom Y=20.5mm.
- Rotacija: `rpy="0 0 0"`
- Prijenos (Origin): `X = -0.086m`, `Y = 0.265m`, `Z = 0.05m`.

**Desni klizač (-Y strana):**
- Montira se na Y=1.4mm plohu.
- Mora gledati u suprotnom smjeru, pa ga rotiramo za 180° oko Z osi torza (`rpy="0 0 3.14159"`).
- Prijenos (Origin): `X = 0.035m`, `Y = 0.022m`, `Z = 0.05m`.

## 3. Montiranje Ruku
Baza Kinova ruku ide na vanjsku površinu klizača. Vanjska površina je na lokalnom Y = 73.5mm (0.0735m).
Centar cilindra klizača je na X = 60.6mm (0.06m) i Z = 112.5mm (0.112m).
Dakle, lokalni ishodišni prijenos baze svake ruke u odnosu na klizač iznosi:
- `xyz="0.060 0.0735 0.112"`
- Ruke rotiramo za `rpy="-1.5708 0 0"` kako bi izlazile prema van (lokalna Z-os ruke se poravnava s lokalnom Y-osi klizača). Desna ruka, s obzirom na to da je montirana na rotirani klizač (180° oko Z osi), prirodno ostvaruje i traženu rotaciju od 180° u globalnom koordinatnom sustavu.
