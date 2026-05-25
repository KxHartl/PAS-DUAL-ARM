Cilj: Odrediti 3D koordinatu objekta u koordinatnom sustavu baze robota
korištenjem RGB slike, depth kamere, hand-eye kalibracije, trenutačne TCP poze
robota. Klikom na objekt u slici izračunava se njegova stvarna pozicija u prostoru.
• Teorijska pozadina:
• Za odabrani piksel 𝑢, 𝑣 i dubinu 𝑍 računa se 3D točka u koordinatnom
sustavu kamere
• Transformacija u bazu robota – koristi se odnos dobiven u hand-eye
kalibraciji, a konačno rješenje je: 𝐵 𝑻𝐶 = 𝐵 𝑻𝑇𝐶𝑃
𝑇𝐶𝑃 𝑻𝐶
• Da dobijemo položaj predmeta u bazi robota: 𝑝𝐵 = 𝐵 𝑻𝐶 𝑝𝐶
UVOD
• Alati: koristi se pyrealsense2, OpenCV, NumPy, socket

Zadatak:
1. Učitati intrinzične parametre kamere (npy. datoteke iz 1. zadatka)
2. Učitati rezultate hand-eye kalibracije (npy. datoteke iz 4. zadatka)
3. Pokrenuti RealSense kameru (color i depth stream)
4. Poravnati depth sliku s RGB slikom (align = rs.align(rs.stream.color))
5. Prikazati live sliku kamere
6. Klikom miša odabrati objekt u slici
7. Očitati depth vrijednost odabranog piksela (depth_frame.get_distance(u,v))
8. Iz piksela 𝑢, 𝑣 i depth podatka izračunati 3D koordinatu objekta u sustavu kamere
9. Dohvatiti trenutačnu TCP pozu robota preko socket komunikacije
10. Pretvoriti TCP pozu u homogenu 4x4 matricu (tcp_to_4x4(...))
11. Primijeniti hand-eye transformaciju i izračunati koordinatu objekta u bazi robota
12. Prikazati rezultate: dubinu objekta, X koordinatu, Y koordinatu, Z koordinatu
13. Resetirati odabir tipkom R
14. Završiti program tipkom Q ili ESC.
15. Provjeriti valjanost rezultata s nastavnim osobljem

