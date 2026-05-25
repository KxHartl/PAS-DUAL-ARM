Cilj: Sinkronizirano prikupiti parove podataka potrebne za hand-eye kalibraciju
(RGB kameru sa slike, pozu robota (TCP))
• Teorijska pozadina:
• Za hand-eye kalibraciju potrebno je više različitih poza robota i pripadnih
slika kalibracijske mete
• Kamera slika ChArUco ploče
• Robot (UR) šalje TCP pozu [𝑥, 𝑦, 𝑧, 𝑅𝑥, 𝑅𝑦, 𝑅𝑧
] → potrebno pretvoriti u matricu
transformacija
UVOD
• Alati: koristi se pyrealsense2, OpenCV, NumPy, socket
• Napomena:
• Svaka slika odgovara točno jednoj robot pozi
• ChArUco ploča mora biti jasno vidljiva (CIJELA)
• Poze robota moraju biti što raznolikije (različiti kutovi i udaljenosti)

Zadatak:
1. Pokrenuti RealSense pipeline za color stream
2. Prikazati live sliku kamere (cv2.imshow)
3. Namjestiti željenu pozu robota, Pritiskom na SPACE spremiti trenutnu RGB sliku
4. Dohvatiti TCP pozu robota preko socket komunikacije
5. Pretvoriti TCP podatke [𝑥, 𝑦, 𝑧, 𝑅𝑥, 𝑅𝑦,𝑅𝑧
] u homogenu matricu transformacija, 4x4
koristeći cv.Rodrigues
6. Spremiti:
• Sliku kao .png u data folder
• robot pozu u robot_positions.txt
UPUTA
7. Ponoviti postupak za više različitih poza robota
8. Program završiti tipkom Q ili ESC