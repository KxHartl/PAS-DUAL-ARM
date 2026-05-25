Cilj: Odrediti točnu transformaciju između kamere i TCP-a robota
korištenjem prethodno prikupljenih slika i robot poza
• Rezultat kalibracije je odnos TCP-a i kamere
• Teorijska pozadina:
• Hand-eye kalibracija rješava problem određivanja položaja kamere
montirane na robotskoj ruci
• Koristit će se Tsai-Lenz metoda: cv2.CALIB_HAND_EYE_TSAI
UVOD
• Alati: koristi se OpenCV, NumPy, glob, natsort
• Napomena:
• Ako dobijete krive rezultate potrebno se vratiti i provjeriti točnost
zadataka 1, 2 i 3

Zadatak:
1. Učitati intrinzične parametre kamere (npy. datoteke iz 1. zadatka)
2. Učitati sve slike iz mape
3. Učitati spremljene robot poze iz datoteke
4. Definirati ChArUco ploču s poznatim dimenzijama
5. Za svaku sliku detektirati ChArUco metu i izračunati pozu mete u odnosu na kameru
6. Formirati parove podataka:
• Poza robota (rotacija, translacija)
• Poza mete (rotacija, translacija)
UPUTA
7. Provjeriti broj valjanih parova (minimalno 5)
8. Pokrenuti hand-eye kalibraciju (cv2.calibrateHandEye(...))
9. Koristiti Tsai-Lenz metodu (cv2.CALIB_HAND_EYE_TSAI)
10. Dobivenu transformaciju pretvoriti u 4x4 matricu
11. Izračunati i inverz te spremiti rezultate kao .npy datoteke:
• T_tcp_from_cam.npy
• T_cam_from_tcp.npy
