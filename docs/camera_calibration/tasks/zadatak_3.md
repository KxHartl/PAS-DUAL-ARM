Cilj: Na temelju prethodno snimljenih slika odrediti položaj i orijentaciju ChArUco
ploče u odnosu na kameru
• Za svaku sliku izračunava se: translacija i rotacija mete te se prikazuju
koordinatne osi mete na slici
• Teorijska pozadina:
• ChArUco ploča kombinira ArUco markere (jedinstveni ID) i chessboard
kutove (veća preciznost)
UVOD
• Alati: koristi se pyrealsense2, OpenCV, NumPy, glob, natsort
• Napomena:
• Za uspješnu procjenu poze potrebno je:
• dovoljno vidljivih markera
• minimalno 6 ChArUco kutova
• dobra osvijetljenost slike
• točni intrinzični parametri kamere 

Zadatak:
1. Učitati intrinzične parametre kamere (npy. datoteke iz 1. zadatka)
2. Definirati ChArUco ploču s poznatim dimenzijama (squareLength = 0.029 m,
markerLength = 0.015 m)
3. Učitati sve slike iz mape snimljene u 2. zadatku
4. Za svaku sliku izvršiti detekciju ArUco markera (cv2.aruco.detectMarkers(...))
5. Interpolirati ChArUco kutove (cv2.aruco.interpolateCornersCharuco(...))
6. Ako je detektirano dovoljno kutova, procijeniti pozu ploče
(cv2.aruco.estimatePoseCharucoBoard(...))
7. Izdvojiti translaciju i rotaciju
8. Nacrtati koordinatne osi mete na slici (cv2.drawFrameAxes(...))
9. Spremiti rezultate
• Tekstualne koordinate u koordinate.txt
• Obrađene slike u dana/results mapu
UPUTA
10. Prikazati rezultate i ponoviti za sve slike