Zadatak 1:
• Cilj: Dobiti intrinzične parametre RGB kamere bez ručne kalibracije
• Teorijska pozadina:
• Intel RealSense kamere već imaju ugrađenu kalibraciju
• Moguće je direktno očitati:
• rezoluciju slike
• žarišne duljine (𝑓𝑥, 𝑓𝑦)
• optičko središte (𝑐𝑥, 𝑐𝑦)
• koeficijente distorzije
UVOD
• Alati: koristi se pyrealsense2
• Napomena:
• Nije potrebno provoditi kalibraciju (kamera je već kalibrirana)
• Parametri se čitaju direktno iz uređaja

Zadatak:
1. Pokrenuti RealSense pipeline za color stream
2. Dohvatiti aktivni profil kamere
3. Iz profila dohvatiti video stream profil
4. Iz video stream profila očitati intrinzične parametre (pristupate varijabli intr)
5. Konstruirati intrinzičnu matricu kamere (3x3) i vektor distorzije
• np.array([*popunite*], dtype=np.float64)
6. Ispisati:
• Rezoluciju (u pikselima)
• 𝑓𝑥, 𝑓𝑦 (u pikselima)
• 𝑐𝑥, 𝑐𝑦 (u pikselima)
• distorzijske koeficijente 𝑘1, 𝑘2 , 𝑝1, 𝑝2, 𝑘3
• intrinzičnu matricu kamere
• vektor distorzije
UPUTA
Utjecaj distorzije je kod RealSense
kamera unaprijed ispravljen
Očekivano rješenje:
7. Spremiti intrinzičnu matricu i distorzijske koeficijente
kao .npy datoteke