Zadatak 5:
Slanje trajektorija na robota (simulacija):
• Potrebno je implementirati Python skriptu koja će ostvariti komunikaciju s UR
robotom putem TCP/IP-a te izvršiti unaprijed generiranu trajektoriju u zglobnom
prostoru (robot simuliran – PolyScope)
• Zadatak uključuje sljedeće korake

1. učitati trajektoriju iz .txt datoteke (servoj naredbe u radijanima)
2. uspostaviti socket vezu s robotom (real-time port)
3. poslati trajektoriju kao URScript program
4. omogućiti da robot izvršava trajektoriju vlastitom kontrolnom frekvencijom
   • Napomena: kod implementacije koda na pravom robotu paziti da je početna
   konfiguracija namještena u početku slanja programa
   𝑝𝑖
   , 𝑅𝑖
   • Potrebno je pustiti datoteku generiranu u četvrtom zadatku i usporediti ponašanje
   robota
