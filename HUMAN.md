# Vizualna Verifikacija (Zatraženo od AI Agenta)

Trenutno sam u procesu generiranja URDF modela robota (`robot.urdf.xacro`). Budući da kao agent nemam pristup grafičkom sučelju (GUI), molim vas da pokrenete sljedeće naredbe kako bismo vizualno potvrdili da su dijelovi (baza, vodilice, 2x ruke, kamera) ispravno spojeni.

### Koraci za pokretanje verifikacije:

1. Otvorite novi terminal.
2. Pozicionirajte se u workspace i pokrenite build:
   ```bash
   cd ~/FSB/PAS-DUAL-ARM
   colcon build --symlink-install
   source install/setup.bash
   ```
3. Pokrenite testnu launch datoteku koja će otvoriti **RViz2** i **Joint State Publisher GUI**:
   ```bash
   ros2 launch pas_dual_arm_bringup display.launch.py
   ```

### Što trebate vidjeti i javiti mi:
- U RViz-u, dodajte `RobotModel` i postavite Fixed Frame na `base_link` ili `base_footprint`.
- **Vodilice (Torzo)**: Trebale bi biti u sredini na bazi.
- **Ruke (Kinova)**: Lijeve i desne ruke bi trebale biti spojene na male crne klizače na vodilicama. Pomičući slidere u `Joint State Publisher GUI` prozoru za `torso_left_carriage_joint` i `torso_right_carriage_joint`, ruke bi se trebale pomicati gore-dolje!
- **Kamera**: Provjerite je li kamera na vrhu (preko pan-tilt mehanizma) orijentirana ispravno (gleda naprijed).

**Ako nešto ne izgleda u redu** (npr. ruke su "u zraku" pored robota, ili baza lebdi), molim vas opišite mi što vidite (npr. "lijeva ruka je previše ulijevo, X pomak je kriv") pa ću korigirati ofsete u URDF-u!

---

## Sljedeći Korak: MoveIt2 Konfiguracija
Nakon što potvrdite da URDF vizualno izgleda dobro, moramo generirati MoveIt2 paket. Budući da generiranje zahtjeva grafičko sučelje (MoveIt Setup Assistant), molim vas da pokrenete sljedeće:
```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```
1. Odaberite **Create New MoveIt Configuration Package**.
2. Učitajte `src/pas_dual_arm_bringup/urdf/robot.urdf.xacro`.
3. Izgenerirajte Self-Collision matrix.
4. Definirajte Planning Groups: `left_arm`, `right_arm`, i `dual_arm` (koja sadrži obje ruke i torzo).
5. Spremite paket u `src/pas_dual_arm_moveit_config`.

Javite mi kad ste to obavili, ili ako imate grešaka!
