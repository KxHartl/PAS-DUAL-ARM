---
id: R-10
type: zahtjev
status: ispunjeno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-02_world_and_sim_launch]]"]
problems: ["[[P-36_walls_lower_than_camera]]"]
decisions: ["[[D-13_three_room_world]]"]
updated: 2026-09-13
---
# R-10: Gazebo okruženje koje robot može mapirati

## Izvor (doslovno)
> „Potrebno je napraviti simulacijsko okruženje u Gazebo-u koje će robot mapirati“ [MAIL]

## Tehnički znači
Svijet ima dovoljno statične strukture (zidovi, stolovi) da ga 2D lidar vidi i da slam_toolbox
složi kartu. Robot ima lidar koji objavljuje `/scan`.

**Kriterij prihvaćanja:**
- [x] `seminar_world.sdf`: **tri sobe u L** (HOME sivo, KUTIJA plavo, ODREDIŠTE crveno), zidovi
  1.2 m, vrata 0.9 m ([[D-13_three_room_world]])
- [x] `gpu_lidar` 360 zraka, 10 Hz → `/scan`
- [x] GUI potvrda novog svijeta (korisnik, 13. 9.)
- [ ] (samo mapiranje je [[R-14_slam_mapping]])

## Trenutno stanje
✅ Svijet s tri sobe je napravljen i u GUI-ju potvrđen 13. 9. Lidar postoji od 12. 6. (`ae60d9b`).
SLAM je radio 23. 6. (M5) nad starim svijetom.

## Kako se rješava
- [[S-02_world_and_sim_launch]]
