---
id: R-10
type: zahtjev
status: ispunjeno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-02_world_and_sim_launch]]"]
problems: []
decisions: []
updated: 2026-09-13
---
# R-10: Gazebo okruženje koje robot može mapirati

## Izvor (doslovno)
> „Potrebno je napraviti simulacijsko okruženje u Gazebo-u koje će robot mapirati“ [MAIL]

## Tehnički znači
Svijet ima dovoljno statične strukture (zidovi, stolovi) da ga 2D lidar vidi i da slam_toolbox
složi kartu. Robot ima lidar koji objavljuje `/scan`.

**Kriterij prihvaćanja:**
- [x] `seminar_world.sdf`: zid s prolazom (x = 2.0), `pick_table`, `place_table`, `target_table` (x = 4.0), kutija
- [x] `gpu_lidar` 360 zraka, 10 Hz → `/scan`
- [ ] (samo mapiranje je [[R-14_slam_mapping]])

## Trenutno stanje
✅ Svijet i lidar postoje od 12. 6. (`ae60d9b`). SLAM je nad njim radio 23. 6. (M5).

## Kako se rješava
- [[S-02_world_and_sim_launch]]
