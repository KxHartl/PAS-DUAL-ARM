---
id: D-07
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-17_dual_arm_lift]]", "[[R-19_door_pass_with_box]]"]
problems: ["[[P-17_detachable_joint_explodes]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-07: Nakon attacha desna ruka popušta, nosi samo lijevi zglob

## Kontekst
S kutijom kruto spojenom na lijevi zglob, desna ruka koja je i dalje stiskala i gibala se svojom
putanjom „gurala“ je fiksiranu kutiju. DetachableJoint solver je eksplodirao i kutija je odletjela
(nedeterministički, i u GUI-ju i headless).

## Odluka (30. 6., `5c9e201`)
Nakon attacha **desna** ruka otvori/odmakne se **prva** (danas linearno 0.10 m po −v, s RRT
fallbackom). Dizanje i spuštanje radi samo **lijeva** ruka. Pravilo: **nikad dvije krute veze na
kutiji istovremeno.**

## Posljedice
Kocka visi na lijevom zglobu. Pri vožnji baze to je konzolni teret, a to je vjerojatno povezano s
[[P-18_transport_drops_box]].

## Odnos prema zahtjevima
[[R-17_dual_arm_lift]]: hvat i odvajanje od stola rade obje ruke, a nošenje jedna + spoj. U
seminaru to iskreno opisati.
