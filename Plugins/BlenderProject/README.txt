================================================================================
  Blender mocap test project — комплекс для cross-log verification with Fox_Mocap
================================================================================

ЦЕЛЬ
----
Этот проект — независимый "глаз" на стрим Fox_Mocap backend'а. Получает MXTP02
поток через UDP, применяет к armature, и пишет ДЕТАЛЬНЫЙ лог alllog.txt в формате,
максимально близком к Fox_Mocap `-test -gloves`. По 2 логам можно diff'ить:
  • что отправлено vs что получено (orientation, sample IDs, positions);
  • mirror-symmetry на правой/левой сторонах;
  • bone application errors;
  • packet loss / jitter.

ФАЙЛЫ
-----
• testproject.blend    — Blender сцена с MVN:Actor armature (23 кости).
• setup_testproject.py — Headless setup script (создаёт armature заново).
• alllog.txt           — Лог-файл (пишется автоматически при работе плагина).

ИНСТРУКЦИЯ ЗАПУСКА
------------------
1. Откройте testproject.blend в Blender.
2. Включите плагин: Edit → Preferences → Add-ons → "MVN Live Plugin" (если не вкл.).
3. Откройте sidebar (View3D → N) → вкладка Xsens.
4. В панели "MVN Live Streaming":
   • Stream Address: localhost
   • Stream Port: 9763
   • Нажмите "Start Stream" (зелёная кнопка).
5. Запустите Fox_Mocap backend и нажмите "Live → Start".
6. После handshake армируется (плагин примет scale packet и создаст target skel).
7. alllog.txt начнёт писаться автоматически рядом с .blend файлом.

ФОРМАТ ЛОГА (mirroring Fox_Mocap backend)
-----------------------------------------
  • [stream reset] counters cleared t=X.XXXs
  • [stream start] ip=... port=...
  • [blender meta] {dict}
  • [blender packet] msgId=02 sample=N dgCounter=128 segCount=23 ft=Xms t=X.Xs recv#N
  • [stream Δpelvis] dxy=X.XXXm dz=±X.XXXm pelvisM=(...) sample=N
  • [stream tick=N] pelvisM=(...) qPelvis=(...) qRHand=(...) qLHand=(...)
  • ========== [BLENDER SNAPSHOT] t=X.XXs packet#N ==========
       pelvisM (received NWU) = (...)
       recv stats: total_packets=N quat_msgs=N
       --- Per-segment received quaternion (NWU as on wire) ---
         seg[ 1] Pelvis           quat=(...) angle_from_identity=  X.XX°
         ... (23 + optional fingers)
       --- s2s pair-symmetry check (L vs mirror_y(R)) ---
         pair seg[ 8/12] R.w=...  L.w=...  devMirr=...°  devPar=...°  best=MY
         ... (8 pairs)
  • [blender fk-jump] BoneName jump=X.X°    (когда detect >35° per-frame)
  • [blender bone-dump]                     (periodic dump всех bone world rot+loc)

ПЕРИОДИЧНОСТЬ
-------------
  • [blender packet]    — первые 5 + every 60th packet (mirroring backend cadence).
  • [BLENDER SNAPSHOT]  — every ~2.0s (matching backend's RENDER SNAPSHOT cadence).
  • [stream tick=]      — every 240th quat-message.
  • [stream Δpelvis]    — when dxy > 3cm OR |dz| > 3cm.

DIFF С Fox_Mocap LOG (fox_mocap.log)
------------------------------------
Маркеры идентичны:
  • "[BLENDER SNAPSHOT]" ≈ Fox_Mocap "[RENDER SNAPSHOT]"
  • "pair seg[X/Y]" — buy-for-buy совместимый с Fox_Mocap line 9540 sym check
  • "[stream Δpelvis]" — same format
  • "[stream tick=]" — same format
Просто diff'ьте оба файла per-line; расхождения видны мгновенно.

ВНУТРЕННЯЯ АРХИТЕКТУРА
----------------------
1. receiver.py (patched): hooks alllog.on_packet_received / on_header_decoded /
   on_quaternion_message / on_metadata / on_scale_message для каждой message.
2. pose.py: не patched напрямую (используется existing skeleton_transform_dict).
3. source_animator.py (patched): hook alllog.on_bone_applied для каждой кости
   после применения rotation_quaternion → detect fk-jumps.
4. alllog.py (new module): главный logger. Авто-resolves log path next to .blend.

================================================================================
