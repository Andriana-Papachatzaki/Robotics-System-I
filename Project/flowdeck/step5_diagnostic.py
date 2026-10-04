"""
ΔΙΑΓΝΩΣΤΙΚΗ έκδοση του ΒΗΜΑΤΟΣ 5 -- μόνο η απογείωση, με συνεχή εκτύπωση z.
Χρησιμοποίησέ το ΜΙΑ φορά για να δούμε τι ακριβώς συμβαίνει στο ύψος κατά
την απογείωση, μετά ξαναγύρνα στο κανονικό step5_explore.py.

    py step5_diagnostic.py
"""
import time

from common import Sensors, arm, check_decks, connect, reset_estimator

HEIGHT = 0.5

with connect() as scf:
    cf = scf.cf
    print("Σύνδεση OK")
    check_decks(cf)
    reset_estimator(cf, scf)
    sensors = Sensors(cf)
    sensors.wait_ready()
    print("Αφετηρία:", Sensors.fmt(sensors.get()))

    try:
        cf.param.set_value("commander.enHighLevel", "1")
    except Exception:
        pass
    arm(cf)
    hl = cf.high_level_commander

    print(f"\nΑπογείωση στα {HEIGHT} m ...")
    t0 = time.time()
    hl.takeoff(HEIGHT, 2.0)

    # ΙΔΙΟ monitoring loop με το step2_hover.py, αλλά για το takeoff() της Flight
    while time.time() - t0 < 4.0:          # 2.0s takeoff + 2.0s παρακολούθηση μετά
        d = sensors.get()
        print(f"[{time.time() - t0:4.2f}s] z={d.get('z', 0):+.3f}  zrange={d.get('zrange', 0):+.3f}  "
              f"vbat={d.get('vbat', 0):.2f}V")
        time.sleep(0.1)

    print("\nΠροσγείωση ...")
    hl.land(0.0, 2.0)
    time.sleep(2.5)
    hl.stop()
    sensors.stop()
print("Τέλος")
