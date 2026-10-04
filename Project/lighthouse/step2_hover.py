"""
ΒΗΜΑ 2: Απογείωση στα 0.4 m, hover 5 s πάνω από το σημείο εκκίνησης, προσγείωση.

    py step2_hover.py

Χρησιμοποιεί τον επίσημο High-Level Commander του Crazyflie (takeoff / land),
που δουλεύει με απόλυτη θέση (ιδανικό για Lighthouse).

Ασφάλεια:
  - πριν την απογείωση ελέγχει: μπαταρία, z ≈ 0 στο πάτωμα, σταθερή θέση, μη παγωμένους αισθητήρες
  - κατά την πτήση: αν z > 1.0 m ή απομακρυνθεί > 0.5 m -> προσγείωση
  - Ctrl+C -> προσγείωση
"""
import math
import statistics
import time

from common import Sensors, arm, check_decks, connect, reset_estimator

HEIGHT = 0.4        # m
HOVER_TIME = 5.0    # s
MAX_HEIGHT = 1.0    # m
MAX_DRIFT = 0.5     # m


def preflight(sensors):
    """Επιστρέφει (x0, y0) ή σταματά με μήνυμα αν κάτι δεν είναι σωστό."""
    xs, ys, zs = [], [], []
    for _ in range(10):                       # 1 s μετρήσεων με το drone ακίνητο
        d = sensors.get()
        xs.append(d["x"]); ys.append(d["y"]); zs.append(d["z"])
        time.sleep(0.1)
    d = sensors.get()
    print("Πριν την απογείωση:", Sensors.fmt(d))
    problems = []
    if d.get("vbat", 0) < 3.8:
        problems.append(f"χαμηλή μπαταρία ({d['vbat']:.2f} V) - φόρτισε/άλλαξε")
    if abs(statistics.mean(zs)) > 0.10:
        problems.append(f"z={statistics.mean(zs):.2f} m ενώ είναι στο πάτωμα (θα έπρεπε ~0). "
                        "Το σύστημα Lighthouse δεν είναι σωστά βαθμονομημένο ως προς το πάτωμα, "
                        "ή το drone δεν βλέπει τους σταθμούς.")
    if max(zs) - min(zs) > 0.05 or max(xs) - min(xs) > 0.05 or max(ys) - min(ys) > 0.05:
        problems.append("η θέση 'χορεύει' ενώ το drone είναι ακίνητο (κακή λήψη Lighthouse)")
    if "bsActive" in d and bin(int(d["bsActive"])).count("1") < 2:
        problems.append("το drone βλέπει λιγότερους από 2 σταθμούς βάσης Lighthouse")
    if sensors.ranges_frozen():
        problems.append("οι αισθητήρες του Multi-ranger είναι παγωμένοι")
    if problems:
        print("\n!!! ΔΕΝ απογειώνομαι:")
        for p in problems:
            print("   -", p)
        return None
    return statistics.mean(xs), statistics.mean(ys)


with connect() as scf:
    cf = scf.cf
    print("Σύνδεση OK")
    check_decks(cf)
    reset_estimator(cf, scf)
    sensors = Sensors(cf)
    sensors.wait_ready()

    start = preflight(sensors)
    if start is None:
        sensors.stop()
        raise SystemExit
    x0, y0 = start

    try:
        cf.param.set_value("commander.enHighLevel", "1")   # σε παλιά firmware χρειάζεται
    except Exception:
        pass
    arm(cf)
    hl = cf.high_level_commander

    print(f"\nΑπογείωση στα {HEIGHT} m ...")
    try:
        hl.takeoff(HEIGHT, 2.0)
        t0 = time.time()
        while time.time() - t0 < 2.0 + HOVER_TIME:
            d = sensors.get()
            print(f"[{time.time() - t0:4.1f}s] {Sensors.fmt(d)}")
            drift = math.hypot(d["x"] - x0, d["y"] - y0)
            if d["z"] > MAX_HEIGHT:
                print("!!! Πολύ ψηλά -> προσγείωση")
                break
            if drift > MAX_DRIFT:
                print(f"!!! Απομακρύνθηκε {drift:.2f} m -> προσγείωση")
                break
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("Ctrl+C -> προσγείωση")
    finally:
        print("Προσγείωση ...")
        hl.land(0.0, 2.0)
        time.sleep(2.5)
        hl.stop()
        sensors.stop()
print("Τέλος")
