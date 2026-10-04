"""
ΒΗΜΑ 2: Απογείωση στα 0.4 m, hover 5 s πάνω από το σημείο εκκίνησης, προσγείωση.

    py step2_hover.py

Χρησιμοποιεί τον επίσημο High-Level Commander του Crazyflie (takeoff / land),
που δουλεύει με απόλυτη θέση (ιδανικό για Lighthouse· με Flow deck δουλεύει το
ίδιο, απλά η "απόλυτη θέση" είναι σχετική με το σημείο όπου έγινε reset).

Ασφάλεια:
  - πριν την απογείωση ελέγχει: μπαταρία, z ≈ 0 στο πάτωμα, σταθερή θέση, μη παγωμένους αισθητήρες
  - κατά την πτήση: αν z > 1.0 m ή απομακρυνθεί > 0.5 m -> προσγείωση
  - Ctrl+C -> προσγείωση

ΠΡΩΤΗ ΠΤΗΣΗ ΜΕΤΑ ΑΠΟ ΑΛΛΑΓΗ DECK (π.χ. Lighthouse -> Flow): κάνε το σε μικρό,
καθαρό χώρο, με το χέρι σου έτοιμο να το πιάσεις αν κάτι πάει στραβά.
"""
import math
import statistics
import time

from common import MODE, Sensors, arm, check_decks, connect, reset_estimator

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
        if MODE == "lighthouse":
            reason = ("Το σύστημα Lighthouse δεν είναι σωστά βαθμονομημένο ως προς "
                      "το πάτωμα, ή το drone δεν βλέπει τους σταθμούς.")
        else:
            reason = ("Ο Kalman estimator δεν έχει συγκλίνει σωστά με το Flow deck, "
                      "ή το zrange δίνει λάθος τιμή -- δοκίμασε ξανά reset_estimator "
                      "ή έλεγξε ότι το deck βλέπει καθαρά το πάτωμα.")
        problems.append(f"z={statistics.mean(zs):.2f} m ενώ είναι στο πάτωμα (θα έπρεπε ~0). " + reason)
    if max(zs) - min(zs) > 0.05 or max(xs) - min(xs) > 0.05 or max(ys) - min(ys) > 0.05:
        cause = "κακή λήψη Lighthouse" if MODE == "lighthouse" else "ασταθής εκτίμηση από το Flow deck"
        problems.append(f"η θέση 'χορεύει' ενώ το drone είναι ακίνητο ({cause})")
    # MODE-gated: το πεδίο bsActive υπάρχει πάντα (είναι log μεταβλητή του ίδιου
    # του firmware, όχι του deck) και σε Flow mode διαβάζει απλά 0 -- χωρίς αυτό
    # το MODE=="lighthouse", ο έλεγχος μπλοκάρει ΚΑΘΕ απογείωση σε Flow mode.
    if MODE == "lighthouse" and "bsActive" in d and bin(int(d["bsActive"])).count("1") < 2:
        problems.append("το drone βλέπει λιγότερους από 2 σταθμούς βάσης Lighthouse")
    if MODE == "flow" and "zrange" in d and d.get("zrange") is not None:
        if abs(d["zrange"] - d.get("z", 0)) > 0.10:
            problems.append(f"zrange ({d['zrange']:.2f} m) και z ({d.get('z', 0):.2f} m) διαφωνούν "
                            "-- πιθανό πρόβλημα με το Flow deck (φωτισμός/υφή πατώματος;)")
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
    print(f"Σύνδεση OK (mode: {MODE})")
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
