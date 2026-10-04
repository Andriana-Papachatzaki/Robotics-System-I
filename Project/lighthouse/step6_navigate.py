"""
ΒΗΜΑ 6: ΠΛΟΗΓΗΣΗ ΣΕ ΣΗΜΕΙΑ με τον χάρτη της εξερεύνησης.

    py step6_navigate.py

1. Φορτώνει τον χάρτη results/explore_map.npz (από το βήμα 5).
2. Απογειώνεται. Όσο πετάει, γράφεις στο cmd στόχους στη μορφή:   x y
   ΜΠΟΡΕΙΣ ΝΑ ΓΡΑΨΕΙΣ ΝΕΟ ΣΤΟΧΟ ΟΠΟΙΑΔΗΠΟΤΕ ΣΤΙΓΜΗ: ο τρέχων ακυρώνεται.
   Αν ο στόχος δεν είναι προσβάσιμος (εμπόδιο), το drone σταματά μετά από λίγα
   δευτερόλεπτα και περιμένει νέο στόχο.
   (συντεταγμένες Lighthouse σε μέτρα, π.χ.  1.2 -0.5 ). Κενό Enter = τέλος.
   (Αν θέλεις σταθερή λίστα αντί για πληκτρολόγηση, συμπλήρωσε το GOALS.)
3. Για κάθε στόχο: βέλτιστη ασφαλής διαδρομή (A* + εξομάλυνση) πάνω στον χάρτη,
   αποφυγή απρόβλεπτων εμποδίων με τον Multi-ranger, επανασχεδιασμός αν χρειαστεί.
4. Στο τέλος επιστρέφει στην αφετηρία και προσγειώνεται.
Αποθηκεύει: results/navigation.png και results/navigation.csv (αποτελέσματα ανά στόχο).

Όσο περιμένει να γράψεις στόχο, το drone κρατά σταθερή θέση (hover).
"""
import csv
import math
import os
import queue
import threading

from common import Flight, SafetyStop, Sensors, check_decks, connect, reset_estimator
from mapping import GridMap
from mission import Mission

MAP_FILE = "results/explore_map.npz"
GOALS = []            # π.χ. [(1.0, 0.5), (-0.8, 1.2)]  -> αν είναι κενό, ρωτάει στο cmd
HEIGHT = 0.4
V_MAX = 0.3
SAFETY = 0.30         # m: απόσταση που κρατά από απρόβλεπτα εμπόδια (Multi-ranger)
FENCE = 6.0
OUT = "results"


def input_thread(q):
    """Διαβάζει στόχους από το πληκτρολόγιο ΧΩΡΙΣ να σταματά ο έλεγχος πτήσης."""
    while True:
        try:
            line = input("στόχος (x y, κενό = τέλος)> ").strip()
        except EOFError:
            line = ""
        if not line:
            q.put(None)
            return
        try:
            x, y = map(float, line.replace(",", " ").split())
            q.put((x, y))
        except ValueError:
            print("   Γράψε δύο αριθμούς, π.χ.  1.0 -0.5")


if not os.path.exists(MAP_FILE):
    raise SystemExit(f"Δεν βρέθηκε ο χάρτης {MAP_FILE}. Τρέξε πρώτα το step5_explore.py.")
grid, _ = GridMap.load(MAP_FILE)
print(f"Φορτώθηκε ο χάρτης {MAP_FILE}")

results = []
with connect() as scf:
    cf = scf.cf
    print("Σύνδεση OK")
    check_decks(cf)
    reset_estimator(cf, scf)
    sensors = Sensors(cf)
    sensors.wait_ready()
    print("Αφετηρία:", Sensors.fmt(sensors.get()))

    flight = Flight(cf, sensors, height=HEIGHT, fence=FENCE)
    mission = None
    try:
        flight.takeoff()
        mission = Mission(flight, grid, v_max=V_MAX, safety=SAFETY)
        mission.hold(1.0)

        q = queue.Queue()
        if GOALS:
            for g in GOALS:
                q.put(g)
            q.put(None)
        else:
            print("\nΓράψε στόχους (συντεταγμένες Lighthouse). Το drone περιμένει σε hover.")
            threading.Thread(target=input_thread, args=(q,), daemon=True).start()

        while True:
            try:
                goal = q.get_nowait()
            except queue.Empty:
                mission.hold(0.2)                    # hover όσο περιμένουμε
                continue
            if goal is None:
                break
            d = sensors.get()
            n0, t0 = len(mission.traj), mission.clock.now()
            straight = math.hypot(goal[0] - d["x"], goal[1] - d["y"])
            mission.log(f"Πλοήγηση προς ({goal[0]:+.2f}, {goal[1]:+.2f}), ευθεία απόσταση {straight:.2f} m")
            status = mission.go_to(goal, abort=lambda: not q.empty())
            ok = status == "ok"
            d = sensors.get()
            err = math.hypot(goal[0] - d["x"], goal[1] - d["y"])
            seg = mission.traj[n0:]
            length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(seg[:-1], seg[1:]))
            res = dict(goal_x=goal[0], goal_y=goal[1], success=ok, status=status, error_m=round(err, 3),
                       path_m=round(length, 2), straight_m=round(straight, 2),
                       time_s=round(mission.clock.now() - t0, 1))
            results.append(res)
            label = {"ok": "ΕΦΤΑΣΕ", "stuck": "ΔΕΝ ΜΠΟΡΕΣΕ ΝΑ ΦΤΑΣΕΙ", "aborted": "ΑΚΥΡΩΘΗΚΕ (νέος στόχος)",
                     "timeout": "TIMEOUT", "nopath": "ΔΕΝ ΥΠΑΡΧΕΙ ΔΙΑΔΡΟΜΗ"}[status]
            mission.log(f"   {label}: σφάλμα {err:.2f} m, διαδρομή {length:.2f} m "
                        f"(ευθεία {straight:.2f} m), χρόνος {res['time_s']} s")
            if not GOALS:
                print("στόχος (x y, κενό = τέλος)> ", end="", flush=True)

        mission.log("Επιστροφή στην αφετηρία")
        mission.go_to(flight.home)
    except SafetyStop as e:
        print("!!! ΑΣΦΑΛΕΙΑ:", e)
    except KeyboardInterrupt:
        print("Ctrl+C")
    finally:
        flight.land()
        sensors.stop()

    os.makedirs(OUT, exist_ok=True)
    if results:
        with open(os.path.join(OUT, "navigation.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            w.writeheader()
            w.writerows(results)
    if mission is not None:
        out = grid.plot(os.path.join(OUT, "navigation.png"), mission.traj,
                        title=f"Βήμα 6: πλοήγηση ({sum(r['success'] for r in results)}/{len(results)} στόχοι)",
                        extra_points=[(r["goal_x"], r["goal_y"]) for r in results] or None)
        print(f"\nΑποθηκεύτηκαν: {out} και {OUT}/navigation.csv")
print("Τέλος")