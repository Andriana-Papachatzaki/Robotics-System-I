"""
ΒΗΜΑ 5: ΑΥΤΟΝΟΜΗ ΕΞΕΡΕΥΝΗΣΗ.

    py step5_explore.py

Το drone:
  1. απογειώνεται και κάνει σάρωση 360° (μένοντας στη θέση του),
  2. χτίζει χάρτη και ΔΙΑΛΕΓΕΙ ΜΟΝΟ ΤΟΥ τον επόμενο στόχο: το κοντινότερο σημείο
     από το οποίο ΔΕΝ έχει περάσει ακόμα (ή που δεν έχει δει), με ασφαλή διαδρομή (Dijkstra),
  3. συνεχίζει μέχρι να έχει περάσει από το TARGET_COVERAGE (80%) του ελεύθερου χώρου
     (ή να τελειώσει ο χρόνος),
  4. επιστρέφει στην αφετηρία (A*) και προσγειώνεται.
Αποθηκεύει: results/explore_map.png, results/explore_map.npz (για το βήμα 6).

Ασφάλεια: Ctrl+C -> προσγείωση. Επίσης: z > 1 m, > FENCE m από την αφετηρία,
παγωμένοι αισθητήρες -> προσγείωση. Η αποφυγή εμποδίων του βήματος 3 είναι πάντα ενεργή.
"""
import os

from common import Flight, SafetyStop, Sensors, check_decks, connect, reset_estimator
from mapping import GridMap
from mission import Mission, save_log

HEIGHT = 0.4
V_MAX = 0.25         # m/s (λίγο πιο αργά: καλύτερη εκτίμηση θέσης με Flow deck)
MAX_TIME = 240.0     # s εξερεύνησης (η μπαταρία κρατά ~5-6 λεπτά)
TARGET_COVERAGE = 0.90  # σταματά όταν έχει περάσει από το 80% του ελεύθερου χώρου
FENCE = 5.0          # m μέγιστη απόσταση από την αφετηρία
OUT = "results"

os.makedirs(OUT, exist_ok=True)
with connect() as scf:
    cf = scf.cf
    print("Σύνδεση OK")
    check_decks(cf)
    reset_estimator(cf, scf)
    sensors = Sensors(cf)
    sensors.wait_ready()
    d0 = sensors.get()
    print("Αφετηρία:", Sensors.fmt(d0))

    grid = GridMap(center=(d0["x"], d0["y"]), size=14.0)
    flight = Flight(cf, sensors, height=HEIGHT, fence=FENCE)
    mission = None
    try:
        flight.takeoff()
        mission = Mission(flight, grid, v_max=V_MAX)
        mission.explore(max_time=MAX_TIME, target_coverage=TARGET_COVERAGE)
        mission.log("Επιστροφή στην αφετηρία")
        mission.go_to(flight.home)
    except SafetyStop as e:
        print("!!! ΑΣΦΑΛΕΙΑ:", e)
    except KeyboardInterrupt:
        print("Ctrl+C")
    finally:
        flight.land()
        sensors.stop()

    if mission is not None:
        grid.save(os.path.join(OUT, "explore_map.npz"), mission.traj)
        save_log(mission.rows, os.path.join(OUT, "explore_log.csv"))
        grid.plot(os.path.join(OUT, "explore_map.png"), mission.traj,
                  title=f"Βήμα 5: εξερεύνηση ({len(mission.targets)} στόχοι, "
                        f"κάλυψη ~{100 * mission.explorer.coverage():.0f}%)")
        print(f"\nΑποθηκεύτηκαν: {OUT}/explore_map.png και {OUT}/explore_map.npz")
print("Τέλος")
