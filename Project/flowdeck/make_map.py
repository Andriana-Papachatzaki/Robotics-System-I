"""
Ξαναφτιάχνει τον χάρτη από ένα αρχείο καταγραφής πτήσης (χωρίς να πετάξει το drone).
Χρήσιμο για να δοκιμάζεις ρυθμίσεις του χάρτη ή να φτιάχνεις εικόνες για την αναφορά.

    py make_map.py                        (διαβάζει results/flight_log.csv)
    py make_map.py results/flight_log.csv
"""
import csv
import sys

from mapping import GridMap

path = sys.argv[1] if len(sys.argv) > 1 else "results/flight_log.csv"
rows = list(csv.DictReader(open(path)))
if not rows:
    sys.exit("Άδειο αρχείο.")
grid = GridMap(center=(float(rows[0]["x"]), float(rows[0]["y"])))
traj = []
for r in rows:
    x, y, yaw = float(r["x"]), float(r["y"]), float(r["yaw"])
    rng = {k: (float(r[k]) if r[k] != "" else None) for k in ("front", "back", "left", "right")}
    grid.integrate(x, y, yaw, rng)
    grid.mark_visited(x, y)
    traj.append((x, y))
out = path.rsplit(".", 1)[0] + "_map.png"
grid.plot(out, traj, title="Χάρτης από καταγραφή")
print("Αποθηκεύτηκε:", out)
