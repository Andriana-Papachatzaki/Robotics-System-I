"""
Φτιάχνει εικόνα από έναν αποθηκευμένο χάρτη (.npz), χωρίς πτήση.

    py plot_map.py                              (results/explore_map.npz)
    py plot_map.py results/map.npz
"""
import sys

from mapping import GridMap

path = sys.argv[1] if len(sys.argv) > 1 else "results/explore_map.npz"
grid, traj = GridMap.load(path)
out = grid.plot(path.rsplit(".", 1)[0] + "_plot.png", traj if len(traj) else None,
                title=path)
print("Αποθηκεύτηκε:", out)
