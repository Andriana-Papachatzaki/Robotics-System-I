"""
ΒΗΜΑ 1: Σύνδεση και ανάγνωση αισθητήρων. ΔΕΝ ΠΕΤΑΕΙ.

    py step1_sensors.py

Με Lighthouse, τι να ελέγξεις όσο τρέχει (Ctrl+C για τέλος):
  1. Drone ακίνητο στο πάτωμα  -> z ≈ 0.00 (±0.05) και ΔΕΝ ανεβαίνει μόνο του.
     LH: λήψη / γεωμ / βαθμ / ενεργοί = 2 (ή όσοι σταθμοί βάσης υπάρχουν):
       λήψη=0   -> το drone δεν "βλέπει" σταθμούς (κλειστοί; κάτι σκεπάζει το deck;)
       γεωμ=0   -> οι θέσεις των σταθμών δεν είναι αποθηκευμένες στο drone (cfclient)
       βαθμ=0   -> λείπουν τα δεδομένα βαθμονόμησης των σταθμών
  2. Σήκωσέ το στα ~50 cm       -> z ≈ 0.50
  3. Μετακίνησέ το 1 m σε μια κατεύθυνση -> το x ή το y αλλάζει κατά ~1 m
  4. Χέρι μπροστά/πίσω/αριστερά/δεξιά/πάνω -> αλλάζουν τα F, B, L, R, U
"""
import time

from common import MODE, Sensors, check_decks, connect, reset_estimator

with connect() as scf:
    cf = scf.cf
    print(f"Σύνδεση OK (mode: {MODE})")
    check_decks(cf)
    reset_estimator(cf, scf)
    sensors = Sensors(cf)
    sensors.wait_ready()
    print("Διάβασμα αισθητήρων (Ctrl+C για τέλος)\n")
    try:
        while True:
            line = Sensors.fmt(sensors.get())
            if sensors.ranges_frozen():
                line += "   <-- ΠΡΟΣΟΧΗ: οι αποστάσεις δεν αλλάζουν (παγωμένοι αισθητήρες;)"
            print(line)
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    sensors.stop()
print("Τέλος")
