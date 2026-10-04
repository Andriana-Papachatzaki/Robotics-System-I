"""
Κοινά εργαλεία για όλα τα βήματα.

ΡΥΘΜΙΣΕΙΣ: άλλαξε εδώ το URI και τον τρόπο εντοπισμού θέσης.
"""
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

URI = "radio://0/90/2M/E7E7E7E7A1"
MODE = "flow"                # "lighthouse" ή "flow"

INVALID_MM = 4000  # οι VL53L1x δίνουν μεγάλες τιμές (π.χ. 65535) όταν δεν μετράνε
RANGE_KEYS = ("front", "back", "left", "right", "up")


def connect(uri=URI):
    """Άνοιγμα σύνδεσης. Χρήση:  with connect() as scf: ..."""
    cflib.crtp.init_drivers()
    return SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache"))


def check_decks(cf):
    """Σταματά αν λείπει κάποιο απαραίτητο deck."""
    need = [("deck.bcMultiranger", "Multi-ranger")]
    if MODE == "lighthouse":
        need.append(("deck.bcLighthouse4", "Lighthouse deck"))
    else:
        need.append(("deck.bcFlow2", "Flow deck V2"))
    for param, name in need:
        if int(cf.param.get_value(param)) != 1:
            raise RuntimeError(f"Δεν βρέθηκε το {name}. Έλεγξε ότι είναι καλά κουμπωμένο.")
        print(f"  {name}: OK")
    if MODE == "lighthouse" and int(cf.param.get_value("deck.bcFlow2")) == 1:
        print("  ΠΡΟΣΟΧΗ: είναι τοποθετημένο και το Flow deck. Αφού χαλάει, ΒΓΑΛ' ΤΟ.")


def reset_estimator(cf, scf):
    """Kalman estimator + reset, και αναμονή μέχρι να συγκλίνει (όπως στα παραδείγματα Bitcraze)."""
    cf.param.set_value("stabilizer.estimator", "2")
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    cf.param.set_value("kalman.resetEstimation", "0")

    lc = LogConfig(name="var", period_in_ms=100)
    for v in ("varPX", "varPY", "varPZ"):
        lc.add_variable(f"kalman.{v}", "float")
    hist = {v: [1000.0] * 10 for v in ("varPX", "varPY", "varPZ")}
    t0 = time.time()
    with SyncLogger(scf, lc) as logger:
        for _, data, _ in logger:
            for v in hist:
                hist[v] = hist[v][1:] + [data[f"kalman.{v}"]]
            if all(max(h) - min(h) < 0.001 for h in hist.values()):
                print("  Estimator: OK")
                return
            if time.time() - t0 > 10:
                print("  Estimator: δεν συνέκλινε πλήρως (συνεχίζω)")
                return


def arm(cf):
    """Τα νεότερα firmware θέλουν arming πριν την πτήση."""
    try:
        if hasattr(cf, "supervisor"):
            cf.supervisor.send_arming_request(True)
        else:
            cf.platform.send_arming_request(True)
        time.sleep(0.5)
    except AttributeError:
        pass


class Sensors:
    """
    Διαβάζει συνεχώς (10 Hz) θέση, αποστάσεις, μπαταρία (+ κατάσταση Lighthouse).
    Τιμές σε μέτρα. Άκυρη μέτρηση απόστασης = None.
    Προσθέτει μόνο όσες μεταβλητές υπάρχουν στο drone (ανάλογα με τα decks).
    """

    def __init__(self, cf, period_ms=100):
        self.lock = threading.Lock()
        self.data = {}
        self.count = 0
        self._last_ranges = None   # για ανίχνευση "παγωμένων" αισθητήρων
        self._last_change = time.time()
        toc = cf.log.toc

        def has(name):
            return toc.get_element_by_complete_name(name) is not None

        # ΣΥΓΧΡΟΝΙΣΜΕΝΟ πακέτο: θέση + 4 οριζόντιες αποστάσεις ΤΗΝ ΙΔΙΑ ΣΤΙΓΜΗ
        # (16 + 8 = 24 bytes <= 26). Αλλιώς, όσο το drone γυρίζει, θέση και μέτρηση
        # απέχουν χρονικά και οι τοίχοι στον χάρτη "μουτζουρώνονται".
        a = LogConfig(name="sync", period_in_ms=period_ms)
        for v in ("x", "y", "z", "yaw"):
            a.add_variable(f"stateEstimate.{v}", "float")
        for v in ("front", "back", "left", "right"):
            if has(f"range.{v}"):
                a.add_variable(f"range.{v}", "uint16_t")

        b = LogConfig(name="extra", period_in_ms=period_ms)
        b.add_variable("pm.vbat", "float")
        for v in ("up", "zrange"):
            if has(f"range.{v}"):
                b.add_variable(f"range.{v}", "uint16_t")

        # κατάσταση Lighthouse (bitmask ανά σταθμό βάσης) σε ξεχωριστό πακέτο
        # (κάθε πακέτο logging χωράει το πολύ 26 bytes)
        configs = [a, b]
        lh_vars = [v for v in ("bsReceive", "bsGeoVal", "bsCalVal", "bsActive")
                   if has(f"lighthouse.{v}")]
        if lh_vars:
            c = LogConfig(name="lh", period_in_ms=max(period_ms, 200))
            for v in lh_vars:
                c.add_variable(f"lighthouse.{v}", "uint16_t")
            configs.append(c)
        self._configs = tuple(configs)
        for lc in self._configs:
            cf.log.add_config(lc)
            lc.data_received_cb.add_callback(self._cb)
            lc.start()

    def _cb(self, ts, data, conf):
        with self.lock:
            for k, v in data.items():
                name = k.split(".")[1]
                if k.startswith("range."):
                    v = None if v >= INVALID_MM else v / 1000.0
                self.data[name] = v
            self.count += 1
            if conf.name == "sync":
                self.data["seq"] = self.data.get("seq", 0) + 1   # αύξων αριθμός μέτρησης
                cur = tuple(self.data.get(k) for k in RANGE_KEYS)
                if cur != self._last_ranges:
                    self._last_ranges = cur
                    self._last_change = time.time()

    def get(self):
        with self.lock:
            return dict(self.data)

    def ranges_frozen(self, seconds=3.0):
        """True αν οι αποστάσεις δεν έχουν αλλάξει ΚΑΘΟΛΟΥ για 'seconds' (παγωμένοι αισθητήρες)."""
        with self.lock:
            numeric = [v for v in (self._last_ranges or ()) if v is not None]
            return bool(numeric) and time.time() - self._last_change > seconds

    def wait_ready(self, timeout=3.0):
        t0 = time.time()
        while self.count < 4 and time.time() - t0 < timeout:
            time.sleep(0.05)
        if self.count < 4:
            raise RuntimeError("Δεν φτάνουν δεδομένα από το drone. Κλείσε το cfclient, "
                               "κάνε restart στο drone και έλεγξε το URI.")
        time.sleep(0.5)
        d = self.get()
        if all(d.get(k) == 0.0 for k in RANGE_KEYS):
            raise RuntimeError("Όλοι οι αισθητήρες του Multi-ranger δείχνουν ακριβώς 0 -> το drone "
                               "δεν ξεκίνησε σωστά. Έλεγξε τα decks και κάνε restart.")

    def stop(self):
        for lc in self._configs:
            try:
                lc.stop()
            except Exception:
                pass

    @staticmethod
    def fmt(d):
        def r(k):
            v = d.get(k)
            return " --- " if v is None else f"{v:5.2f}"
        s = (f"x={d.get('x', 0):+.2f} y={d.get('y', 0):+.2f} z={d.get('z', 0):+.2f} "
             f"yaw={d.get('yaw', 0):+6.1f}° | "
             f"F={r('front')} B={r('back')} L={r('left')} R={r('right')} U={r('up')} | "
             f"bat={d.get('vbat', 0):.2f}V")
        def n(k):
            return bin(int(d[k])).count("1") if k in d else "?"
        if MODE == "lighthouse" and any(k in d for k in ("bsReceive", "bsGeoVal", "bsCalVal", "bsActive")):
            s += (f" | LH: λήψη={n('bsReceive')} γεωμ={n('bsGeoVal')} "
                  f"βαθμ={n('bsCalVal')} ενεργοί={n('bsActive')}")
        if MODE == "flow" and "zrange" in d:
            s += f" | zrange={r('zrange')}"
        return s


# =====================================================================================
#  Πτήση & αποφυγή εμποδίων (χρησιμοποιούνται από το βήμα 3 και μετά)
# =====================================================================================
import math  # noqa: E402

SENSOR_DIRS = {"front": 0.0, "left": 90.0, "back": 180.0, "right": -90.0}  # μοίρες, πλαίσιο σώματος


class SafetyStop(Exception):
    """Κάτι πήγε στραβά -> προσγείωση."""


def avoid(vx, vy, yaw_deg, d, safety=0.40, gain=1.2, vmax=0.3, brake=0.8):
    """
    Αποφυγή εμποδίων με τον Multi-ranger (τελευταίο επίπεδο ασφαλείας).
    Για κάθε αισθητήρα που βλέπει εμπόδιο:
      - ΦΡΕΝΑΡΙΣΜΑ: η ταχύτητα ΠΡΟΣ το εμπόδιο περιορίζεται σε  brake * (r - safety),
        δηλαδή μειώνεται σταδιακά όσο πλησιάζει και μηδενίζεται στην απόσταση 'safety',
      - ΑΠΩΣΗ: αν είναι πιο κοντά από 'safety', προστίθεται ταχύτητα που το απομακρύνει.
    Επιστρέφει (vx, vy, blocked): blocked=True αν ένα εμπόδιο εμποδίζει την κίνηση.
    """
    blocked = False
    for name, ang in SENSOR_DIRS.items():
        r = d.get(name)
        if r is None:
            continue
        a = math.radians(yaw_deg + ang)
        ux, uy = math.cos(a), math.sin(a)          # κατεύθυνση αισθητήρα στο πλαίσιο του κόσμου
        toward = vx * ux + vy * uy                  # ταχύτητα προς το εμπόδιο
        allowed = max(0.0, brake * (r - safety))
        if toward > allowed + 1e-3:
            vx -= (toward - allowed) * ux
            vy -= (toward - allowed) * uy
            if r < safety + 0.10 and toward > 0.02:
                blocked = True
        if r < safety:
            push = gain * (safety - r)
            vx -= push * ux
            vy -= push * uy
    s = math.hypot(vx, vy)
    if s > vmax:
        vx, vy = vx / s * vmax, vy / s * vmax
    return vx, vy, blocked


class HeightMonitor:
    """
    (Μόνο για Flow deck) Ελέγχει ότι ο αισθητήρας ύψους του Flow deck δουλεύει σωστά:
      - δίνει έγκυρες μετρήσεις όσο πετάμε,
      - δεν έχει "παγώσει" (ίδια ακριβώς τιμή για > 1.5 s),
      - συμφωνεί με το ύψος του Kalman (|z - zrange| < 0.3 m).
    Αν κάτι από αυτά αποτύχει για > 0.6 s συνεχόμενα -> πρόβλημα.
    (Αυτό ακριβώς συνέβη όταν χάλασε το πρώτο Flow deck: το zrange πάγωσε και το z ανέβαινε.)
    """

    def __init__(self):
        self.bad_since = None
        self.last_zr = None
        self.last_change = time.time()

    def update(self, d):
        now = time.time()
        zr, z = d.get("zrange"), d.get("z", 0.0)
        if zr != self.last_zr:
            self.last_zr, self.last_change = zr, now
        problem = None
        if zr is None or zr < 0.05:
            problem = f"ο αισθητήρας ύψους δεν δίνει μέτρηση (zrange={zr})"
        elif now - self.last_change > 1.5:
            problem = f"ο αισθητήρας ύψους έχει παγώσει (zrange={zr:.2f})"
        elif abs(z - zr) > 0.3:
            problem = f"ύψος Kalman z={z:.2f} και αισθητήρα zrange={zr:.2f} διαφωνούν"
        if problem is None:
            self.bad_since = None
            return None
        self.bad_since = self.bad_since or now
        return problem if now - self.bad_since > 0.6 else None


class Flight:
    """
    Απογείωση/προσγείωση με τον High-Level Commander και κίνηση με εντολές ταχύτητας
    στο πλαίσιο του κόσμου. Το ύψος κρατιέται με απλό P-έλεγχο.
    Lighthouse: το "κόσμος" είναι το πλαίσιο του εργαστηρίου.
    Flow deck:  το "κόσμος" έχει αρχή το σημείο απογείωσης και άξονα x την αρχική
                κατεύθυνση του drone (μετά το reset του Kalman).
    """

    def __init__(self, cf, sensors, height=0.4, max_height=1.0, fence=3.0):
        self.cf = cf
        self.s = sensors
        self.height = height
        self.max_height = max_height
        self.fence = fence              # μέγιστη απόσταση από την αφετηρία [m]
        self.home = None
        self.flying = False
        self.use_up_sensor = True
        self.hmon = HeightMonitor() if MODE == "flow" else None

    def takeoff(self):
        d = self.s.get()
        self.home = (d["x"], d["y"])
        up = d.get("up")
        if up is not None and up < 0.10:
            # π.χ. το Lighthouse deck είναι πάνω από το Multi-ranger και σκεπάζει τον πάνω αισθητήρα
            self.use_up_sensor = False
            print(f"  Ο πάνω αισθητήρας είναι σκεπασμένος (U={up:.2f} m) -> "
                  "απενεργοποιώ τον έλεγχο 'χέρι από πάνω'. Χρησιμοποίησε Ctrl+C για διακοπή.")
        try:
            self.cf.param.set_value("commander.enHighLevel", "1")
        except Exception:
            pass
        arm(self.cf)
        print(f"Απογείωση στα {self.height} m ...")
        self.cf.high_level_commander.takeoff(self.height, 2.0)
        self.flying = True
        # (μετά την απογείωση, οι εντολές send() συνεχίζουν στο ΙΔΙΟ ύψος -> ομαλή μετάβαση)
        # Παρακολούθηση ΚΑΤΑ την απογείωση (εδώ θα φαινόταν ένα "φευγάτο" drone)
        t0 = time.time()
        while time.time() - t0 < 2.5:
            d = self.s.get()
            zr = d.get("zrange") or 0.0
            if d["z"] > self.max_height or zr > self.max_height:
                self.emergency_stop()
                raise SafetyStop(f"ανέβηκε πολύ ψηλά στην απογείωση (z={d['z']:.2f}, zrange={zr:.2f})"
                                 " -> ΣΒΗΣΑΝ τα μοτέρ")
            if self.hmon is not None and time.time() - t0 > 1.5:
                problem = self.hmon.update(d)
                if problem:
                    raise SafetyStop("Flow deck: " + problem)
            time.sleep(0.05)

    def check(self):
        d = self.s.get()
        if d["z"] > self.max_height:
            raise SafetyStop(f"πολύ ψηλά (z={d['z']:.2f} m)")
        dist = math.hypot(d["x"] - self.home[0], d["y"] - self.home[1])
        if dist > self.fence:
            raise SafetyStop(f"βγήκε από την επιτρεπτή περιοχή ({dist:.2f} m από την αφετηρία)")
        if self.s.ranges_frozen():
            raise SafetyStop("παγωμένοι αισθητήρες Multi-ranger")
        up = d.get("up")
        if self.use_up_sensor and up is not None and up < 0.2:
            raise SafetyStop("κάτι πάνω από το drone (χέρι;)")
        if self.hmon is not None:
            problem = self.hmon.update(d)
            if problem:
                raise SafetyStop("Flow deck: " + problem)
        return d

    def emergency_stop(self):
        """Σβήνει αμέσως τα μοτέρ (μόνο αν το drone ανεβαίνει ανεξέλεγκτα)."""
        try:
            self.cf.commander.send_stop_setpoint()
            self.cf.high_level_commander.stop()
        except Exception:
            pass
        self.flying = False

    def send(self, vx, vy, yawrate=0.0):
        """
        Εντολή ταχύτητας (m/s) στο πλαίσιο του κόσμου (η μετατροπή στους άξονες του drone
        γίνεται μέσα στο firmware). Το ύψος κρατιέται με P-έλεγχο (vz = 2*(στόχος - z)).
        Πρέπει να καλείται συνεχώς (~10-20 Hz).
        """
        d = self.check()
        vz = max(-0.3, min(0.3, 2.0 * (self.height - d["z"])))
        s = math.hypot(vx, vy)
        if s > 0.4:                                   # απόλυτο όριο ασφαλείας ταχύτητας
            vx, vy = vx / s * 0.4, vy / s * 0.4
        self.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yawrate)
        return d

    def land(self):
        if not self.flying:
            return
        print("Προσγείωση ...")
        try:
            self.cf.commander.send_notify_setpoint_stop()   # επιστροφή στον High-Level Commander
        except Exception:
            pass
        self.cf.high_level_commander.land(0.0, 2.0)
        time.sleep(2.5)
        self.cf.high_level_commander.stop()
        self.flying = False
