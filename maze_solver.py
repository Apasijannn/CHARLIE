"""
Maze Solver - Pybricks (berbasis sensor, tanpa urutan belok manual)
Komponen:
  - 1 Color Sensor (menghadap ke bawah) -> ikuti garis & cari cabang jalur
  - 1 Ultrasonic Sensor (menghadap ke depan) -> cek obstacle di tiap cabang

Alur:
  1. Kalibrasi otomatis nilai hitam/putih (robot goyang kiri-kanan di atas garis).
  2. Robot mengikuti garis (PD line follower).
  3. Robot berhenti di sebuah TITIK KEPUTUSAN kalau:
       - color sensor mendeteksi persimpangan / tikungan tajam (hitam penuh), atau
       - ultrasonic mendeteksi obstacle di depan (robot pelan, lalu berhenti).
  4. Di PERSIMPANGAN: kalau depan BEBAS (tidak ada obstacle) & garis lurus ada
     -> robot langsung jalan LURUS tanpa memindai.
     Selain itu robot MEMINDAI: berputar dari kiri ke kanan sambil
       - color sensor mencatat arah mana saja yang ADA GARIS (KIRI / LURUS / KANAN)
       - ultrasonic mencatat arah mana saja yang ADA OBSTACLE
  5. Robot memilih cabang yang ada garis DAN tidak ada obstacle,
     berdasarkan urutan prioritas PRIORITY (default: kiri, lurus, kanan).
  6. Kasus khusus:
       - Obstacle di tengah garis tanpa cabang -> menghindar ke sisi yang kosong,
         lalu cari garis lagi.
       - Semua cabang tertutup -> putar balik.
  7. FINISH = garis hitam PUTUS sebentar (celah putih pendek) di jalur lurus.
     Kalau garis habis saat robot jalan lurus, robot cek:
       - garis ada di samping (cuma meleset)? -> lanjut follow
       - jalan lurus sebentar, ketemu hitam lagi dalam jarak pendek -> FINISH, berhenti
       - tidak ketemu -> garis memang hilang, mundur & cari garis
"""

from pybricks.hubs import PrimeHub          # ganti ke InventorHub / TechnicHub jika perlu
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Color, Icon, Side
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, Matrix
from umath import sin, cos, pi
from usys import stdin
from uselect import poll

# ===================== SETUP HARDWARE =====================
hub = PrimeHub()

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.E)
color_sensor = ColorSensor(Port.C)
ultrasonic = UltrasonicSensor(Port.D)

# Sesuaikan dengan ukuran roda & jarak antar roda robot kamu (mm)
robot = DriveBase(left_motor, right_motor, wheel_diameter=50, axle_track=130)

# ===================== PARAMETER TUNING =====================
AUTO_CALIBRATE = True # True = kalibrasi otomatis di awal (robot harus mulai DI ATAS garis)
BLACK = 10            # dipakai kalau AUTO_CALIBRATE = False
WHITE = 80
DEBUG = True          # print debug ke console Pybricks
DEBUG_EVERY = 10      # print data line follower tiap N loop
TELEMETRY = True      # print data rute (baris "T," & "E,") untuk app tracker
TEL_EVERY = 100       # ms antar data posisi

# --- Strategi ---
# Urutan prioritas memilih cabang yang terbuka.
# ["KIRI", "LURUS", "KANAN"] = aturan tangan kiri (umum untuk maze)
PRIORITY = ["KIRI", "LURUS", "KANAN"]
# True = di persimpangan, kalau depan bebas & garis lurus ada -> langsung LURUS
STRAIGHT_IF_CLEAR = True
# Tepi garis yang diikuti. "KIRI" cocok dengan prioritas kiri
# (cabang kiri pasti terdeteksi). "KANAN" = perilaku versi sebelumnya.
FOLLOW_EDGE = "KIRI"

# --- Mata & lagu ---
EYES_ENABLED = True     # ekspresi mata di layar 5x5 hub
MUSIC_ENABLED = True    # lagu saat start & finish
VOLUME = 70             # 0 - 100
BLINK_EVERY = 3000      # ms, robot kedip tiap ... ms
BG_MUSIC_ENABLED = True # lagu latar selama robot jalan (lihat SONG_DRIVE)
BG_TEMPO = 120          # ketukan per menit lagu latar (lihat angka tempo di partitur)
BG_LOOP = True          # True = lagu latar diulang terus
SFX_ENABLED = True      # bunyi pendek saat obstacle / garis hilang
DISPLAY_UP = None       # kalau mata miring/terbalik, isi Side.TOP / Side.BOTTOM /
                        # Side.LEFT / Side.RIGHT (sisi hub yang jadi "atas" layar)

# --- Line follower (PD controller) ---
KP = 2.3            # gain proportional
KD = 5.7          # gain derivative
DRIVE_SPEED = 150    # mm/s kecepatan maksimal
MIN_SPEED = 40        # mm/s kecepatan minimal saat tikungan tajam 40
SLOW_FACTOR = 1.5     # makin besar error, makin pelan 1.5
APPROACH_SPEED = 50   # mm/s saat mendekati obstacle

# --- Garis hilang ---
LOST_TIME = 400       # ms di atas putih terus-menerus -> garis hilang
SEARCH_ANGLE = 120    # derajat sapuan maksimal saat mencari garis 120

# --- Finish (celah putih di garis) ---
FINISH_ENABLED = False
WHITE_TRIGGER = 12      # mm putih berturut-turut -> cek apakah ini celah finish
GAP_MAX_DIST = 80      # mm maks jalan lurus mencari sambungan garis setelah celah 80
STRAIGHT_TOL = 30       # derajat, robot dianggap "jalan lurus" kalau arahnya
                        # berubah kurang dari ini dalam ~100 mm terakhir
SIDE_CHECK_ANGLE = 20   # derajat cek garis di samping (bedakan meleset vs putus)

# --- Titik keputusan (persimpangan / obstacle) ---
INTERSECTION_DIST = 20  # mm hitam penuh berturut-turut -> persimpangan
CENTER_DIST = 60        # mm maju supaya AS RODA pas di tengah persimpangan
                        # (ukur: jarak color sensor ke as roda)
COOLDOWN_DIST = 60      # mm setelah keputusan, deteksi persimpangan dimatikan
SCAN_ANGLE = 110        # derajat pindai ke kiri & kanan
SCAN_TURN_RATE = 80     # derajat/detik saat memindai (pelan biar akurat)
BIN_WIDTH = 40          # toleransi sudut cabang (misal KIRI = -90 +- 40)
US_WINDOW = 15          # ultrasonic dibaca di sudut cabang +- 15 derajat
SIDE_OBSTACLE_DIST = 200  # mm -> cabang dianggap tertutup obstacle

# --- Obstacle di depan ---
OBSTACLE_DIST = 150   # mm -> mulai pelan (ada obstacle di depan)
STOP_DIST = 70        # mm -> berhenti & ambil keputusan 70
CLEAR_DIST = 300      # mm -> jalur dianggap bebas saat menghindar
TURN_STEP = 10        # derajat per langkah putar saat menghindar
MAX_TURN = 180        # batas maksimal putaran saat menghindar
EXTRA_TURN = 15       # putaran tambahan setelah bebas
SEARCH_SPEED = 80     # mm/s saat jalan lurus mencari garis
MAX_SEARCH_DIST = 300 # mm batas jalan lurus mencari garis

# ===================== VARIABEL INTERNAL =====================
EDGE_SIGN = 1 if FOLLOW_EDGE == "KIRI" else -1
DIR_ANGLE = {"KIRI": -90, "LURUS": 0, "KANAN": 90}

THRESHOLD = 0
LINE_FOUND = 0
WHITE_LEVEL = 0

last_error = 0
lost_timer = StopWatch()
run_timer = StopWatch()
loop_count = 0
node_count = 0
black_start_dist = None
black_start_angle = 0
white_start_dist = None   # posisi (mm) saat mulai masuk putih
heading_hist = []         # riwayat (jarak, sudut) untuk cek jalan lurus
cooldown_start = 0
state = "INIT"


# ===================== MATA (layar 5x5) =====================
# Nilai 0-100 = kecerahan tiap LED. Mata kiri = kolom 0-1, mata kanan = kolom 3-4.
FACES = {
    "DEPAN": [[0,   0, 0,   0,   0],
              [50, 50, 0,  50,  50],
              [100,100,0, 100, 100],
              [50, 50, 0,  50,  50],
              [0,   0, 0,   0,   0]],
    "KIRI":  [[0,   0, 0,   0,   0],
              [50, 20, 0,  50,  20],
              [100, 20, 0, 100,  20],
              [50, 20, 0,  50,  20],
              [0,   0, 0,   0,   0]],
    "KANAN": [[0,   0, 0,   0,   0],
              [20, 50, 0,  20,  50],
              [20,100, 0,  20, 100],
              [20, 50, 0,  20,  50],
              [0,   0, 0,   0,   0]],
    "KEDIP": [[0,   0, 0,   0,   0],
              [0,   0, 0,   0,   0],
              [60, 60, 0,  60,  60],
              [0,   0, 0,   0,   0],
              [0,   0, 0,   0,   0]],
    "KAGET": [[100,100,0, 100, 100],
              [100,100,0, 100, 100],
              [0,   0, 0,   0,   0],
              [0,   0,100,  0,   0],
              [0,   0, 0,   0,   0]],
    "BINGUNG": [[0,   0, 0,  60,  60],
                [100,100,0, 100, 100],
                [100,100,0,   0,   0],
                [0,   0, 0,   0,   0],
                [0,  60, 60, 60,  0]],
    "SENANG": [[0,   0, 0,   0,   0],
               [100,100,0, 100, 100],
               [0,   0, 0,   0,   0],
               [100, 0, 0,   0, 100],
               [0, 100,100,100,  0]],
    "FOKUS": [[0,   0, 0,   0,   0],
              [0,   0, 0,   0,   0],
              [100,100,0, 100, 100],
              [50, 50, 0,  50,  50],
              [0,   0, 0,   0,   0]],
}
FACE_MATRIX = {}
current_face = None
blink_timer = StopWatch()


def setup_display():
    if not EYES_ENABLED:
        return
    if DISPLAY_UP is not None:
        hub.display.orientation(DISPLAY_UP)
    for name, rows in FACES.items():
        FACE_MATRIX[name] = Matrix(rows)


def show_face(name):
    """Tampilkan ekspresi (hanya update layar kalau berubah)."""
    global current_face
    if not EYES_ENABLED or name == current_face:
        return
    current_face = name
    hub.display.icon(FACE_MATRIX[name])


def look_by_angle(angle, deadzone=20):
    """Mata melirik sesuai arah: sudut negatif = kiri, positif = kanan."""
    if angle < -deadzone:
        show_face("KIRI")
    elif angle > deadzone:
        show_face("KANAN")
    else:
        show_face("DEPAN")


def face_while_driving(turn_rate):
    """Saat ikut garis: mata melirik ke arah belok + kedip berkala."""
    t = blink_timer.time()
    if t > BLINK_EVERY:
        show_face("KEDIP")
        if t > BLINK_EVERY + 150:
            blink_timer.reset()
        return
    look_by_angle(turn_rate, deadzone=40)


# Ekspresi otomatis tiap ganti state
STATE_FACE = {
    "CALIBRATE": "FOKUS",
    "SCAN": "DEPAN",
    "NODE": "FOKUS",
    "AVOID": "KAGET",
    "SEARCH_LINE": "BINGUNG",
    "FIND_LINE": "BINGUNG",
    "CHECK_FINISH": "FOKUS",
    "FINISH": "SENANG",
}


# ===================== LAGU =====================
# Format nada Pybricks: "C4/4" = nada C oktaf 4, 1/4 ketukan.
# "R/8" = diam 1/8 ketukan, "." = titik (lebih panjang 1.5x).
SONG_START = ["C5/16", "E5/16", "G5/16", "C6/8", "R/16", "G5/16", "C6/4"]
# Ode to Joy (Beethoven, domain publik)
SONG_FINISH = ["E4/4", "E4/4", "F4/4", "G4/4", "G4/4", "F4/4", "E4/4", "D4/4",
               "C4/4", "C4/4", "D4/4", "E4/4", "E4/4.", "D4/8", "D4/2"]
SOUND_OBSTACLE = ["G5/16", "E5/16", "C5/8"]
SOUND_LOST = ["E5/16", "R/16", "E5/16", "C5/8"]


# ---------- LAGU LATAR (diputar selama robot jalan) ----------
# GANTI isi SONG_DRIVE dengan not lagu pilihanmu (format sama seperti di atas).
# Yang di bawah ini cuma melodi contoh sederhana.
SONG_DRIVE = [
    "D4/8", "F#4/8", "A4/8", "F#4/8", "D4/8", "F#4/8", "A4/4",
    "E4/8", "G4/8", "B4/8", "G4/8", "E4/8", "G4/8", "B4/4",
    "F#4/8", "A4/8", "D5/8", "A4/8", "G4/8", "B4/8", "E5/4",
    "D5/4", "A4/4", "D4/2",
]

NOTE_INDEX = {"C": 0, "D": 2, "E": 4, "F": 5, "G": 7, "A": 9, "B": 11}


def parse_note(text, tempo):
    """Ubah "C#4/8." -> (frekuensi Hz, durasi ms). Frekuensi 0 = diam (R)."""
    text = text.replace("_", "")
    name, dur = text.split("/")
    dotted = dur.endswith(".")
    if dotted:
        dur = dur[:-1]
    ms = 60000 / tempo * 4 / int(dur)
    if dotted:
        ms *= 1.5
    if name[0] == "R":
        return 0, int(ms)
    rest = name[1:]
    acc = 0
    if rest[0] == "#":
        acc, rest = 1, rest[1:]
    elif rest[0] == "b":
        acc, rest = -1, rest[1:]
    midi = 12 * (int(rest) + 1) + NOTE_INDEX[name[0]] + acc
    return int(440 * 2 ** ((midi - 69) / 12)), int(ms)


bg_song = []
bg_idx = 0
bg_active = False
bg_on = False          # nada sedang berbunyi?
bg_t_off = 0           # waktu nada dimatikan (biar nada berulang terdengar terpisah)
bg_t_next = 0          # waktu pindah ke nada berikutnya
bg_timer = StopWatch()


def silence():
    # beep 1 ms = menghentikan nada yang sedang berbunyi terus-menerus
    hub.speaker.beep(100, 1)


def bg_start_note(start_time):
    global bg_on, bg_t_off, bg_t_next
    freq, ms = bg_song[bg_idx]
    if freq:
        hub.speaker.beep(freq, -1)      # durasi negatif = bunyi terus, tidak menahan program
        bg_on = True
    else:
        silence()
        bg_on = False
    bg_t_off = start_time + ms * 0.85
    bg_t_next = start_time + ms


def music_start():
    global bg_song, bg_idx, bg_active
    if not (MUSIC_ENABLED and BG_MUSIC_ENABLED and SONG_DRIVE):
        return
    bg_song = [parse_note(n, BG_TEMPO) for n in SONG_DRIVE]
    bg_idx = 0
    bg_active = True
    bg_timer.reset()
    bg_start_note(0)
    dbg("Lagu latar mulai:", len(bg_song), "nada | tempo", BG_TEMPO)


def music_stop():
    global bg_active
    if bg_active:
        bg_active = False
        silence()


def music_tick():
    """Dipanggil sesering mungkin: ganti nada kalau waktunya sudah lewat."""
    global bg_idx, bg_on, bg_active
    if not bg_active:
        return
    t = bg_timer.time()
    if bg_on and t >= bg_t_off:
        silence()
        bg_on = False
    if t >= bg_t_next:
        bg_idx += 1
        if bg_idx >= len(bg_song):
            if not BG_LOOP:
                music_stop()
                return
            bg_idx = 0
        # kalau program sempat tertahan lama (belok/scan), lanjut dari sekarang
        start = bg_t_next if t - bg_t_next < 200 else t
        bg_start_note(start)


def tick_wait(ms):
    """Pengganti wait() di dalam loop: sekalian update lagu latar & telemetri."""
    music_tick()
    pose_update()
    telemetry()
    cmd_tick()
    wait(ms)


# ===================== TELEMETRI (untuk app tracker) =====================
# Posisi robot dihitung dari jarak & sudut roda (odometri).
# x, y dalam mm dari titik start. Arah 0 = arah awal robot, positif = belok kanan.
# Baris yang di-print:
#   T,waktu_ms,x,y,arah,state,reflection,ultrasonic,kecepatan
#   E,waktu_ms,x,y,JENIS,detail...
pose_x = 0.0
pose_y = 0.0
pose_h = 0.0
pose_last_d = 0
pose_last_a = 0
tel_timer = StopWatch()


def pose_update():
    global pose_x, pose_y, pose_h, pose_last_d, pose_last_a
    d = robot.distance()
    a = robot.angle()
    dd = d - pose_last_d
    da = a - pose_last_a
    h_mid = (pose_h + da / 2) * pi / 180
    pose_x += dd * cos(h_mid)
    pose_y += dd * sin(h_mid)
    pose_h += da
    pose_last_d = d
    pose_last_a = a


def drive_reset():
    """Pengganti robot.reset(): posisi di tracker tetap nyambung."""
    global pose_last_d, pose_last_a
    pose_update()
    robot.reset()
    pose_last_d = 0
    pose_last_a = 0


def telemetry(force=False):
    if not TELEMETRY:
        return
    if not force and tel_timer.time() < TEL_EVERY:
        return
    tel_timer.reset()
    print("T,{},{:.0f},{:.0f},{:.0f},{},{},{},{:.0f}".format(
        run_timer.time(), pose_x, pose_y, pose_h, state,
        color_sensor.reflection(), ultrasonic.distance(), robot.state()[1]))


# ===================== PERINTAH DARI APP (lewat Bluetooth) =====================
# App tracker mengirim satu baris per perintah:
#   GET                  -> robot print semua parameter (baris "P,NAMA,NILAI")
#   SET NAMA NILAI       -> ubah parameter saat robot jalan (balasan "A,OK,..." / "A,ERR,...")
#   PAUSE / RESUME       -> robot berhenti di tempat / lanjut jalan
# Hanya parameter di daftar ini yang boleh diubah dari app.
LIVE_PARAMS = {
    "DRIVE_SPEED": int, "MIN_SPEED": int, "APPROACH_SPEED": int, "SEARCH_SPEED": int,
    "KP": float, "KD": float, "SLOW_FACTOR": float,
    "LOST_TIME": int, "SEARCH_ANGLE": int,
    "INTERSECTION_DIST": int, "CENTER_DIST": int, "COOLDOWN_DIST": int,
    "SCAN_ANGLE": int, "SCAN_TURN_RATE": int, "SIDE_OBSTACLE_DIST": int,
    "OBSTACLE_DIST": int, "STOP_DIST": int, "CLEAR_DIST": int, "MAX_SEARCH_DIST": int,
    "STRAIGHT_IF_CLEAR": bool, "PRIORITY": list, "FOLLOW_EDGE": str,
    "FINISH_ENABLED": bool, "WHITE_TRIGGER": int, "GAP_MAX_DIST": int, "STRAIGHT_TOL": int,
    "EYES_ENABLED": bool, "MUSIC_ENABLED": bool, "BG_MUSIC_ENABLED": bool,
    "SFX_ENABLED": bool, "VOLUME": int, "DEBUG": bool, "TEL_EVERY": int,
}
cmd_poll = poll()
cmd_poll.register(stdin)
cmd_buf = ""
paused = False


def fmt_param(v):
    if isinstance(v, list):
        return "|".join(v)
    return str(v)


def dump_params():
    g = globals()
    for name in LIVE_PARAMS:
        print("P,{},{}".format(name, fmt_param(g[name])))


def set_param(name, text):
    global EDGE_SIGN
    kind = LIVE_PARAMS.get(name)
    if kind is None:
        print("A,ERR,{},parameter tidak dikenal".format(name))
        return
    try:
        if kind is bool:
            val = text.lower() in ("1", "true", "on", "ya")
        elif kind is list:
            val = [x for x in text.replace(",", "|").split("|") if x]
        elif kind is int:
            val = int(float(text))
        elif kind is float:
            val = float(text)
        else:
            val = text
    except ValueError:
        print("A,ERR,{},nilai tidak valid".format(name))
        return
    globals()[name] = val
    if name == "FOLLOW_EDGE":
        EDGE_SIGN = 1 if val == "KIRI" else -1
    elif name == "VOLUME":
        hub.speaker.volume(val)
    print("A,OK,{},{}".format(name, fmt_param(val)))


def handle_cmd(line):
    global paused
    parts = line.strip().split(" ", 2)
    if not parts or not parts[0]:
        return
    c = parts[0].upper()
    if c == "GET":
        dump_params()
    elif c == "SET" and len(parts) == 3:
        set_param(parts[1].upper(), parts[2].strip())
    elif c == "PAUSE":
        paused = True
        robot.stop()
        print("A,OK,PAUSE,1")
    elif c == "RESUME":
        paused = False
        print("A,OK,PAUSE,0")
    else:
        print("A,ERR,{},perintah tidak dikenal".format(c))


def cmd_tick():
    """Baca perintah dari app tanpa menahan program."""
    global cmd_buf
    while cmd_poll.poll(0):
        ch = stdin.read(1)
        if ch in ("\n", "\r"):
            if cmd_buf:
                handle_cmd(cmd_buf)
            cmd_buf = ""
        else:
            cmd_buf += ch


def ev(kind, *detail):
    """Catat kejadian penting (persimpangan, obstacle, pilihan arah, dst)."""
    if not TELEMETRY:
        return
    pose_update()
    print("E,{},{:.0f},{:.0f},{}".format(run_timer.time(), pose_x, pose_y, kind)
          + "".join("," + str(x) for x in detail))


def play(notes, tempo=160, sfx=False):
    """Mainkan lagu (robot diam selama lagu berbunyi).
    Lagu latar dijeda sebentar lalu lanjut lagi."""
    if not MUSIC_ENABLED or (sfx and not SFX_ENABLED):
        return
    if bg_active:
        silence()
    hub.speaker.play_notes(notes, tempo)
    if bg_active:
        bg_start_note(bg_timer.time())


# ===================== DEBUG =====================
def dbg(*args):
    """Print dengan timestamp (detik) & state saat ini."""
    if DEBUG:
        print("[{:6.2f}s][{}]".format(run_timer.time() / 1000, state), *args)


def set_state(new_state):
    global state
    if new_state != state:
        old = state
        state = new_state
        dbg("STATE:", old, "->", new_state)
        ev("STATE", new_state)
        if new_state in STATE_FACE:
            show_face(STATE_FACE[new_state])


# ===================== SENSOR =====================
def update_thresholds():
    global THRESHOLD, LINE_FOUND, WHITE_LEVEL
    THRESHOLD = (BLACK + WHITE) / 2
    LINE_FOUND = BLACK + (WHITE - BLACK) * 0.35
    WHITE_LEVEL = WHITE - (WHITE - BLACK) * 0.25
    dbg("KALIBRASI -> BLACK:", BLACK, "| WHITE:", WHITE,
        "| THRESHOLD:", THRESHOLD, "| LINE_FOUND:", LINE_FOUND,
        "| WHITE_LEVEL:", WHITE_LEVEL)


def calibrate():
    global BLACK, WHITE
    set_state("CALIBRATE")
    dbg("Mulai kalibrasi, pastikan sensor di atas garis hitam")
    lo, hi = 100, 0

    def sweep(angle):
        nonlocal lo, hi
        robot.turn(angle, wait=False)
        while not robot.done():
            r = color_sensor.reflection()
            lo = min(lo, r)
            look_by_angle(robot.angle())
            hi = max(hi, r)
            tick_wait(5)

    robot.settings(turn_rate=60)
    sweep(-40)
    dbg("Sapuan kiri  -> min:", lo, "max:", hi)
    sweep(80)
    dbg("Sapuan kanan -> min:", lo, "max:", hi)
    sweep(-40)
    dbg("Kembali tengah -> min:", lo, "max:", hi, "| kontras:", hi - lo)
    robot.settings(turn_rate=150)

    if hi - lo < 20:
        hub.speaker.beep(300, 800)
        dbg("PERINGATAN: kontras kecil! lo =", lo, "hi =", hi,
            "-> turunkan sensor / cek posisi di atas garis")
    BLACK, WHITE = lo, hi
    ev("CAL", lo, hi)


def on_line():
    return color_sensor.reflection() < LINE_FOUND


def on_white():
    return color_sensor.reflection() > WHITE_LEVEL


def path_clear():
    return ultrasonic.distance() > CLEAR_DIST


def read_distance(n=3):
    """Baca ultrasonic beberapa kali, ambil median biar stabil."""
    vals = []
    for _ in range(n):
        vals.append(ultrasonic.distance())
        tick_wait(30)
    vals.sort()
    return vals[len(vals) // 2]


# ===================== GERAK DASAR =====================
def turn_until_line(angle):
    """Putar sebesar 'angle', berhenti lebih awal kalau ketemu garis."""
    dbg("Putar cari garis:", angle, "derajat")
    drive_reset()
    robot.turn(angle, wait=False)
    while not robot.done():
        r = color_sensor.reflection()
        if r < LINE_FOUND:
            robot.stop()
            dbg("Garis KETEMU saat putar | R:", r, "| sudut:", robot.angle())
            return True
        tick_wait(5)
    dbg("Garis tidak ketemu dalam", angle, "derajat")
    return False


def search_line(first=None):
    """Sapu kiri-kanan mencari garis. first: -1 kiri dulu, +1 kanan dulu.
    Default: arah yang paling mungkin sesuai tepi yang diikuti."""
    if first is None:
        first = EDGE_SIGN
    set_state("SEARCH_LINE")
    robot.stop()
    play(SOUND_LOST, tempo=240, sfx=True)
    nama = "KIRI" if first < 0 else "KANAN"
    lawan = "KANAN" if first < 0 else "KIRI"
    dbg("Sapu", nama, "dulu")
    if turn_until_line(first * SEARCH_ANGLE):
        return True
    dbg("Sapu", lawan)
    if turn_until_line(-first * 2 * SEARCH_ANGLE):
        return True
    dbg("GAGAL menemukan garis, kembali ke arah semula")
    turn_until_line(first * SEARCH_ANGLE)
    return False


def align_to_line():
    """Setelah belok ke cabang, pastikan sensor pas di garis (koreksi kecil)."""
    if not on_white():
        return
    dbg("Sensor belum di garis -> koreksi kecil")
    if turn_until_line(-25):
        return
    if turn_until_line(50):
        return
    turn_until_line(-25)
    search_line()


def reset_follower():
    global last_error, black_start_dist, cooldown_start, white_start_dist, heading_hist
    last_error = 0
    lost_timer.reset()
    black_start_dist = None
    white_start_dist = None
    heading_hist = []
    cooldown_start = robot.distance()


# ===================== FINISH =====================
def record_heading():
    """Simpan sudut robot tiap 20 mm (maks 6 titik = ~100 mm terakhir)."""
    global heading_hist
    d = robot.distance()
    if not heading_hist or d - heading_hist[-1][0] >= 20:
        heading_hist.append((d, robot.angle()))
        if len(heading_hist) > 6:
            heading_hist.pop(0)


def straight_heading():
    """Return sudut rata-rata kalau robot sedang jalan lurus, None kalau tidak."""
    if len(heading_hist) < 4:
        return None
    angles = [a for _, a in heading_hist]
    if max(angles) - min(angles) > STRAIGHT_TOL:
        return None
    return sum(angles) / len(angles)


def finish():
    set_state("FINISH")
    robot.stop()
    dbg("=== FINISH! === total waktu:", run_timer.time() / 1000, "detik")
    telemetry(force=True)
    ev("FINISH", run_timer.time())
    hub.light.on(Color.GREEN)
    show_face("SENANG")
    music_stop()
    play(SONG_FINISH, tempo=200)
    # Animasi hati berdenyut beberapa kali
    if EYES_ENABLED:
        for _ in range(4):
            hub.display.icon(Icon.HEART)
            wait(400)
            hub.display.icon(Icon.HEART * 0.3)
            wait(300)
        hub.display.icon(Icon.HEART)
    raise SystemExit


def check_finish(line_heading):
    """Garis habis saat jalan lurus -> cek apakah ini celah finish."""
    set_state("CHECK_FINISH")
    robot.stop()
    dbg("Garis habis di jalur lurus | arah garis:", int(line_heading))

    # 1) Luruskan ke arah garis
    robot.turn(line_heading - robot.angle())

    # 2) Cek garis di samping kiri/kanan -> kalau ada, cuma meleset
    if turn_until_line(-SIDE_CHECK_ANGLE) or turn_until_line(2 * SIDE_CHECK_ANGLE):
        dbg("Garis ada di samping -> cuma meleset, lanjut follow")
        return
    robot.turn(-SIDE_CHECK_ANGLE)        # kembali ke arah garis

    # 3) Jalan lurus mencari sambungan garis
    drive_reset()
    robot.drive(SEARCH_SPEED, 0)
    found = False
    while robot.distance() < GAP_MAX_DIST:
        if on_line():
            found = True
            break
        tick_wait(5)
    robot.stop()
    gap = robot.distance() + WHITE_TRIGGER
    if found:
        dbg("Garis SAMBUNG lagi setelah celah +-", gap, "mm -> FINISH")
        finish()

    # 4) Bukan celah finish -> mundur & cari garis biasa
    dbg("Tidak ada sambungan dalam", GAP_MAX_DIST, "mm -> bukan finish, cari garis")
    back_distance = robot.distance()
    normal_speed = robot.settings()[0]

    robot.settings(straight_speed=200)  # faster reverse; try 200 first
    robot.straight(-back_distance)
    robot.settings(straight_speed=normal_speed)
    search_line()


# ===================== TITIK KEPUTUSAN =====================
def scan_node():
    """Scan left first. Take an open left branch immediately; otherwise
    finish scanning straight and right.
    """
    set_state("SCAN")
    drive_reset()

    hits = {"KIRI": 0, "LURUS": 0, "KANAN": 0}
    dist = {"KIRI": 2000, "LURUS": 2000, "KANAN": 2000}

    left_saw_white = False
    left_checked = False

    robot.settings(turn_rate=SCAN_TURN_RATE)

    for target in (-SCAN_ANGLE, SCAN_ANGLE):
        robot.turn(target - robot.angle(), wait=False)

        while not robot.done():
            a = robot.angle()
            r = color_sensor.reflection()
            d = ultrasonic.distance()
            look_by_angle(a)

            # The sensor must leave the black junction patch before
            # another black reading can count as a LEFT branch.
            if a < 0 and r > WHITE_LEVEL:
                left_saw_white = True

            for name, center in DIR_ANGLE.items():
                if abs(a - center) <= US_WINDOW:
                    dist[name] = min(dist[name], d)

                if abs(a - center) <= BIN_WIDTH and r < LINE_FOUND:
                    if name != "KIRI" or left_saw_white:
                        hits[name] += 1

            # At roughly -90 degrees, check whether LEFT is usable.
            # Do this only once, so a blocked left branch won't repeatedly
            # interrupt the sweep.
            if (not left_checked
                    and -98 <= a <= -90
                    and hits["KIRI"] >= 2):

                left_checked = True
                robot.stop()

                # Confirm obstacle distance while facing the branch.
                dist["KIRI"] = min(dist["KIRI"], read_distance())

                if dist["KIRI"] >= SIDE_OBSTACLE_DIST:
                    robot.settings(turn_rate=150)
                    dbg("KIRI ada garis dan bebas -> berhenti scan, ambil KIRI")
                    return ["KIRI"], dist

                dbg("KIRI ada garis tapi terhalang -> lanjut scan")
                robot.turn(target - robot.angle(), wait=False)

            tick_wait(10)

    robot.settings(turn_rate=150)

    branches = [
        name for name in ("KIRI", "LURUS", "KANAN")
        if hits[name] >= 2
    ]

    for name in ("KIRI", "LURUS", "KANAN"):
        dbg("  ", name,
            "| garis:", "ADA" if name in branches else "tidak",
            "(hits", hits[name], ")",
            "| US:", dist[name], "mm")

    return branches, dist


def face(direction_name):
    """Hadapkan robot ke arah cabang (sudut relatif dari awal scan)."""
    target = DIR_ANGLE[direction_name]
    robot.turn(target - robot.angle())


def straight_line_ahead():
    """Cek garis lurus di depan sensor (dengan koreksi sudut kecil)."""
    if not on_white():
        return True
    if turn_until_line(-20):
        return True
    if turn_until_line(40):
        return True
    robot.turn(-20)          # kembali ke arah semula
    return False


def decide_at_node(start_angle=None, at_junction=True):
    """Titik keputusan: pindai semua arah lalu pilih cabang yang terbuka."""
    global node_count
    node_count += 1
    set_state("NODE")
    robot.stop()
    ev("NODE", node_count, "persimpangan" if at_junction else "obstacle")
    dbg(">>> TITIK KEPUTUSAN ke-" + str(node_count),
        "| sebab:", "persimpangan" if at_junction else "obstacle di depan")

    # 1) Luruskan arah (batalkan belokan sesaat sebelum terdeteksi)
    if start_angle is not None:
        koreksi = start_angle - robot.angle()
        dbg("Koreksi arah:", koreksi, "derajat")
        robot.turn(koreksi)

    # 2) Maju sampai as roda di tengah persimpangan (jangan sampai nabrak)
    if at_junction:
        front = read_distance()
        maju = CENTER_DIST
        if front < CENTER_DIST + 50:
            maju = max(0, front - 50)
        dbg("Maju ke tengah persimpangan:", maju, "mm | US depan:", front, "mm")
        robot.straight(maju)

        # 2b) Depan bebas & ada garis lurus -> langsung LURUS tanpa scan
        if STRAIGHT_IF_CLEAR and front >= SIDE_OBSTACLE_DIST:
            if straight_line_ahead():
                dbg("Depan BEBAS & garis lurus ada -> langsung LURUS")
                ev("PILIH", "LURUS", "langsung")
                reset_follower()
                return
            dbg("Depan bebas tapi tidak ada garis lurus (tikungan) -> scan")
        elif STRAIGHT_IF_CLEAR:
            dbg("Ada obstacle di depan (", front, "mm) -> scan")

    # 3) Pindai
    branches, dist = scan_node()
    open_dirs = [n for n in PRIORITY
                 if n in branches and dist[n] >= SIDE_OBSTACLE_DIST]
    front_blocked = dist["LURUS"] < SIDE_OBSTACLE_DIST
    side_branches = [n for n in branches if n != "LURUS"]
    dbg("Cabang ada:", branches, "| terbuka:", open_dirs)
    ev("SCAN", "+".join(branches) or "-", dist["KIRI"], dist["LURUS"], dist["KANAN"])

    # 4) Putuskan
    if open_dirs:
        pilih = open_dirs[0]
        dbg("PILIH:", pilih)
        ev("PILIH", pilih, "scan")
        face(pilih)
        align_to_line()

    elif front_blocked and not side_branches:
        # Obstacle di tengah garis tanpa cabang -> menghindar ke sisi kosong
        sisi = [n for n in PRIORITY if n != "LURUS"
                and dist[n] >= SIDE_OBSTACLE_DIST]
        if not sisi:
            sisi = ["KIRI" if dist["KIRI"] >= dist["KANAN"] else "KANAN"]
        dbg("Tidak ada cabang, obstacle di depan -> MENGHINDAR ke", sisi[0])
        ev("AVOID", sisi[0])
        face("LURUS")
        avoid_obstacle(DIR_ANGLE[sisi[0]] // 90)

    elif branches:
        dbg("Semua cabang tertutup -> PUTAR BALIK")
        ev("UTURN")
        face("LURUS")
        robot.turn(180)
        align_to_line()

    else:
        dbg("Tidak ada cabang & tidak ada obstacle -> cari garis")
        face("LURUS")
        search_line()

    reset_follower()


def avoid_obstacle(direction):
    """Menghindar: putar ke 'direction' (-1 kiri, +1 kanan) sampai jalur bebas,
    lalu jalan lurus mencari garis lagi."""
    set_state("AVOID")
    arah = "KIRI" if direction < 0 else "KANAN"
    dbg("Putar", arah, "sampai bebas (US >", CLEAR_DIST, "mm)")
    turned = 0
    while not path_clear() and turned < MAX_TURN:
        robot.turn(direction * TURN_STEP)
        turned += TURN_STEP
        dbg("  putar", turned, "derajat | US:", ultrasonic.distance(), "mm")
    if turned >= MAX_TURN:
        dbg("PERINGATAN: sudah", MAX_TURN, "derajat tapi belum bebas")
    robot.turn(direction * EXTRA_TURN)
    find_line_after_turn()


def find_line_after_turn():
    """Jalan lurus sampai ketemu garis, kalau tidak ketemu sapu kiri-kanan."""
    set_state("FIND_LINE")
    if on_line():
        dbg("Sudah di atas garis -> lanjut follow")
        return
    dbg("Jalan lurus cari garis, maks", MAX_SEARCH_DIST, "mm")
    drive_reset()
    robot.drive(SEARCH_SPEED, 0)
    while robot.distance() < MAX_SEARCH_DIST:
        if on_line():
            dbg("Garis KETEMU setelah jalan", robot.distance(), "mm")
            break
        if ultrasonic.distance() < STOP_DIST:
            dbg("Obstacle saat mencari garis | US:", ultrasonic.distance(), "mm")
            break
        tick_wait(5)
    robot.stop()
    if not on_line() and ultrasonic.distance() >= STOP_DIST:
        search_line()


# ===================== LINE FOLLOWER =====================
def follow_line(max_speed):
    """PD line follower + deteksi persimpangan & garis hilang."""
    global last_error, loop_count, black_start_dist, black_start_angle, white_start_dist
    set_state("FOLLOW_LINE")
    loop_count += 1
    reflection = color_sensor.reflection()
    error = reflection - THRESHOLD
    derivative = error - last_error
    last_error = error

    # Deteksi persimpangan: hitam penuh berturut-turut
    armed = robot.distance() - cooldown_start > COOLDOWN_DIST
    if reflection < LINE_FOUND and armed:
        if black_start_dist is None:
            black_start_dist = robot.distance()
            black_start_angle = robot.angle()
        else:
            black_len = robot.distance() - black_start_dist
            if black_len > INTERSECTION_DIST:
                dbg("Hitam penuh", black_len, "mm -> PERSIMPANGAN")
                decide_at_node(black_start_angle, at_junction=True)
                return
    else:
        if black_start_dist is not None and DEBUG:
            black_len = robot.distance() - black_start_dist
            if black_len > 5:
                dbg("Hitam penuh", black_len, "mm (bukan persimpangan, batas",
                    INTERSECTION_DIST, "mm)")
        black_start_dist = None

    turn_rate = EDGE_SIGN * (KP * error + KD * derivative)
    speed = max(MIN_SPEED, max_speed - abs(error) * SLOW_FACTOR)
    speed = min(speed, max_speed)
    robot.drive(speed, turn_rate)
    face_while_driving(turn_rate)

    # Garis hilang / celah finish
    record_heading()
    if on_white():
        if white_start_dist is None:
            white_start_dist = robot.distance()
        white_len = robot.distance() - white_start_dist
        heading = straight_heading()
        if FINISH_ENABLED and white_len > WHITE_TRIGGER and heading is not None:
            check_finish(heading)
            reset_follower()
            return
        if lost_timer.time() > LOST_TIME:
            dbg("Garis HILANG (putih selama", lost_timer.time(), "ms) -> mencari")
            ev("LOST")
            search_line()
            reset_follower()
            return
    else:
        white_start_dist = None
        lost_timer.reset()

    if DEBUG and loop_count % DEBUG_EVERY == 0:
        pos = "HITAM" if reflection < LINE_FOUND else ("PUTIH" if reflection > WHITE_LEVEL else "TEPI")
        dbg("R:", reflection, "(" + pos + ")",
            "| err:", round(error, 1), "| d:", round(derivative, 1),
            "| turn:", int(turn_rate), "| speed:", int(speed),
            "| US:", ultrasonic.distance(), "mm")


# ===================== MAIN =====================
hub.speaker.volume(VOLUME)
setup_display()
show_face("KEDIP")
wait(100)
show_face("DEPAN")
play(SONG_START, tempo=180)
dbg("=== MAZE SOLVER START ===")
dbg("Baterai:", hub.battery.voltage(), "mV | US awal:", ultrasonic.distance(),
    "mm | R awal:", color_sensor.reflection())
dbg("Prioritas:", PRIORITY, "| ikuti tepi:", FOLLOW_EDGE)
wait(100)

if AUTO_CALIBRATE:
    calibrate()
else:
    dbg("Pakai nilai manual (tanpa kalibrasi)")
update_thresholds()
drive_reset()
reset_follower()
show_face("SENANG")
hub.speaker.beep(800, 200)
wait(100)
blink_timer.reset()
music_start()
ev("START", DRIVE_SPEED, "-".join(PRIORITY), FOLLOW_EDGE)
dump_params()

while True:
    if paused:
        robot.stop()
        show_face("FOKUS")
        tick_wait(50)
        continue
    d = ultrasonic.distance()
    if d < STOP_DIST:
        dbg(">>> OBSTACLE di depan | jarak:", d, "mm")
        ev("OBSTACLE", d)
        robot.stop()
        print('kecepatan jadi 110')
        show_face("KAGET")
        play(SOUND_OBSTACLE, tempo=240, sfx=True)
        decide_at_node(at_junction=False)
    elif d < OBSTACLE_DIST:
        follow_line(APPROACH_SPEED)   # ada obstacle di depan -> pelan
    else:
        follow_line(DRIVE_SPEED)
    tick_wait(10)
