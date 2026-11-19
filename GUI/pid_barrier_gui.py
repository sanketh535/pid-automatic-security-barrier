import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import time
from collections import deque

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# =========================================================
# ESP32 PID BARRIER GUI
#
# Expected ESP32 telemetry format:
# DATA,time,distance,targetAngle,measuredAngle,error,P,I,D,servoCommand,mode
# =========================================================

BAUD_RATE = 115200
MAX_POINTS = 150

ser = None
running = True
data_queue = queue.Queue()
data_lock = threading.Lock()

history = {
    "time": deque(maxlen=MAX_POINTS),
    "distance": deque(maxlen=MAX_POINTS),
    "error": deque(maxlen=MAX_POINTS),
    "pid_sum": deque(maxlen=MAX_POINTS),
    "angle": deque(maxlen=MAX_POINTS),
    "servo": deque(maxlen=MAX_POINTS),
}

# =========================================================
# MAIN WINDOW
# =========================================================

root = tk.Tk()
root.title("ESP32 PID Barrier Controller")
root.geometry("1250x900")
root.minsize(950, 700)

style = ttk.Style()
style.configure("Title.TLabel", font=("Arial", 16, "bold"))
style.configure("Value.TLabel", font=("Arial", 12, "bold"))

# =========================================================
# SERIAL CONNECTION
# =========================================================

connection_frame = ttk.LabelFrame(root, text="ESP32 Connection", padding=8)
connection_frame.pack(fill="x", padx=10, pady=5)

ttk.Label(connection_frame, text="COM Port:").pack(side="left")

port_var = tk.StringVar()
port_combo = ttk.Combobox(
    connection_frame, textvariable=port_var, width=15, state="readonly"
)
port_combo.pack(side="left", padx=5)

connection_status = tk.StringVar(value="Disconnected")
ttk.Label(connection_frame, textvariable=connection_status).pack(
    side="left", padx=10
)


def refresh_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    port_combo["values"] = ports
    if ports and port_var.get() not in ports:
        port_var.set(ports[0])


def serial_reader():
    """Read serial data in a background thread; do not update Tk widgets here."""
    global ser

    while running:
        try:
            current_serial = ser
            if current_serial and current_serial.is_open:
                line = current_serial.readline().decode(
                    "utf-8", errors="ignore"
                ).strip()
                if line:
                    data_queue.put(line)
            else:
                time.sleep(0.05)
        except Exception as exc:
            data_queue.put("ERROR," + str(exc))
            time.sleep(0.2)


def connect_serial():
    global ser

    if ser and ser.is_open:
        try:
            ser.close()
        except Exception:
            pass
        ser = None
        connection_status.set("Disconnected")
        connect_button.config(text="Connect")
        return

    port = port_var.get()
    if not port:
        messagebox.showwarning(
            "Select Port", "Select the ESP32 COM port first."
        )
        return

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.2)
        # Opening the ESP32 serial port may reset the board.
        time.sleep(2)
        connection_status.set("Connected: " + port)
        connect_button.config(text="Disconnect")
        log_message("Connected to " + port)
    except Exception as exc:
        ser = None
        messagebox.showerror("Connection Error", str(exc))


def send_command(command):
    if not ser or not ser.is_open:
        messagebox.showwarning(
            "Not Connected", "Connect to the ESP32 first."
        )
        return

    try:
        ser.write((command.strip() + "\n").encode("utf-8"))
        log_message("TX: " + command)
    except Exception as exc:
        messagebox.showerror("Send Error", str(exc))


ttk.Button(
    connection_frame, text="Refresh Ports", command=refresh_ports
).pack(side="left", padx=5)

connect_button = ttk.Button(
    connection_frame, text="Connect", command=connect_serial
)
connect_button.pack(side="left", padx=5)

# =========================================================
# LIVE READINGS
# =========================================================

readings_frame = ttk.LabelFrame(root, text="Live System Readings", padding=8)
readings_frame.pack(fill="x", padx=10, pady=5)

reading_vars = {
    "Distance (cm)": tk.StringVar(value="--"),
    "Target Angle": tk.StringVar(value="--"),
    "AS5600 Angle": tk.StringVar(value="--"),
    "PID Error": tk.StringVar(value="--"),
    "PID Output (P+I+D)": tk.StringVar(value="--"),
    "Servo Position": tk.StringVar(value="--"),
    "Mode": tk.StringVar(value="--"),
}

for index, (label, variable) in enumerate(reading_vars.items()):
    row = index // 4
    col = (index % 4) * 2
    ttk.Label(readings_frame, text=label).grid(
        row=row, column=col, sticky="w", padx=5, pady=4
    )
    ttk.Label(
        readings_frame, textvariable=variable, style="Value.TLabel"
    ).grid(row=row, column=col + 1, sticky="w", padx=5, pady=4)

# =========================================================
# PID TUNING
# Defaults match the values stated in the ESP32 firmware.
# =========================================================

pid_frame = ttk.LabelFrame(root, text="Live PID Tuning", padding=8)
pid_frame.pack(fill="x", padx=10, pady=5)

kp_var = tk.StringVar(value="3.0")
ki_var = tk.StringVar(value="0.20")
kd_var = tk.StringVar(value="0.08")

for col, (label, variable) in enumerate([
    ("Kp", kp_var), ("Ki", ki_var), ("Kd", kd_var)
]):
    ttk.Label(pid_frame, text=label).grid(
        row=0, column=col * 2, padx=5, pady=3
    )
    ttk.Entry(pid_frame, textvariable=variable, width=10).grid(
        row=0, column=col * 2 + 1, padx=5, pady=3
    )


def apply_pid():
    try:
        kp = float(kp_var.get())
        ki = float(ki_var.get())
        kd = float(kd_var.get())
        if min(kp, ki, kd) < 0:
            raise ValueError("PID values cannot be negative.")
        send_command(f"PID,{kp},{ki},{kd}")
    except ValueError as exc:
        messagebox.showerror("Invalid PID", str(exc))


ttk.Button(
    pid_frame, text="Apply PID Values", command=apply_pid
).grid(row=0, column=6, padx=10)

# =========================================================
# DISTANCE THRESHOLDS
# =========================================================

distance_frame = ttk.LabelFrame(
    root, text="Distance Thresholds (cm)", padding=8
)
distance_frame.pack(fill="x", padx=10, pady=5)

far_var = tk.StringVar(value="30")
mid_var = tk.StringVar(value="20")

ttk.Label(distance_frame, text="Far boundary:").grid(
    row=0, column=0, padx=5
)
ttk.Entry(distance_frame, textvariable=far_var, width=9).grid(
    row=0, column=1, padx=5
)
ttk.Label(distance_frame, text="Middle boundary:").grid(
    row=0, column=2, padx=5
)
ttk.Entry(distance_frame, textvariable=mid_var, width=9).grid(
    row=0, column=3, padx=5
)


def apply_thresholds():
    try:
        far = float(far_var.get())
        mid = float(mid_var.get())
        if far <= mid or mid <= 0:
            raise ValueError(
                "Far boundary must be greater than the middle boundary."
            )
        send_command(f"SET,{far},{mid}")
    except ValueError as exc:
        messagebox.showerror("Invalid Thresholds", str(exc))


ttk.Button(
    distance_frame, text="Apply Thresholds", command=apply_thresholds
).grid(row=0, column=4, padx=10)

# =========================================================
# CONTROL PANEL
# =========================================================

control_frame = ttk.LabelFrame(root, text="Control Panel", padding=8)
control_frame.pack(fill="x", padx=10, pady=5)

ttk.Button(
    control_frame, text="AUTO Mode",
    command=lambda: send_command("MODE,AUTO")
).pack(side="left", padx=4)

ttk.Button(
    control_frame, text="MANUAL Mode",
    command=lambda: send_command("MODE,MANUAL")
).pack(side="left", padx=4)

ttk.Button(
    control_frame, text="Ref 0 (80°)",
    command=lambda: send_command("REF,0")
).pack(side="left", padx=4)

ttk.Button(
    control_frame, text="Ref 40 (120°)",
    command=lambda: send_command("REF,40")
).pack(side="left", padx=4)

ttk.Button(
    control_frame, text="Ref 70 (150°)",
    command=lambda: send_command("REF,70")
).pack(side="left", padx=4)

# =========================================================
# LIVE GRAPHS
# =========================================================

graphs_frame = ttk.LabelFrame(
    root, text="Live Performance Graphs", padding=5
)
graphs_frame.pack(fill="both", expand=True, padx=10, pady=5)

figure = Figure(figsize=(10, 6), dpi=100)
figure.subplots_adjust(
    left=0.09, right=0.96, top=0.93, bottom=0.10,
    hspace=0.42, wspace=0.28
)

ax_distance = figure.add_subplot(221)
ax_error = figure.add_subplot(222)
ax_output = figure.add_subplot(223)
ax_feedback = figure.add_subplot(224)

ax_distance.set_title("Ultrasonic Distance vs Time")
ax_distance.set_xlabel("Time (s)")
ax_distance.set_ylabel("Distance (cm)")
ax_distance.grid(True, alpha=0.3)
distance_line, = ax_distance.plot([], [], label="Distance", linewidth=1.8)
far_line = ax_distance.axhline(30, linestyle="--", label="Far boundary")
mid_line = ax_distance.axhline(20, linestyle="--", label="Middle boundary")
ax_distance.legend(loc="upper right", fontsize=8)

ax_error.set_title("PID Error vs Time")
ax_error.set_xlabel("Time (s)")
ax_error.set_ylabel("Error")
ax_error.grid(True, alpha=0.3)
error_line, = ax_error.plot([], [], label="Error", linewidth=1.8)
ax_error.axhline(0, linestyle="--", linewidth=1)
ax_error.legend(loc="upper right", fontsize=8)

ax_output.set_title("PID Term Sum (P + I + D) vs Time")
ax_output.set_xlabel("Time (s)")
ax_output.set_ylabel("P + I + D")
ax_output.grid(True, alpha=0.3)
output_line, = ax_output.plot(
    [], [], label="P + I + D", linewidth=1.8
)
ax_output.axhline(0, linestyle="--", linewidth=1)
ax_output.legend(loc="upper right", fontsize=8)

ax_feedback.set_title("AS5600 Angle and Servo Command")
ax_feedback.set_xlabel("Time (s)")
ax_feedback.set_ylabel("Angle (degrees)")
ax_feedback.grid(True, alpha=0.3)
angle_line, = ax_feedback.plot(
    [], [], label="AS5600 angle", linewidth=1.8
)
servo_line, = ax_feedback.plot(
    [], [], label="Servo command", linewidth=1.8
)
ax_feedback.legend(loc="upper right", fontsize=8)

plot_canvas = FigureCanvasTkAgg(figure, master=graphs_frame)
plot_canvas.get_tk_widget().pack(fill="both", expand=True)

# =========================================================
# SERIAL LOG
# =========================================================

log_frame = ttk.LabelFrame(root, text="Serial Log", padding=5)
log_frame.pack(fill="x", padx=10, pady=5)

log_text = tk.Text(log_frame, height=4, state="disabled", wrap="word")
log_text.pack(fill="x")


def log_message(message):
    log_text.config(state="normal")
    log_text.insert("end", message + "\n")
    line_count = int(log_text.index("end-1c").split(".")[0])
    if line_count > 150:
        log_text.delete("1.0", "31.0")
    log_text.see("end")
    log_text.config(state="disabled")


# =========================================================
# PROCESS INCOMING TELEMETRY
# =========================================================

def process_serial_data():
    changed = False

    while True:
        try:
            line = data_queue.get_nowait()
        except queue.Empty:
            break

        if line.startswith("ERROR,"):
            log_message(line)
            continue

        if not line.startswith("DATA,"):
            log_message("RX: " + line)
            continue

        try:
            fields = line.split(",")

            # DATA,time,distance,targetAngle,measuredAngle,error,P,I,D,
            # servoCommand,mode
            if len(fields) != 11:
                log_message(
                    f"Unexpected telemetry format ({len(fields)} fields): {line}"
                )
                continue

            timestamp_ms = float(fields[1])
            distance = float(fields[2])
            target_angle = float(fields[3])
            measured_angle = float(fields[4])
            error = float(fields[5])
            p_term = float(fields[6])
            i_term = float(fields[7])
            d_term = float(fields[8])
            servo_command = float(fields[9])
            mode = fields[10]

            # The firmware sends P, I and D terms, not a separate final
            # PID output field. Their sum is shown as a term-sum estimate.
            pid_term_sum = p_term + i_term + d_term

            reading_vars["Distance (cm)"].set(f"{distance:.2f}")
            reading_vars["Target Angle"].set(f"{target_angle:.2f}°")
            reading_vars["AS5600 Angle"].set(
                f"{measured_angle:.2f}°" if measured_angle >= 0 else "Invalid"
            )
            reading_vars["PID Error"].set(f"{error:.2f}")
            reading_vars["PID Output (P+I+D)"].set(f"{pid_term_sum:.2f}")
            reading_vars["Servo Position"].set(f"{servo_command:.0f}°")
            reading_vars["Mode"].set(mode)

            elapsed = timestamp_ms / 1000.0

            with data_lock:
                history["time"].append(elapsed)
                history["distance"].append(
                    distance if distance >= 0 else float("nan")
                )
                history["error"].append(error)
                history["pid_sum"].append(pid_term_sum)
                history["angle"].append(
                    measured_angle if measured_angle >= 0 else float("nan")
                )
                history["servo"].append(servo_command)

            changed = True

        except (ValueError, IndexError) as exc:
            log_message("Telemetry parse error: " + str(exc))

    if changed:
        update_graphs()

    if running:
        root.after(50, process_serial_data)


# =========================================================
# UPDATE GRAPHS
# =========================================================

def update_graphs():
    with data_lock:
        t = list(history["time"])
        distance = list(history["distance"])
        error = list(history["error"])
        pid_sum = list(history["pid_sum"])
        angle = list(history["angle"])
        servo = list(history["servo"])

    if not t:
        return

    first_time = t[0]
    x = [value - first_time for value in t]

    distance_line.set_data(x, distance)
    error_line.set_data(x, error)
    output_line.set_data(x, pid_sum)
    angle_line.set_data(x, angle)
    servo_line.set_data(x, servo)

    try:
        far_line.set_ydata([float(far_var.get())] * 2)
        mid_line.set_ydata([float(mid_var.get())] * 2)
    except ValueError:
        pass

    xmin = max(0, x[-1] - 30)
    xmax = max(10, x[-1])

    for ax in (ax_distance, ax_error, ax_output, ax_feedback):
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)
        ax.set_xlim(xmin, xmax)

    plot_canvas.draw_idle()


# =========================================================
# CLEAN SHUTDOWN
# =========================================================

def close_app():
    global running, ser
    running = False

    try:
        if ser and ser.is_open:
            ser.close()
    except Exception:
        pass

    root.destroy()


root.protocol("WM_DELETE_WINDOW", close_app)

refresh_ports()

reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

process_serial_data()
root.mainloop()
