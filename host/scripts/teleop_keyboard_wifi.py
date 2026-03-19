import socket
import time
import threading
import queue
from pynput import keyboard

# ESP32 Configuration
ESP32_IP = "192.168.0.183" 
ESP32_PORT = 8080
TIMEOUT_S = 0.5

MAX_MOVE_M = 2.0          # cap per key hold
MAX_TURN_DEG = 180.0      # cap per key hold

MOVE_TIMEOUT_S = 20.0
TURN_TIMEOUT_S = 20.0

SPEED_MPS = 2.0          # meters per second for MOVE mapping
ANGULAR_SPEED_DPS = 40.0 # degrees per second for TURN mapping

MIN_HOLD_S = 0.03  # ignore super-short noise taps

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def send_and_wait_done(sock: socket.socket, cmd: str, timeout_s: float = 20.0):
    """Send command and wait for DONE response."""
    sock.sendall((cmd.strip() + "\n").encode("utf-8"))
    
    sock.settimeout(TIMEOUT_S)
    deadline = time.time() + timeout_s
    buffer = ""
    
    while time.time() < deadline:
        try:
            data = sock.recv(1024)
            if not data:
                raise ConnectionError("Connection closed by ESP32")
            
            buffer += data.decode(errors="ignore")
            lines = buffer.split("\n")
            buffer = lines[-1]  # Keep incomplete line in buffer
            
            for line in lines[:-1]:
                text = line.strip()
                if text:
                    print("<<", text)
                if text == "DONE":
                    return
        except socket.timeout:
            continue
    
    raise TimeoutError(f"Timed out waiting DONE for: {cmd}")

print(f"Connecting to ESP32 at {ESP32_IP}:{ESP32_PORT}...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(5.0)
sock.connect((ESP32_IP, ESP32_PORT))
print("Connected!")

# Wait for READY message
sock.settimeout(2.0)
data = sock.recv(1024).decode(errors="ignore")
print("<<", data.strip())

# --- Command worker (so keyboard thread never blocks on DONE) ---
cmd_q: "queue.Queue[tuple[str, float]]" = queue.Queue()
stop_event = threading.Event()

def worker():
    while not stop_event.is_set():
        try:
            cmd, timeout_s = cmd_q.get(timeout=0.1)
        except queue.Empty:
            continue

        try:
            print(">>", cmd)
            send_and_wait_done(sock, cmd, timeout_s=timeout_s)
        except Exception as e:
            print("!!", repr(e))
        finally:
            cmd_q.task_done()

worker_thread = threading.Thread(target=worker, daemon=True)
worker_thread.start()

# --- Key hold tracking ---
pressed_at: dict[str, float] = {}

def key_id(key) -> str | None:
    # Normalize only keys we care about
    if key in (keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right):
        return str(key)
    if hasattr(key, "char") and key.char:
        return key.char.lower()
    if key == keyboard.Key.space:
        return "space"
    return None

def on_press(key):
    kid = key_id(key)
    if kid is None:
        return

    # Quit
    if kid == "q":
        stop_event.set()
        return False
    
    if kid == "n":
        cmd_q.put(("LED ON", MOVE_TIMEOUT_S))
        return

    if kid == "f":
        cmd_q.put(("LED OFF", MOVE_TIMEOUT_S))
        return

    # Panic (clear queue)
    if kid == "space":
        # Drop queued commands (doesn't interrupt currently running one)
        try:
            while True:
                cmd_q.get_nowait()
                cmd_q.task_done()
        except queue.Empty:
            pass
        print("!! queue cleared")
        return

    # Start timing for arrows (ignore repeats)
    if kid not in pressed_at:
        pressed_at[kid] = time.time()

def on_release(key):
    kid = key_id(key)
    if kid is None:
        return

    if kid not in pressed_at:
        return

    t0 = pressed_at.pop(kid)
    dt = time.time() - t0
    if dt < MIN_HOLD_S:
        return

    # Map to action
    if kid == str(keyboard.Key.up):
        meters = clamp(SPEED_MPS * dt, 0.0, MAX_MOVE_M)
        if meters > 0:
            cmd_q.put((f"MOVE {meters:.3f}", MOVE_TIMEOUT_S))

    elif kid == str(keyboard.Key.down):
        meters = clamp(SPEED_MPS * dt, 0.0, MAX_MOVE_M)
        if meters > 0:
            cmd_q.put((f"MOVE {-meters:.3f}", MOVE_TIMEOUT_S))

    elif kid == str(keyboard.Key.left):
        deg = clamp(ANGULAR_SPEED_DPS * dt, 0.0, MAX_TURN_DEG)
        if deg > 0:
            cmd_q.put((f"TURN {-deg:.1f}", TURN_TIMEOUT_S))

    elif kid == str(keyboard.Key.right):
        deg = clamp(ANGULAR_SPEED_DPS * dt, 0.0, MAX_TURN_DEG)
        if deg > 0:
            cmd_q.put((f"TURN {deg:.1f}", TURN_TIMEOUT_S))

def main():
    print("Keyboard teleop started.")
    print("Hold arrows -> release sends MOVE/TURN scaled by hold time.")
    print("Q = quit, Space = clear queued commands.")

    # small “alive” blink
    cmd_q.put(("LED ON", 2.0))
    cmd_q.put(("LED OFF", 2.0))

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    # shutdown
    stop_event.set()
    try:
        cmd_q.join()
    except Exception:
        pass
    finally:
        sock.close()
        print("Connection closed.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop_event.set()
        try:
            sock.close()
            print("Connection closed.")
        except Exception:
            pass