import socket
import time

# ESP32 Configuration
ESP32_IP = "192.168.0.183"
ESP32_PORT = 8080
TIMEOUT_S = 0.5

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

def main():
    # Connect to ESP32
    print(f"Connecting to ESP32 at {ESP32_IP}:{ESP32_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    
    try:
        sock.connect((ESP32_IP, ESP32_PORT))
        print("Connected!")
        
        # Wait for READY message
        sock.settimeout(2.0)
        data = sock.recv(1024).decode(errors="ignore")
        print("<<", data.strip())
        
        # Send test commands
        pause = 0.1
        send_and_wait_done(sock, "LED ON", timeout_s=2)
        time.sleep(pause)
        send_and_wait_done(sock, "LED OFF", timeout_s=2)
        time.sleep(pause)

        send_and_wait_done(sock, "GET_DISTANCE", timeout_s=2)
        time.sleep(pause)

        send_and_wait_done(sock, "MOVE 1", timeout_s=10)
        time.sleep(pause)
        # send_and_wait_done(sock, "MOVE -1", timeout_s=10)
        # time.sleep(pause)

        # send_and_wait_done(sock, "TURN 90", timeout_s=10)
        # time.sleep(pause)
        # send_and_wait_done(sock, "TURN -90", timeout_s=10)
        # time.sleep(pause)
        
        # Uncomment to test movement commands
        # for _ in range(3):
        #     send_and_wait_done(sock, "MOVE -1.5", timeout_s=10)
        #     time.sleep(pause)
        #     send_and_wait_done(sock, "MOVE 1.5", timeout_s=10)
        #     time.sleep(pause)
        # for _ in range(2):
        #     send_and_wait_done(sock, "TURN 90", timeout_s=10)
        #     time.sleep(pause)
        #     send_and_wait_done(sock, "TURN -90", timeout_s=10)
        #     time.sleep(pause)
        
        send_and_wait_done(sock, "LED ON", timeout_s=2)
        time.sleep(pause)
        send_and_wait_done(sock, "LED OFF", timeout_s=2)
        
        print("All commands completed successfully!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()
        print("Connection closed.")

if __name__ == "__main__":
    main()
