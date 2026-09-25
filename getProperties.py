import socket
import csv
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 5500
CSV_FILE = "flight_data.csv"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("Waiting for FlightGear...")

# Wait for the first packet
data, addr = sock.recvfrom(4096)

print(f"FlightGear detected from {addr}. Logging started.")

header = [
    "time",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "p_degps",
    "q_degps",
    "r_degps",
    "ax_body_fps2",
    "ay_body_fps2",
    "az_body_fps2",
    "airspeed_kt",
    "u_body_fps",
    "v_body_fps",
    "w_body_fps",
    "altitude_ft"
]

with open(CSV_FILE, "w", newline="") as csvfile:

    writer = csv.writer(csvfile)
    writer.writerow(header)

    start_time = time.perf_counter()

    try:
        # Process first packet
        line = data.decode("utf-8").strip()
        values = line.split(",")

        if len(values) != 14:
            print(f"Warning: expected 14 values, received {len(values)}")
        else:
            writer.writerow([0.0] + values)
            csvfile.flush()

        print("Press Ctrl+C to stop.")

        while True:

            data, addr = sock.recvfrom(4096)

            elapsed = time.perf_counter() - start_time

            line = data.decode("utf-8").strip()
            values = line.split(",")

            if len(values) != 14:
                print(
                    f"Warning: expected 14 values, "
                    f"received {len(values)}"
                )
                continue

            writer.writerow([elapsed] + values)
            csvfile.flush()

    except KeyboardInterrupt:
        print("\nLogging stopped.")

sock.close()

print(f"Data saved to {CSV_FILE}")