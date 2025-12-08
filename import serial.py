import serial
import time
import matplotlib.pyplot as plt

# ----------- Configure your COM port here -------------
# Example:
# Windows: "COM3"
# Linux/Mac: "/dev/ttyUSB0"
port = "COM3"
# ------------------------------------------------------

ser = serial.Serial(port, 115200, timeout=1)

left_data = []
mid_data = []
right_data = []
time_data = []

print("Recording 10 seconds of sensor data...")
start_time = time.time()

while time.time() - start_time < 10:
    line = ser.readline().decode().strip()

    if line:
        try:
            L, M, R = map(int, line.split(","))
            current_time = time.time() - start_time

            left_data.append(L)
            mid_data.append(M)
            right_data.append(R)
            time_data.append(current_time)

        except:
            pass  # skip corrupted lines

ser.close()
print("Done recording!")

# ---- Plotting ----
plt.figure(figsize=(10, 5))
plt.plot(time_data, left_data, label="Left Sensor")
plt.plot(time_data, mid_data, label="Middle Sensor")
plt.plot(time_data, right_data, label="Right Sensor")

plt.xlabel("Time (seconds)")
plt.ylabel("IR Reading (0 = black, 1 = white)")
plt.title("Elegoo IR Sensor Noise Demonstration")
plt.legend()
plt.tight_layout()

# Save image
plt.savefig("ir_noise_plot.png")

plt.show()

print("Graph saved as ir_noise_plot.png")
