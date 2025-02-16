import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
from datetime import datetime

def find_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return None
    for port in ports:
        print(f"Found port: {port.device}")
        if "tty" in port.device:  # Adjust this if needed to identify your specific device
            return port.device
    print("No suitable port found.")
    return None

def parse_serial_line(line, monitored_vars):
    """Parse a line from the serial input and extract monitored variables."""
    match = re.match(r"(\w+)\s*:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", line)
    if match:
        var_name, value = match.group(1), float(match.group(2))
        if var_name in monitored_vars:
            return var_name, value
    return None, None

def update_plot(frame, ser, monitored_vars, time_data, data_dict, lines):
    """Update function for real-time plotting."""
    try:
        if ser.is_open:
            line = ser.readline().decode('utf-8').strip()
            print(f"Read line: {line}")  # Print the raw serial data
            var_name, value = parse_serial_line(line, monitored_vars)
            print(f"Parsed data - var_name: {var_name}, value: {value}")  # Print parsed data
            if var_name and value is not None:
                current_time = datetime.now().strftime("%H:%M:%S")
                time_data.append(current_time)
                data_dict[var_name].append(value)
                if len(data_dict[var_name]) > 100:
                    time_data.popleft()
                    data_dict[var_name].popleft()
        else:
            print("Error: Serial port is not open")
    except Exception as e:
        print(f"Error reading serial data: {e}")
    
    for var_name, line in lines.items():
        line.set_ydata(data_dict[var_name])
        line.set_xdata(range(len(data_dict[var_name])))
    
    return lines.values()

def main(serial_port, baud_rate, monitored_vars):
    if serial_port is None:
        print("No serial port available. Please check your device connection.")
        return
    
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        if ser.is_open:
            print(f"Serial port {serial_port} opened successfully.")
        else:
            print(f"Failed to open serial port {serial_port}.")
            return
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return
    
    # Data storage
    time_data = deque([datetime.now().strftime("%H:%M:%S")] * 100, maxlen=100)
    data_dict = {var: deque([0] * 100, maxlen=100) for var in monitored_vars}
    
    # Setup Matplotlib figure
    fig, axes = plt.subplots(len(monitored_vars), 1, figsize=(8, 5 * len(monitored_vars)))
    if len(monitored_vars) == 1:
        axes = [axes]
    
    lines = {}
    for ax, var in zip(axes, monitored_vars):
        ax.set_title(var)
        ax.set_ylim(-200, 200)  # Adjust as needed
        ax.set_xlim(0, 100)
        line, = ax.plot(range(100), list(data_dict[var]), label=var)
        lines[var] = line
        ax.legend()
    
    ani = animation.FuncAnimation(fig, update_plot, fargs=(ser, monitored_vars, time_data, data_dict, lines), interval=100, blit=False, cache_frame_data=False)
    plt.show()
    
    ser.close()

if __name__ == "__main__":
    serial_port = find_port()
    if serial_port is None:
        # Allow user to manually set the serial port if auto-detection fails
        serial_port = input("Enter serial port (e.g., /dev/ttyUSB0): ")
    
    baud_rate = 1000
    monitored_vars = ["F_whl"]  # Customize
    main(serial_port, baud_rate, monitored_vars)
