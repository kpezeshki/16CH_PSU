import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# Set up serial connection (adjust port as needed)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# Maximum ratings (less than datasheet spec)
MAX_VGS = 0.5  # Maximum Vgs (V)
MIN_VGS = -5.0  # Minimum Vgs (V)
MAX_VDS = 5.0  # Maximum Vds (V)
MAX_IDS = 0.05  # Maximum Ids (A)

# Sweep parameters (adjust as needed)
VGS_START = 0
VGS_STOP = 0.34
VGS_POINTS = 21  # Number of Vgs points
VDS_START = 0
VDS_STOP = 3
VDS_POINTS = 21  # Number of Vds points

# Channel assignments
VGS_CHANNEL = 1
VDS_CHANNEL = 2

def send_command(cmd):
    ser.reset_input_buffer()  # Clear input buffer before sending command
    ser.write(f"{cmd}\n".encode())
    time.sleep(0.1)
    response = ser.readline().decode().strip()
    ser.reset_input_buffer()  # Clear any remaining data in the buffer
    return response

def set_voltage(channel, voltage):
    send_command(f"SETV {channel} {voltage:.3f}")
    time.sleep(0.1)  # Allow 0.1 seconds for the system to stabilize after setting voltage

def read_current(channel):
    try:
        response = send_command(f"READI {channel}")
        return float(response)
    except ValueError:
        print(f"ERROR: Malformed current reading received: {response}")
        turn_off_supplies()
        raise

def turn_off_supplies():
    set_voltage(VGS_CHANNEL, 0)
    set_voltage(VDS_CHANNEL, 0)
    print("WARNING: Supplies turned off due to error or excessive current!")

def sweep_vgs_vds():
    vgs_values = np.linspace(VGS_START, VGS_STOP, VGS_POINTS)
    vds_values = np.linspace(VDS_START, VDS_STOP, VDS_POINTS)
    ids_matrix = np.zeros((len(vgs_values), len(vds_values)))

    try:
        for i, vgs in enumerate(vgs_values):
            if vgs < MIN_VGS or vgs > MAX_VGS:
                continue

            set_voltage(VGS_CHANNEL, vgs)

            for j, vds in enumerate(vds_values):
                if vds > MAX_VDS:
                    break

                set_voltage(VDS_CHANNEL, vds)

                ids = read_current(VDS_CHANNEL)
                
                if ids > MAX_IDS:
                    turn_off_supplies()
                    return None

                ids_matrix[i, j] = ids
                print(f"Vgs={vgs:.2f}V, Vds={vds:.2f}V, Ids={ids:.6f}A")

        return vgs_values, vds_values, ids_matrix

    except Exception as e:
        print(f"An error occurred during the sweep: {str(e)}")
        turn_off_supplies()
        return None

def plot_results(vgs_values, vds_values, ids_matrix):
    plt.figure(figsize=(10, 8))
    plt.imshow(ids_matrix, origin='lower', aspect='auto', 
               extent=[vds_values[0], vds_values[-1], vgs_values[0], vgs_values[-1]])
    plt.colorbar(label='Ids (A)')
    plt.xlabel('Vds (V)')
    plt.ylabel('Vgs (V)')
    plt.title('Ids vs Vgs and Vds')
    plt.show()

try:
    print("Starting Vgs-Vds sweep...")
    result = sweep_vgs_vds()
    if result is not None:
        vgs_values, vds_values, ids_matrix = result
        print("Sweep completed successfully.")
        plot_results(vgs_values, vds_values, ids_matrix)
    else:
        print("Sweep aborted due to error or excessive current.")
    
finally:
    # Ensure voltages are set to 0V when done
    turn_off_supplies()
    ser.close()