# ================= IMPORTS =======================
import serial
import subprocess
import csv
import numpy as np
import matplotlib.pyplot as plt
import time 

# ================= INITIALIZE VARIABLES =======================
SERIAL_PORT = "COM10"  # Change this to your serial port
BAUD_RATE = 230400
RECORD_SAMPLES = 6400  # Number of samples to read
TRIGGER_DISTANCE = 10  # Distance in cm to trigger recording
AUDIO_TIMEOUT = 2  # Timeout for audio recording
txt_file = "raw_adc_values.txt"  # File to save raw ADC values
# C program to generate WAV file
csv_file = "raw_adc_values.csv"  # File to save raw ADC values in CSV format
png_file = "waveform.png"  # File to save waveform image


# ==================== FUNCTION DEFINITIONS =====================
""" Menu: This function displays the menu operation options to the user.
    Based on the user's choice, it will call the appropriate function.
    Input: None
    Output: None
"""
def menu():
    print("\n =================MAIN MENU=====================")
    print("\n Please select an option:")
    print("0. Exit")
    print("1. Manual Recording Mode")
    print("2. Distance Trigger Mode")
    while True:
        try:
            global choice
            choice = int(input("Enter your preferred operation: "))
            if choice == 0:
                run_serial()
                print("Exiting...")
                exit(0)
            elif choice == 1:
                print("Manual Recording Mode selected.")
                manual_recording_mode()
            elif choice == 2:
                print("Distance Trigger Mode selected.")
                distance_trigger_mode()
            else:
                print("Invalid choice. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

# ==================== MANUAL RECORDING MODE =====================
# Manual Recording Mode
""" This function displays the manual recording mode options to the user.
    Input: None
    Output: None
"""
def manual_recording_mode():
    print("==================== MANUAL RECORDING MODE =====================")
    global duration
    duration = int(input(f"\nEnter the length of audio to record (e.g 8): "))
    global samples
    samples = duration * RECORD_SAMPLES
    ser = run_serial()
    save_txt_file(ser, samples)
    ser.close()
    output_options(duration)

def dtm_to_manual_recording_mode():
    global choice
    global dtm_duration
    choice = 1
    print("==================== MANUAL RECORDING MODE =====================")
    duration = int(input(f"\nEnter the length of audio to record (e.g 8): "))
    samples = (duration) * RECORD_SAMPLES
    ser = run_serial()
    append_txt_file(ser, samples)
    ser.close()
    output_options(dtm_duration+duration)

def manual_to_dtm_recording_mode():
    choice = 2
    print("==================== DISTANCE TRIGGER MODE =====================")

    try:
        global max_distance
        max_distance = int(input("Enter max trigger distance in cm : "))
    except ValueError:
        print("Invalid input. Using default distance 10cm.")
        max_distance = 10.0

    run_serial()
    capture_uart_dtm()
    output_options(dtm_duration+duration)

# ==================== DISTANCE TRIGGER MODE =====================
# Distance Trigger Mode Menu
""" This function displays the distance trigger mode options to the user.
    Input: None
    Output: None
"""

def distance_trigger_mode():
    import time
    
    print("==================== DISTANCE TRIGGER MODE =====================")

    try:
        global max_distance
        max_distance = int(input("Enter max trigger distance in cm : "))
    except ValueError:
        print("Invalid input. Using default distance 10cm.")
        max_distance = 10.0

    run_serial()
    capture_uart_dtm()
    output_options(dtm_duration)
    # Send mode selection and distance to STM

def capture_uart_dtm():
# Serial port configuration
    ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, bytesize=8, parity="N", stopbits=1, timeout=0.01)
    output_filename = 'dtm_output.txt'

    # Detection configuration
    idle_timeout = 0.001  # Time (in seconds) to consider UART idle
    last_data_time = time.time()
    writing = False
    file = None

    # Timing tracking variables
    global dtm_duration
    dtm_duration = 0  # Total active writing duration
    start_time = None
    print("Starting data capture in distance trigger mode...")
    try:
        while True:
            data = ser.read(2)  # 2 bytes per sample from STM32

            if data:
                # Data received, reset idle timer
                last_data_time = time.time()

                # Start writing if not already
                if not writing:
                    print("Data started, opening file...")
                    file = open(output_filename, 'ab')  # Append mode
                    ## start/resume dtm_duration
                    start_time = time.time()  # Start timing
                    writing = True

                file.write(data)

            elif writing and ((time.time() - last_data_time) > idle_timeout):
                # No data received for a while, close file
                print("Idle detected, closing file...")
                ## pause dtm_duration
                if start_time is not None:
                    dtm_duration += time.time() - start_time
                    start_time = None
                file.close()
                writing = False

    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        if file and not file.closed:
            file.close()
        ser.close()

        # Final pause timing
        if writing and start_time is not None:
            dtm_duration += time.time() - start_time

        print(f"Total DTM active duration: {dtm_duration:.2f} seconds")
        print("DTM data capture completed. Switch to manual recording mode or exit?")
        print("Press '1' to switch to manual recording mode or '0' to exit.")
        while True:
            try:
                choice = int(input("Enter your choice: "))
                if choice == 1:
                    print("Switching to manual recording mode...")
                    dtm_to_manual_recording_mode()
                elif choice == 0:
                    print("Exiting...")
                    # Overwrite raw_ADC_value.txt with the collected data
                    overwrite_raw_adc()
                else:
                    print("Invalid choice. Please try again.")
            except ValueError:
                print("Invalid input. Please enter a number.")

def overwrite_raw_adc():
    with open('dtm_output.txt', 'rb') as src_file:
        data = src_file.read()

    with open('raw_adc_values.txt', 'wb') as dest_file:
        dest_file.write(data)
    open('dtm_output.txt', 'wb').close()

    print("raw_ADC_value.txt has been overwritten with contents from dtm_output.txt")

# ==================== OUTPUT OPTIONS =====================
# Output Options
""" This function displays the output options to the user and allows multiple selections.
    Input: None
    Output: None
"""
global valid_options
valid_options = [0, 1, 2, 3, 4]
def output_options(duration):
    """ This function displays the output options to the user and allows multiple selections.
        Input: duration
        Output: None
    """
    print("\n =================OUTPUT OPTIONS=====================")
    print("\nPlease select one or more output options by entering their numbers e.g (1,2,3):")
    print("\n0. Exit")
    print("\n1. Generate a .wav file")
    print("\n2. Generate a .csv file")
    print("\n3. Generate a waveform (.png) file")
    print("\n4. Generate an fft file")


    while True:
        try:
            inputs = input("Enter your preferred output option(s): ")
            selected_options = [int(i) for i in inputs.split(",")]
            if not all(option in valid_options for option in selected_options):
                print("Invalid input. Please enter numbers from the available options.")
                continue

            # Check if the user wants to exit
            if 0 in selected_options:
                print("Exiting...")
                exit(0)
            if 1 in selected_options:
                print("Generating .wav file...")
                compile_and_generate_wav(duration)
            if 2 in selected_options:
                print("Generating .csv file...")
                generate_csv_file()
            if 3 in selected_options:
                print("Generating waveform image...")
                plot_waveform()
            if 4 in selected_options:
                print("Generating FFT plot...")
                plot_fft()
            break
        except ValueError:
            print("Invalid input. Please enter numbers separated by commas.")




def run_serial():
    """ This function sets up the serial connection and reads data from the serial port.
        Input: None
        Output: None
    """
    ser = serial.Serial(port=SERIAL_PORT, baudrate=230400, bytesize=8, parity="N", stopbits=1, timeout=5)
    if choice == 2:
        print("Setting up serial connection...")
        cmd = str(choice).encode('utf-8')  # '1' becomes b'1'
        print(f"Transmitting command: {cmd}")
        ser.write(cmd)
        time.sleep(1)
        cmd = str(max_distance).encode('utf-8')  # '1' becomes b'1'
        print(f"Transmitting command: {cmd}")
        ser.write(cmd)
    else:
        print("Setting up serial connection...")
        # Serial UART parameters
        cmd = str(choice).encode('utf-8')  # '1' becomes b'1'
        print(f"Transmitting command: {cmd}")
        ser.write(cmd)
        print("Connected to:", ser.name)
    return ser


def record_audio(samples):
    """ This function records audio data from the serial port and saves it to a .txt file.
        Input: samples - Number of samples to record
        Output: None
    """
    print("Recording audio...")
    ser = run_serial()
    save_txt_file(ser, samples)

def save_txt_file(ser,samples):
    """ This function saves the received data to a .txt file.
        Input: 
          ser - Serial object
          samples - Number of samples to record
        Output: None
    """
    print("Saving data to .txt file...")
    with open(txt_file, "wb") as file_1:
        print("Receiving data...")
        for i in range(samples):
            chunk = ser.read(2)
            file_1.write(chunk)
    print("Recording completed.")
    print(f"Data saved to {txt_file}")

def append_txt_file(ser,samples):
    """ This function saves the received data to a .txt file.
        Input: 
          ser - Serial object
          samples - Number of samples to record
        Output: None
    """
    print("Saving data to .txt file...")
    with open('dtm_output.txt', "ab") as file_1:
        print("Receiving data...")
        for i in range(samples):
            chunk = ser.read(2)
            file_1.write(chunk)
    print("Recording completed.")
    overwrite_raw_adc()
    print(f"Data saved to {txt_file}")


def generate_csv_file():
    """ This function generates a .csv file from the raw ADC values.
        Input: None
        Output: None
    """
    print("Generating .csv file...")

    # Assume txt_file and csv_file are defined somewhere globally or passed in
    with open(txt_file, "rb") as f_in, open(csv_file, "w", newline="") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(["Sample Rate: 16000"])
        writer.writerow(["Raw ADC Values"])
        index = 0
        while True:
            chunk = f_in.read(2)
            if not chunk:
                break
            # Convert the bytes to an integer
            value = int.from_bytes(chunk, byteorder='little', signed=False)
            # Write the index and value to the CSV file
            writer.writerow([index, value])
            index += 1

    print(f"CSV file generated: {csv_file}")


def plot_waveform():
    """ This function generates a waveform image from the raw ADC values.
        Input: None
        Output: None
    """
    print("Generating waveform image...")
    with open(txt_file, "rb") as f:
        data = f.read()
        # Convert the bytes to a list of integers
        values = [int.from_bytes(data[i:i+2], byteorder='little', signed=False) for i in range(0, len(data), 2)]
        time_axis = np.arange(0, len(values)) / RECORD_SAMPLES  # Assuming a sample rate of 16000 Hz
        plt.figure(figsize=(10, 4))
        plt.plot(time_axis, values)
        plt.title("Amplitude vs time")
        plt.xlabel("Time (s)")
        plt.ylabel("Amplitude (V)")
        plt.grid(True)
        plt.savefig(png_file)
        plt.close()
    print(f"Waveform image generated: {png_file}")

def plot_fft(): 
    """ This function generates a FFT plot from the raw ADC values.
        Input: None
        Output: None
    """
    print("Generating FFT plot...")
    with open(txt_file, "rb") as f:
        data = f.read()
        # Convert the bytes to a list of integers
        values = [int.from_bytes(data[i:i+2], byteorder='little', signed=False) for i in range(0, len(data), 2)]
        
        # Perform FFT
        fft_result = np.fft.fft(values)
        freq = np.fft.fftfreq(len(values), 1/RECORD_SAMPLES)
        plt.figure(figsize=(10, 4))
        plt.plot(freq[:len(freq)//2], np.abs(fft_result)[:len(fft_result)//2])
        plt.title("FFT of the signal")
        plt.xlabel("Frequency (Hz)")
        plt.ylabel("Magnitude")
        plt.grid(True)
        plt.savefig("fft_plot.png")
        plt.close()
    print("FFT plot generated: fft_plot.png")


def compile_and_generate_wav(duration):
    """ This function compiles the C program and generates a .wav file.
        Input: None
        Output: None
    """

    # Compile the C program using GCC
    print("Compiling C program...")
    compile_result = subprocess.run(["gcc", "generate_wav.c", "-o", "generate_wav.exe"], capture_output=True, text=True)

    # Check if compilation was successful
    if compile_result.returncode != 0:
        print("Compilation failed:\n", compile_result.stderr)
        exit(1)
    else:
        print("Compilation succeeded.")

    # Run the executable to generate the wav file
    print("Running the C program to convert to .wav...")
    run_result = subprocess.run(["./generate_wav.exe",str(duration)],capture_output=True,text=True)

    # Check if the executable ran successfully
    if run_result.returncode == 0:
        print("WAV file generated successfully.")
    else:
        print("Error during execution:\n", run_result.stderr)


# Main function to run the program
def main():
    while True:
        try:
            menu()
        except KeyboardInterrupt:
            print("\nExiting...")
            break


if __name__ == "__main__":
    main()
# End of the code
# ==================== END OF CODE =====================