import serial
import time

# Open a serial connection to the Arduino
arduino = serial.Serial(port='COM4', baudrate=9600, timeout=0.1)

def read_distances_values():
    # Read data from the Arduino and decode it as a UTF-8 string
    data = arduino.readline().decode('utf-8').strip()

    # Split the received string using a comma as the delimiter
    parts = data.split(",")

    # Check if we have received two parts (i.e., two values separated by a comma)
    if len(parts) == 3:
        try:
            # Try to convert the first part to an integer and the second part to a float
           distance1 = float(parts[0])
           distance2 = float(parts[1])
           distance3 = float(parts[2])
           return distance1, distance2, distance3  # Return the three values
        except (ValueError, IndexError):
            pass

    # If there was an error or we didn't receive valid data, return None
    return None

while True:
    # Call the read_two_values function to read and process the data from the Arduino
    values = read_distances_values()

    # Check if valid values were received
    if values is not None:
        # If valid values are received, unpack the tuple
        value1, value2, value3 = values

        # Print the received values
        print("Received values:", value1, value2,value3)
        time.sleep(1)