
import serial

# Open a serial connection to the Arduino (change the port name as needed)
arduino = serial.Serial('COM4', 9600)
def read_distance():
    # Read data from the Arduino and decode it as a UTF-8 string
    data = arduino.readline().decode('utf-8').strip()

    # Split the received string using a comma as the delimiter
    parts = data.split(",")

    # Check if we have received two parts (i.e., two values separated by a comma)
    if len(parts) == 3:
        try:
            # Try to fetch the 3 distances received from the ultrasound sensors
            dist1 = float(parts[0])
            dist2 = float(parts[1])
            dist3 = float(parts[2])
            return dist1, dist2, dist3  # Return the 3 distance values
        except (ValueError, IndexError):
            pass

    # If there was an error or we didn't receive valid data, return None
    return None

while True:
    # Call the read_two_values function to read and process the data from the Arduino
    values = read_distance()

    # Check if valid values were received
    if values is not None:
        # If valid values are received, unpack the tuple
        dist1, dist2, dist3 = values
        
        # Print the received values
        print("distance 1:" + str(dist1))
        print("distance 1:" + str(dist2))
        print("distance 1:" + str(dist3))
