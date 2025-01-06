import serial
import time

def main():
    ser = serial.Serial('COM6', 6000000, timeout=1)  # Open COM5 with baud rate 9600

    if not ser.is_open:
        print("Failed to open serial port.")
        return

    # Send data
    #data_to_send = "1"
    #ser.write(data_to_send.encode())
    #print(f"Sent: {data_to_send}")
    
    # Read data
    received_data = b''
    while True:
    #for i in range(3):
        ct = 0
        received_data=b''
        while True:
            ct += 1
            byte = ser.read()
            if byte ==b'\n':
                break
            received_data += byte

        rec = str({received_data.decode('utf-8')})
        if(rec==str({'10000000\r'})):
            start = time.time_ns()
        if(rec==str({'10500000\r'})):
            end = time.time_ns()
            break
    print((end-start)/1000000000)
        #rec_str = rec[2]+rec[3]+rec[4]+rec[5]+rec[6]+rec[7]+rec[8]+rec[9]
        #rec_int = int(rec_str)
        #if(rec_int == 10500000):
        #    end = time.time_ns()
        #    break
    #while True:
        #print(rec_int+'\n')
        #length = (end - start)/1000;
        #print("Ittook", length, "microseconds!"+'\n')

    ser.close()  # Close the serial port

if __name__ == "__main__":
    main()
