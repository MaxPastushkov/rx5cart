# Written by Max Pastushkov in August 2021

import serial
import sys, os, argparse
import zlib

# Due to size of serial buffer on teensy
write_blocksize = 64

operation = ""
filepath = ""
device = "/dev/ttyACM0"

parser = argparse.ArgumentParser()

parser.add_argument("-d", "--device", help="device file to use")

group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("-r", "--read", help="file to read data into", metavar="<output file>")
group.add_argument("-w", "--write", help="file to write from", metavar="<input file>")

args = parser.parse_args()

if (args.read):
    operation = "wb"
    filepath = args.read
elif (args.write):
    operation = "rb"
    filepath = args.write
    
if (args.device):
    device = args.device

try:
    ser = serial.Serial(device, 115200)
except serial.SerialException as e:
    print("Error opening device: " + str(e))
    sys.exit(1)

buff = b''
    
try:
    with open(filepath, operation) as f:
        if operation == "wb":
            
            ser.write(b'r') # Send start signal
            
            print("Reading data...")
            buff = ser.read(0x20000) # Read data
            
            # Send checksum
            print("Calculating checksum...")
            crc = zlib.crc32(buff) & 0xFFFFFFFF
            print("Checksum is " + str(crc))
            
            print("Sending checksum...")
            ser.write(crc.to_bytes(4, 'little'))
            
            print("Waiting for status...")
            status = ser.read(1)
            print("Received status: " + str(status))
            
            if status != b'\x01':
                print("Verification Error: checksum does not match")
                ser.close()
                sys.exit(1)
                
            f.write(buff) # Save data
            
            print("Successfully received data from device")
        
        elif operation == "rb":
            
            if os.path.getsize(filepath) != 0x20000:
                print("Error: invalid file size")
                ser.close()
                sys.exit(1)
            buff = f.read(0x20000)
            
            crc = zlib.crc32(buff) & 0xFFFFFFFF # Calculate checksum
            crc = crc.to_bytes(4, 'little')
            
            ser.write(b'w') # Send start signal
            
            print("Sending data...")
            for i in range(0, 0x20000, write_blocksize):
                
                blockbuff = buff[i:i+write_blocksize] # Get single chunk
                ser.write(blockbuff)
                ser.read(1) # Wait for acknowledgement
            
            print("Waiting for checksum...")
            devicecrc = ser.read(4) # Get recipient checksum
            print("Received checksum: " + str(devicecrc))
            
            if devicecrc != crc:
                print("Verification Error: checksum does not match")
                print("Expected: " + str(crc))
                ser.close()
                sys.exit(1)
            
            print("Successfully sent data to device")
        
except IOError as e:
    print("Could not open file: " + str(e))
    ser.close()
    sys.exit(1)

ser.close()
