import pygame
import serial
import glob
import sys
import os
import re
import time

# Function to find the first USB serial port
def find_usb_serial_port():
    if sys.platform.startswith('darwin'):  # macOS
        ports = glob.glob('/dev/tty.usbmodem*')
    elif sys.platform.startswith('linux'):  # Linux
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    elif sys.platform.startswith('win'):  # Windows
        ports = ['COM%s' % (i + 1) for i in range(256)]
    else:
        print(f"Unsupported platform: {sys.platform}")
        return None
    
    for port in ports:
        try:
            print(f"Trying port: {port}")
            # Just check if we can open it
            s = serial.Serial(port, 9600, timeout=1)
            s.close()
            return port
        except (OSError, serial.SerialException):
            pass
    
    return None

# Initialize pygame
pygame.init()

# Set up the display
width, height = 800, 300
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Serial Data Display")

# Set up fonts
font = pygame.font.SysFont('Arial', 72, bold=True)

# Set up colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)

def main():
    # Find USB serial port
    port = find_usb_serial_port()
    if not port:
        print("No USB serial port found.")
        pygame.quit()
        sys.exit()
    
    print(f"Connecting to {port}")
    
    # Connect to the serial port
    try:
        ser = serial.Serial(port, 9600, timeout=1)
        print(f"Connected to {port}")
    except serial.SerialException as e:
        print(f"Error connecting to {port}: {e}")
        pygame.quit()
        sys.exit()
    
    distance = 0
    roataion = 0 # between 0 and 1
    running = True
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Try to read data from serial
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                print(f"Received: {line}")
                
                # Check if the line matches our expected format
                if ',' in line:
                    try:
                        parts = line.split(',')
                        distance = int(parts[0])
                        roataion = ((int(parts[1]) - 500) / 2500) * 100 # percentage of rotation
                    except (ValueError, IndexError) as e:
                        print(f"Error parsing values: {e}")
        except Exception as e:
            print(f"Error reading from serial: {e}")
        
        # Clear the screen
        screen.fill(BLACK)
        
        # Draw the values
        text1 = font.render(f"Value 1: {distance}", True, RED)
        text2 = font.render(f"Value 2: {roataion}", True, BLUE)
        
        # Position the text
        screen.blit(text1, (width//2 - text1.get_width()//2, height//3 - text1.get_height()//2))
        screen.blit(text2, (width//2 - text2.get_width()//2, 2*height//3 - text2.get_height()//2))
        
        # Update the display
        pygame.display.flip()
        
        # Cap the frame rate
        pygame.time.delay(50)
    
    # Clean up
    ser.close()
    pygame.quit()

if __name__ == "__main__":
    main()