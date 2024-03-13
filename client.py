import sys
import socket
import threading
from datetime import datetime
import pandas as pd

IP = '192.168.1.1' #input("IP MKR1000: ")
PORT = 23 # telnet
ADDR = (IP, PORT)
OUTPUT_FILE = 'freqs_GT.csv'

print('INSTRUCTIONS')
print('------------')
print('First send amplitude value, then send frequency values')
print('-> To set a new frequency, send the value desired')
print('-> To set a new amplitude value, send "A" and then you will be asked for a new amplitude')
print('-> To stop the movement at the extremes, send "X" and the motors will stop when the oscillation is')
print('   at -A, send X again to go to +A, send X again to leave static mode and oscillate with last A and f.')

df = pd.DataFrame(columns=['Timestamp', 'Amplitude(mm)', 'Freq(Hz)'])

connected = False

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

TERMINATOR_CHAR = '\n'

last_f = 1.0
last_A = 0

def handle_client():
    global connected, df, last_f, last_A

    connected = True

    try:
        sendAmplitude()
    except Exception:
            print('[MKR1000 SERVER] Could not send amplitude data.')
            connected = False

    while connected:
        try:
            f = input('Desired frequency (Hz), A or X = ')
            # Send frequency
            server.send((f + TERMINATOR_CHAR).encode())

            # If input is "A" then amplitude change is requested, else wait for frequency OK
            if f == "A":
                try:
                    sendAmplitude()
                except Exception:
                        print('[MKR1000 SERVER] Could not send amplitude data.')
                        connected = False
            else:
                # When MKR1000 answers with 1, f will be updated -> store f and timestamp
                msg = int.from_bytes(server.recv(1))
                if (msg):
                    if f == "X":
                         print('[MKR1000 SERVER] Oscillation stopped at a extreme.')
                    else:
                        timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")
                        last_f = f
                        df = pd.concat([df, pd.DataFrame([{'Timestamp': timestamp, 'Amplitude(mm)': last_A, 'Freq(Hz)': f}])])
                else:
                    raise Exception('[FREQ. CHANGE] MKR1000 did not send CHANGE_OK value')
            
        except Exception:
            print('[MKR1000 SERVER] Could not send frequency data.')
            connected = False

    stopServer()
    df.to_csv(OUTPUT_FILE, index=False)
    print(f'[MKR1000 SERVER] {OUTPUT_FILE} saved!')


def sendAmplitude():
    global last_A, last_f
    a = input('Desired peak Amplitude (mm) = ')
    # Send amplitude
    server.send((a + TERMINATOR_CHAR).encode())

    # When MKR1000 answers with 1, a will be received
    msg = int.from_bytes(server.recv(1))
    if not(msg):
        raise Exception('[AMPLITUDE CHANGE] MKR1000 did not send CHANGE_OK value')
    
    last_A = a
    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")
    df = pd.concat([df, pd.DataFrame([{'Timestamp': timestamp, 'Amplitude(mm)': a, 'Freq(Hz)': last_f}])])


def startServer():
    print('[MKR1000 SERVER] Connecting to server...')
    server.connect(ADDR)

    print('[MKR1000 SERVER] Connected!')
    
    # BG thread to handle client connection
    thread = threading.Thread(target=handle_client)
    thread.start()


def stopServer():
    global connected
    
    print('[MKR1000 SERVER] Closing connection...')
    connected = False
    server.close()

startServer()
