# web_util.py
# April 4th, 2023

# imports
import network
import usocket
import uhashlib
import ubinascii

# defines
OPCODE_CONT = 0
OPCODE_TEXT = 1
OPCODE_BINARY = 2
OPCODE_CLOSE = 8

# general helpers

def serve_webpage():
    
    # create listening socket
    addr = ('0.0.0.0', 80)
    s = usocket.socket()
    s.setsockopt(usocket.SOL_SOCKET, usocket.SO_REUSEADDR, 1)
    s.setblocking(False)
    s.bind(addr)
    s.listen(1)
    
    # wait for connection
    conn, adrr = s.accept()
    
    # get headers
    receive = conn.recv(1024).decode()
    s.close()
    header_lines = receive.split("\r\n")
    headers = {}

    for line in header_lines:
        if ":" in line:
            key, value = line.split(":", 1)
            headers[key] = value.strip()
            
    # serve page
    response = open("index.html", "r").read()
    conn.send("HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n")
    conn.send(response)
    conn.close()

# network helpers

def start_network(ssid, password):
    ap = network.WLAN(network.AP_IF)
    ap.config(essid=ssid, password=password)
    ap.active(True)
    print(f"started network '{ssid}'.")
    return ap

def stop_network(network):
    network.active(False)
    print(f"stopped network '{network.config('ssid')}'.")

# web socket helper

class web_socket:
    
    def __init__(self, addr):
        # create socket
        self.s = usocket.socket()
        self.s.setsockopt(usocket.SOL_SOCKET, usocket.SO_REUSEADDR, 1)
        self.s.bind(addr)
        self.s.listen(1)
        print("created socket.")

    def upgrade(self):
        
        # wait for connection
        conn, addr = self.s.accept()
        print("recieved connection. sending handshake...")
        
        # get headers
        receive = conn.recv(1024).decode()
        self.s.close()
        header_lines = receive.split("\r\n")
        headers = {}

        for line in header_lines:
            if ":" in line:
                key, value = line.split(":", 1)
                headers[key] = value.strip()
        
        if (headers["Connection"] == "Upgrade"):
            
            # generate key
            key = headers["Sec-WebSocket-Key"]
            d = uhashlib.sha1(key.encode())
            d.update(b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11')
            response_key = ubinascii.b2a_base64(d.digest())[:-1]
            
            # send response
            conn.send(b'HTTP/1.1 101 Switching Protocols\r\n')
            conn.send(b'Upgrade: websocket\r\n')
            conn.send(b'Connection: Upgrade\r\n')
            conn.send(b'Sec-WebSocket-Accept: ' + response_key + b'\r\n\r\n')
        
            self.s = conn
        
        print("handshake complete.")
    
    # receeive data
    def receive(self):
        
        # get header
        header = self.s.recv(2)
        if len(header) != 2:
            return None
        
        # parse the header
        fin = header[0] & 0x80
        opcode = header[0] & 0x0f
        has_mask = header[1] & 0x80
        length = header[1] & 0x7f
        
        if opcode == OPCODE_CLOSE:
            return None
        
        if length == 126: length = -2
        elif length == 127: length = -8
        
        if length < 0:
            length = self.s.recv(-length)
            length = int.from_bytes(length, 'big')
        if has_mask:
            mask = self.s.recv(4)
        
        # receive the message
        payload = self.s.recv(length)
        
        if has_mask:
            payload = bytes(x ^ mask[i % 4] for i, x in enumerate(payload))

        if opcode == OPCODE_TEXT:
            payload.decode()
        
        return payload
    
    # send data
    def send(self, message):
        
        # write header
        payload = bytearray()
        payload.append(0x80 | 1)
        
        if len(message) < 126:
            payload.append(len(message))
        elif len(message) < (1 << 16):
            payload.append(126)
            payload.extend(len(message).to_bytes(2, 'big'))
        else:
            payload.append(127)
            payload.extend(len(message).to_bytes(8, 'big'))
            
        payload.extend(message)
        
        # send the message
        self.s.send(payload)
    
    # close the socket
    def close(self):
        self.s.close()
        print("web socket closed.")