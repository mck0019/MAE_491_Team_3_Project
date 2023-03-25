# web utility

# imports
import network
import usocket as socket
import uselect as select

# helper functions

def get_html(html_name):
    with open(html_name, 'r') as file:
        html = file.read()
    return html

class web_server:
    
    def __init__(self, ssid, password):
        
        # start an access point
        self.ap = network.WLAN(network.AP_IF)
        self.ap.config(essid=ssid, password=password)
        self.ap.active(True)

        while self.ap.active() == False:
            pass
        print("[log] Wireless Access Point '" + ssid + "' Started!")
        
    def create_socket(self):
        
         #Create a new non-blocking socket
        addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
        self.s = socket.socket()
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.setblocking(False)

        # Bind the socket to a specific host and port
        self.s.bind(addr)

        # Listen for incoming connections
        self.s.listen(5)
        #print('listening on', addr)
        
        self.sockets = [self.s]
        
    def receive(self):
        readable, writable, errored = select.select(self.sockets, [], [], 0)
        
        for s in readable:
            if s is self.s:
                # Accept the incoming connection and create a new socket object
                client_socket, address = s.accept()
                #print("Accepted connection from", address)

                # Add the new socket to the list of sockets to monitor
                self.sockets.append(client_socket)

            else:
                # Receive data from the client
                data = str(s.recv(1024))
                if data:
                    return s, data
                else:
                    return s, None
        return None,None
    
    def stop(self):
        print("[log] Wireless Access Point Stopped!")
        for s in self.sockets:
            if (s):
                s.close()
        
class parser:
    
    def __init__(self, string):
        self.string = string
        self.index = 0
        
    def get_element(self, element, default):
        start_index = self.string.find(element + "=", self.index)
        length = len(element + "=")
        if (start_index == -1):
            return default
        for i, char in enumerate(self.string[start_index+length:]):
            if char in [" ", "&", "\n", "\r"]:
                break;
        last_index = start_index+length+i
        self.index = last_index
        
        return self.string[start_index+length:last_index]