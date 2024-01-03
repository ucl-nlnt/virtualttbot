import pygame, sys, threading, time, socket, struct, uuid, os

if not os.path.exists("datalogs"):
    os.mkdir("datalogs")

class turtlebot_controller:

    def __init__(self, manual_control = True):

        # will be useful once data gathering autmation is started
        self.manual_control = manual_control
        
        # keyboard stuff
        self.keyboard_input = None
        self.keyboard_impulse = False
        self.killswitch = False
        self.debug = True
        self.debug_window_process = True
        self.label = None

        # socket programming stuff
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', 50000))
        self.server_sock.listen(0)
        print(f'Server Listening on port 50000')

        self.client, client_address = self.server_sock.accept()
        print('Client copnnected from', client_address)

        # multithreading processes
        self.window_thread = threading.Thread(target = self.window_process)
        self.window_thread.start()
        self.listener_thread = threading.Thread(target = self.kb_listener)
        self.listener_thread.start()



    def window_process(self):

        pygame.init()
        self.screen = pygame.display.set_mode((300, 300))
        pygame.display.set_caption('Turtlebot Controller')
        self.pygame_font = pygame.font.SysFont('Arial', 20)
        self.text_color = (25, 25, 25)
        self.text_display_content = 'DEFAULT TEXT'
        self.is_collecting_data = False

        # Cap the frame rate
        pygame.time.Clock().tick(60)
        
        while not self.killswitch:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.killswitch = True
                    self.send_data('@KILL')
                    print('Killing program...')
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.KEYDOWN:
                    self.keyboard_input = event.unicode; self.keyboard_impulse = True
                    if self.debug_window_process: print(f"pressed {self.keyboard_input}")
                elif event.type == pygame.KEYUP:
                    self.keyboard_input = None; self.keyboard_impulse = True
                    if self.debug_window_process: print("Released key.")
            # Fill the screen with a background color
            self.screen.fill((255, 255, 255))

            # Render and display the text input
            text_surface = self.pygame_font.render(self.text_display_content, True, self.text_color)
            self.screen.blit(text_surface, (10, 10))

            # Update the display
            pygame.display.flip()

            
    def kb_listener(self): # do Turtlebot controller stuff here


        """
        Instructions: // all data instructions are 5 bytes wide
        @XXXX, where the 4 X's are the instructions to be sent to the robot.
        @0000 - Stop
        @FRWD - Forward
        @LEFT - Turn Left
        @RGHT - Turn Right
        @ODOM - ask for odometry data
        @STRT - start recording data
        @STOP - stop recording data
        @KILL - stop program
        @RNDM - randomize position, -5 <= x,y <= 5, 0 <= theta <= 2pi

        Each data send starts with sending an 8-byte long size indicator. Each data segment is sent in 1024-byte-sized chunks.
        Once all is received, receiver sends an 'OK' to sender to indicate that is done processing whatever data was
        sent over.
        """

        # assumption: socket connection is successful
        while not self.killswitch: # Outer loop

            print("starting new data collection loop...")
            self.label = input('Enter data label: ')
            self.text_display_content = "Now collecting Data." + '\nLabel:{}\n'.format(self.label) 
            self.is_collecting_data = True
            
            print("Sending START signal...")
            self.send_data('@STRT')

            print("START good... receiving origin data...")
            data_logs = [self.receive_data().decode()]
            print(f"Origin data received: {data_logs}")
            while self.is_collecting_data and not self.killswitch:

                # print(self.is_collecting_data, self.killswitch, self.keyboard_input)
                if self.keyboard_impulse:
                    print(self.keyboard_input)
                    self.keyboard_impulse = False

                    if self.keyboard_input == None:
                        self.send_data('@0000')

                    elif self.keyboard_input == 'w': # move forward
                        self.send_data('@FRWD')
                    
                    elif self.keyboard_input == 'a': # turn left
                        self.send_data('@LEFT')

                    elif self.keyboard_input == 'd': # turn right
                        self.send_data('@RGHT')

                    elif self.keyboard_input == 'o': # retrieve current odometry
                        self.send_data('@ODOM')
                        odometry_data = self.receive_data()
                        self.text_display_content += '\n' + str(odometry_data)
                        data_logs.append(odometry_data.decode())

                    elif self.keyboard_input == '/': # stop recording and save data points
                        self.send_data('@STOP')
                        self.is_collecting_data = False
                        print("Data collection finished. Restarting loop.")

                    elif self.keyboard_input == '=':
                    
                        self.send_data('@RNDM')
                        # user-side waits for Turtlebot to randomize position.

                        # wait for READY signal
                        go_signal = self.receive_data() # wait for @RNDM
                       
                else: pass

            if not self.killswitch:

                filename = os.path.join(os.getcwd(),'datalogs',self.generate_random_filename())
                
                with open(filename, 'w') as f:
                    f.write(str({'label':self.text_display_content, 'data_points':data_logs}))
                    print(f'Data written to {filename}.')

    def generate_random_filename(self):
        random_filename = str(uuid.uuid4().hex)[:16]
        return random_filename
    
    def send_data(self, data: bytes):

        if isinstance(data, str):
            data = data.encode()

        # NOTE: data may or may not be in string format.
            
        length_bytes = struct.pack('!I', len(data))
        
        if self.debug: print('[S] Sending byte length...')
        self.client.sendall(length_bytes)
        ack = self.client.recv(2) # wait for other side to process data size
        if ack != b'OK': print(f'[S] ERROR: unmatched send ACK. Received: {ack}')
        if self.debug: print('[S] ACK good')

        if self.debug: print('[S] Sending data...')
        self.client.sendall(data) # send data
        if self.debug: print('[S] Data sent; waiting for ACK...')
        ack = self.client.recv(2) # wait for other side to process data size
        if ack != b'OK': print(f'[S] ERROR: unmatched send ACK. Received: {ack}')
        if self.debug: print('[S] ACK good. Data send success.')

    def receive_data(self):

        # NOTE: Returns data in BINARY. You must decode it on your own

        if self.debug: print('[R] Waiting for byte length...')
        length_bytes = self.client.recv(4)
        length = struct.unpack('!I', length_bytes)[0]
        if self.debug: print(f'[R] Byte length received. Expecting: {length}')
        data, data_size = b'', 0

        self.client.send(b'OK') # allow other side to send over the data
        if self.debug: print(f'[R] ACK sent.')
        while data_size < length:

            chunk_size = min(2048, length - data_size)
            data_size += chunk_size
            data += self.client.recv(chunk_size)
            if self.debug: print(f'[R] RECV {chunk_size}')

        if self.debug: print('[R] Transmission received successfull. Sending ACK')       
        self.client.send(b'OK') # unblock other end
        if self.debug: print('[R] ACK sent.')
        return data # up to user to interpret the data
    
x = turtlebot_controller()
while not x.killswitch:
    pass