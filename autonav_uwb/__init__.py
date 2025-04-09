import time
import serial
import threading
import enum
import typing
import logging

class UWBTag:

    class Event(enum.Enum):
        CONNECT = "CONNECT"
        DISCONNECT = "DISCONNECT"
        DISTANCE = "DISTANCE"
        SERIAL_DISCONNECT = "SERIAL_DISCONNECT"

    class _LogFormatter(logging.Formatter):

        cyan = "\x1b[36m"
        grey = "\x1b[38;20m"
        yellow = "\x1b[33;20m"
        red = "\x1b[31;20m"
        bold_red = "\x1b[31;1m"
        reset = "\x1b[0m"
        format = "%(levelname)s : %(name)s : %(message)s"

        FORMATS = {
            logging.DEBUG: cyan + format + reset,
            logging.INFO: grey + format + reset,
            logging.WARNING: yellow + format + reset,
            logging.ERROR: red + format + reset,
            logging.CRITICAL: bold_red + format + reset
        }

        def format(self, record):
            log_fmt = self.FORMATS.get(record.levelno)
            formatter = logging.Formatter(log_fmt)
            return formatter.format(record)

    def __init__(self,
                 port: str = None,
                 baudrate: int = 115200, 
                 bytesize: int = serial.EIGHTBITS,
                 parity: str = serial.PARITY_NONE,
                 stopbits: float = serial.STOPBITS_ONE,
                 debug : bool = False):

        # General Config
        self._debug = debug        

        # Serial Config
        self._serial = None
        self._port = port
        self._baudrate = baudrate
        self._bytesize = bytesize
        self._parity = parity
        self._stopbits = stopbits

        # Threading states
        self._thread = None
        self._stop_event = threading.Event()
        self._thread_lock = threading.Lock()

        # Callbacks
        self._callbacks = { e: [] for e in self.Event }

        # Logging Setup
        logging.basicConfig(level=logging.DEBUG if debug else logging.WARNING)
        
        self._logger = logging.getLogger(self.__class__.__name__)
        self._logger.propagate = 0
        
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG if debug else logging.WARNING)
        ch.setFormatter(self._LogFormatter())
        
        if not self._logger.handlers:
            self._logger.addHandler(ch)

    @property
    def is_running(self):
        return self._running
    
    @property
    def is_connected(self):
        return self._serial is not None and self._serial.is_open

    def on(self, event: Event, callback: typing.Callable):
        if event in self._callbacks:
            self._callbacks[event].append(callback)
        else:
            self._logger.warning(f"Event {event} not recognized for callbacks.")

    def trigger(self, event: Event, *args, **kwargs):
        if event in self._callbacks:
            for callback in self._callbacks[event]:
                try:
                    callback(*args, **kwargs)
                except Exception as e:
                    self._logger.error(f"Error in callback for event {event}: {e}")

    def _process(self):
        self._running = True
        self._logger.debug("processing thread started")

        # main event loop
        while not self._stop_event.is_set():
            if self._serial is not None:
                try:
                    if self._serial.is_open:
                        if self._serial.in_waiting > 0:
                            # TODO - Regex
                            data = self._serial.readline()
                            datastr = str(data)[2:][:-5]
                            self._logger.debug("received: %s" % datastr)
                            action = datastr[0:3]
                            
                            if action == "[+]":
                                addr = datastr[3:]

                                self._logger.debug(f"anchor found: {addr}")

                                self.trigger(self.Event.CONNECT, addr)

                            elif action == "[-]":
                                addr = datastr[3:]
                                self._logger.info(f"anchor lost: {addr}")
                                self.trigger(self.Event.DISCONNECT, addr)

                            elif action == "[>]":
                                try:
                                    splitIndex = datastr.index(";")
                                    addr = datastr[3:splitIndex]
                                    distance = float(datastr[splitIndex + 1:])

                                    self._logger.info(f"Distance Received: {distance} cm from {addr}")

                                    self.trigger(self.Event.DISTANCE, 
                                                address=addr,
                                                distance=distance)
                                except ValueError:
                                    self._logger.error("Invalid distance data received.")
                            else:
                                self._logger.debug("Unknown data format received")
                    else:
                        self._logger.error("Serial port became disconnected!")
                        self.trigger(self.Event.SERIAL_DISCONNECT)
                        self.stop()
                except serial.SerialException as e:
                    self._logger.error(f"Serial exeception: {e}")
                    self.trigger(self.Event.SERIAL_DISCONNECT)
                    self.stop()
            else:
                self._logger.error("Serial object is not created!")
                self.stop()

        self._logger.debug("processing thread killed")
        self._running = False



    def start(self):
        with self._thread_lock:
            if not self._port:
                self._logger.warning("No serial port specified.")
                return
            
            if self._running:
                self._logger.warning("Attempted to start when process is already running.")
                return
            
            if self._serial is not None:
                self._logger.warning("Serial port already connected.")
                return

            # Create serial object
            try:
                self._serial = serial.Serial(
                    port = self._port,
                    baudrate = self._baudrate,
                    bytesize = self._bytesize,
                    parity = self._parity,
                    stopbits = self._stopbits,
                )
            except serial.SerialException as e:
                self._logger.error(f"Failed to open serial port: {e}")
                return

            # Give serial time to connect
            time.sleep(0.5) 

            # Check for connection
            if self._serial.is_open:
                self._logger.debug("serial port connected")
            else:
                self._logger.warning("Failed to connect to serial port.")
                return
            
            if self._stop_event.is_set():
                self._stop_event.clear()

            self._thread = threading.Thread(target=self._process, daemon=True)
            self._logger.debug("created processing thread")
            self._thread.start()

    def stop(self):
        with self._thread_lock:
            # Set stop event
            if not self._stop_event.is_set():
                self._stop_event.set()

            # Delete thread
            if self._thread is not None:
                self._thread.join()
                self._logger.debug("stopped processing thread")
                self._thread = None
            
            if self._serial is not None:
                self._serial.close()
                self._logger.debug("serial port disconnected")
                self._serial = None

    def __del__(self):
        if self._running:
            self.stop()