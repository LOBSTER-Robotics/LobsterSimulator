import socket


class Logger:

    logger = None

    @staticmethod
    def get_logger():
        if Logger.logger is None:
            Logger.logger = Logger()

        return Logger.logger

    connections = []

    def __init__(self):
        self._data = dict()

    def store(self, target, value):
        self._data[target] = value

    def update(self):
        for conn in self.connections:
            conn.send(conn.send(bytes(self._data), 'utf-8'))

        print(self._data)

    def add_tcp_client(self):
        TCP_IP = '127.0.0.1'
        TCP_PORT = 5005
        BUFFER_SIZE = 1024  # Normally 1024, but we want fast response

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((TCP_IP, TCP_PORT))
        s.listen(1)

        conn, addr = s.accept()
        self.connections.append(conn)

    def info(self, message, sep=' ', end='\n', file=None):
        for conn in self.connections:
            conn.send(conn.send(bytes(str(message), 'utf-8')))

        print(message)
