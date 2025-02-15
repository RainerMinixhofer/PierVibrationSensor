"""
Websocket connection
"""
import socket
# pylint: disable=import-error
from websocket import websocket
# pylint: enable=import-error


class ClientClosedError(Exception):
    """
    Client Closed Error exception
    """
    pass #pylint: disable=unnecessary-pass


class WebSocketConnection:
    """
    Websocket Connection
    """
    def __init__(self, addr, s, close_callback):
        self.client_close = False
        self._need_check = False

        self.address = addr
        self.socket = s
        self.ws = websocket(s, True)
        self.close_callback = close_callback

        s.setblocking(False)
        s.setsockopt(socket.SOL_SOCKET, 20, self.notify)

    def notify(self, s):
        """
        notify
        """
        self._need_check = True

    def read(self):
        """ read """
        if self._need_check:
            self._check_socket_state()

        msg_bytes = None
        try:
            msg_bytes = self.ws.read() # type: ignore
        except OSError:
            self.client_close = True
            print("read: Client closed connection.")
            if not msg_bytes:
                raise ClientClosedError()

#        if not msg_bytes and self.client_close:
#            raise ClientClosedError()

        return msg_bytes

    def write(self, msg):
        """ write """
        try:
            self.ws.write(msg) # type: ignore
        except OSError:
            self.client_close = True
            print("write: Client closed connection.")

    def _check_socket_state(self):
        self._need_check = False
        sock_str = str(self.socket)
        state_str = sock_str.split(" ")[1]
        state = int(state_str.split("=")[1])

        if state == 3:
            self.client_close = True
            print("check_socket_state: Client closed connection.")

    def is_closed(self):
        """ is closed """
        return self.socket is None

    def close(self):
        """ close """
        print("Closing connection.")
        self.socket.setsockopt(socket.SOL_SOCKET, 20, None) # type: ignore
        self.socket.close() # type: ignore
        self.socket = None
        self.ws = None
        if self.close_callback:
            self.close_callback(self)
