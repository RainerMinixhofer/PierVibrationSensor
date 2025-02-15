"""
websocked server
"""
import os
import socket
import time
# pylint: disable=import-error
#import network
# pylint: enable=import-error
import websocket_helper
from ws_connection import WebSocketConnection


class WebSocketClient:
    """ Websocket Client"""
    def __init__(self, conn):
        self.connection = conn

    def process(self):
        """ process """
        pass # pylint: disable=unnecessary-pass


class WebSocketServer:
    """ Websocket server """
    def __init__(self, page, iface, max_connections=1):
        self._listen_s = None
        self._clients = []
        self._iface = iface
        self._max_connections = max_connections
        self._page = page

    def _setup_conn(self, port, accept_handler):
        self._listen_s = socket.socket()
        self._listen_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        ai = socket.getaddrinfo("0.0.0.0", port)
        addr = ai[0][4]

        self._listen_s.bind(addr)
        self._listen_s.listen(1)
        if accept_handler:
            self._listen_s.setsockopt(socket.SOL_SOCKET, 20, accept_handler)

        if self._iface.active():
            ip = self._iface.ifconfig()[0]
            print(f"WebSocket started on ws://{ip:s}:{port:d}")

    def _accept_conn(self, listen_sock):
        cl, remote_addr = listen_sock.accept()
        print("Client connection from:", remote_addr)

        if len(self._clients) >= self._max_connections:
            # Maximum connections limit reached
            cl.setblocking(True)
            cl.sendall("HTTP/1.1 503 Too many connections\n\n")
            cl.sendall("\n")
            time.sleep(0.1)
            cl.close()
            return

        try:
            websocket_helper.server_handshake(cl)
        except OSError:
            # Not a websocket connection, serve webpage
            self._serve_page(cl)
            return

        self._clients.append(self._make_client(WebSocketConnection(remote_addr, cl, self.remove_connection)))

    def _make_client(self, conn):
        return WebSocketClient(conn)

    def _serve_page(self, sock):
        try:
            sock.sendall('HTTP/1.1 200 OK\nConnection: close\nServer: WebSocket Server\nContent-Type: text/html\n')
            length = os.stat(self._page)[6]
            sock.sendall(f"Content-Length: {length:d}\n\n")
            # Process page by lines to avoid large strings
            with open(self._page, 'r', encoding="utf-8") as f:
                for line in f:
                    sock.sendall(line)
        except OSError:
            # Error while serving webpage
            pass
        sock.close()

    def stop(self):
        """ stop """
        if self._listen_s:
            self._listen_s.close()
        self._listen_s = None
        for client in self._clients:
            client.connection.close()
        print("Stopped WebSocket server.")

    def start(self, port=80):
        """ start """
        if self._listen_s:
            self.stop()
        self._setup_conn(port, self._accept_conn)
        print("Started WebSocket server.")

    def process_all(self):
        """ process all """
        for client in self._clients:
            client.process()

    def send_something(self):
        """ send something """
        for client in self._clients:
            client.connection.write(str(time.ticks_ms())) # pylint: disable = E1101

    def remove_connection(self, conn):
        """ remove connection """
        for client in self._clients:
            if client.connection is conn:
                self._clients.remove(client)
                return
