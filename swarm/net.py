"""RadioLink: the only path between nodes. Every datagram goes to the radio emulator (sim/radio.py),
which drops or delays it and forwards it.

A node binds its own UDP port and tells the radio about it with HELLO, once a second, so a
restarted radio relearns every node within a second. HELLO uses seq 0; data messages count
from 1 (the radio's loss decision depends on seq).
"""
import socket
import time

from swarm import config, protocol

HOST = "127.0.0.1"
HELLO_PERIOD_S = 1.0


def listen_port(node_id, port_base=0) -> int:
    if node_id == config.GROUND_ID:
        return config.GROUND_PORT + port_base
    return config.agent_port(node_id) + port_base


class RadioLink:
    def __init__(self, node_id: int, port_base: int = 0):
        self.node_id = node_id
        self.port = listen_port(node_id, port_base)
        self.radio = (HOST, config.RADIO_PORT + port_base)
        self.seq = 0
        self.next_hello = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((HOST, self.port))
        self.sock.setblocking(False)

    def _to_radio(self, data):
        try:
            self.sock.sendto(data, self.radio)
        except OSError:
            pass   # radio not up: the message is lost, as on a real radio

    def _hello_if_due(self):
        now = time.time()
        if now >= self.next_hello:
            self._to_radio(protocol.encode(self.node_id, "radio", 0, now, "HELLO", port=self.port))
            self.next_hello = now + HELLO_PERIOD_S

    def send(self, dst, mtype: str, **body) -> int:
        """Send one message to node `dst` or "all". Returns its seq."""
        self._hello_if_due()
        self.seq += 1
        self._to_radio(protocol.encode(self.node_id, dst, self.seq, time.time(), mtype, **body))
        return self.seq

    def recv_all(self) -> list:
        """Every message waiting, decoded, with "t_recv" (wall time) added. Never blocks."""
        self._hello_if_due()
        messages = []
        while True:
            try:
                data = self.sock.recv(65535)
            except BlockingIOError:
                return messages
            msg = protocol.decode(data)
            msg["t_recv"] = time.time()
            messages.append(msg)

    def close(self):
        self.sock.close()
