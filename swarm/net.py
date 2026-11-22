"""RadioLink: the only path between nodes. Every MAVLink packet goes to the radio emulator
(sim/radio.py), which drops or delays it and forwards it to the receivers' listen ports.
"""
import socket
import time

from swarm import config, protocol
from swarm import swarm_mav as mav

HOST = "127.0.0.1"


def listen_port(node_id, port_base=0) -> int:
    if node_id == config.GROUND_ID:
        return config.GROUND_PORT + port_base
    return config.agent_port(node_id) + port_base


class RadioLink:
    def __init__(self, node_id: int, port_base: int = 0):
        self.node_id = node_id
        self.radio = (HOST, config.RADIO_PORT + port_base)
        self.mav = mav.MAVLink(self, srcSystem=node_id, srcComponent=protocol.COMPONENT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((HOST, listen_port(node_id, port_base)))
        self.sock.setblocking(False)

    def write(self, data):
        """Called by self.mav.send: one packet, one datagram."""
        try:
            self.sock.sendto(data, self.radio)
        except OSError:
            pass   # radio not up: the message is lost, as on a real radio

    def send(self, msg) -> None:
        self.mav.send(msg)

    def recv_all(self) -> list:
        """Every message waiting, decoded, with .t_recv (wall time) set. Never blocks."""
        messages = []
        while True:
            try:
                data = self.sock.recv(65535)
            except BlockingIOError:
                return messages
            try:
                msg = self.mav.decode(bytearray(data))
            except mav.MAVError:
                continue   # not a packet of our dialect
            msg.t_recv = time.time()
            messages.append(msg)

    def close(self):
        self.sock.close()
