"""RadioLink: the only path between nodes (ground station and agents).

Each node binds its own UDP listen port on 127.0.0.1 and sends every datagram to the radio
process (swarm/radio.py), which drops/delays it and forwards it to the receivers.

    link = RadioLink(node_id, port_base)
    link.send("all", "STATE", pos=[...], vel=[...], epoch=3, slot=0, status="cover")
    for msg in link.recv_all():      # non-blocking, decoded dicts
        ...

HELLO (registration of this node's listen port with the radio) is sent once at construction
and then re-sent every HELLO_PERIOD_S from inside send()/recv_all(), so a node that keeps
calling recv_all() in its loop stays registered even if the radio restarts.

Sequence numbers: `seq` counts only data messages sent by this node (HELLO uses seq 0 and
does not advance the counter). The radio's loss decision is a function of (seed, src, dst, seq).
"""
import socket
import time

from swarm import config, protocol

HELLO_PERIOD_S = 1.0
RADIO_HOST = "127.0.0.1"


def listen_port(node_id, port_base=0):
    """UDP listen port of a node: ground uses GROUND_PORT, agents use agent_port(id)."""
    if node_id == config.GROUND_ID:
        return config.GROUND_PORT + port_base
    return config.agent_port(node_id) + port_base


def radio_port(port_base=0):
    return config.RADIO_PORT + port_base


class RadioLink:
    def __init__(self, node_id, port_base=0):
        self.node_id = node_id
        self.port = listen_port(node_id, port_base)
        self.radio_addr = (RADIO_HOST, radio_port(port_base))
        self.seq = 0
        self._last_hello = float("-inf")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((RADIO_HOST, self.port))
        self.sock.setblocking(False)
        self._maybe_hello()

    def _sendto_radio(self, data):
        try:
            self.sock.sendto(data, self.radio_addr)
        except OSError:
            pass  # radio not up (yet): the message is simply lost, like on a real radio

    def _maybe_hello(self):
        now = time.time()
        if now - self._last_hello >= HELLO_PERIOD_S:
            self._last_hello = now
            self._sendto_radio(protocol.encode(self.node_id, "radio", 0, now, "HELLO", port=self.port))

    def send(self, dst, mtype, **body):
        """Send one message to node id `dst` or "all". Returns the seq used."""
        self._maybe_hello()
        self.seq += 1
        self._sendto_radio(protocol.encode(self.node_id, dst, self.seq, time.time(), mtype, **body))
        return self.seq

    def recv_all(self):
        """All datagrams waiting in the socket, decoded. Never blocks. Adds 't_recv' (wall time)."""
        self._maybe_hello()
        out = []
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break  # e.g. ICMP port unreachable reported on some platforms
            try:
                msg = protocol.decode(data)
            except (ValueError, UnicodeDecodeError):
                continue
            if isinstance(msg, dict):
                msg["t_recv"] = time.time()
                out.append(msg)
        return out

    def close(self):
        self.sock.close()
