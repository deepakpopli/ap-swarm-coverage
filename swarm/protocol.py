"""Wire format. Every agent<->agent and agent<->ground message is one UDP datagram holding
one JSON object, sent to the radio process, which drops/delays it and forwards it.

Envelope (added by the sender):
  {"src": int, "dst": int | "all", "seq": int, "t": float(wall time at send), "type": str, ...body}
src/dst: 0 = ground station, 1..N = agents. dst "all" = everyone except the sender.

Message types and bodies
  HELLO      any -> radio only (not forwarded): {"port": int}   registers the sender's UDP listen port
  STATE      agent -> all, STATE_HZ:
             {"pos":[x,y,z], "vel":[vx,vy,vz], "epoch": int, "slot": int, "status": str}
             status in: "ground","takeoff","ready","cover","hold","fallback"
  START      ground -> all, repeated: {"t0": float}   wall time of mission t=0
  ASSIGN     ground -> one agent, re-sent at ASSIGN_HZ (idempotent by epoch):
             {"epoch": int,                # global, increases on every re-plan
              "agent": int,
              "share": [xmin,xmax,ymin,ymax],
              "slots": [[x,y], ...],       # ordered goal slots; after the last, loop back to "loop_from"
              "loop_from": int,            # index in slots where the repeating coverage loop starts
              "alive": [ids]}              # the allocator's current live set
  INFEASIBLE agent -> ground, repeated until a newer epoch arrives:
             {"epoch": int, "slot": int, "reason": "timeout"|"safety", "pos":[x,y,z]}
The allocator never sends velocities. Agents never read another vehicle's MAVLink.
"""
import json

def encode(src, dst, seq, t, mtype, **body):
    d = {"src": src, "dst": dst, "seq": seq, "t": t, "type": mtype}
    d.update(body)
    return json.dumps(d, separators=(",", ":")).encode()

def decode(data):
    return json.loads(data.decode())
