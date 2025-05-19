####################################################
# DVrouter.py
# Name:
# HUID:
#####################################################

from router import Router
import json
from packet import Packet

class DVrouter(Router):
    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)
        self.heartbeat_time = heartbeat_time
        self.last_time = 0

        self.table = {}             # destination → (cost, port)
        self.vectors = {}           # neighbor → distance vector {dst: cost}
        self.neighbors = {}         # port → (neighbor_addr, cost)
        self.ports = {}             # neighbor_addr → port
        self.costs = {}             # neighbor_addr → cost

        self.table[self.addr] = (0, None)  # Self cost = 0

    def handle_packet(self, port, packet):
        if packet.is_traceroute:
            dst = packet.dst_addr
            if dst in self.table:
                out_port = self.table[dst][1]
                if out_port is not None:
                    self.send(out_port, packet)
        else:
            try:
                vec = json.loads(packet.content)
            except:
                return

            src = packet.src_addr
            self.vectors[src] = vec

            cost_to_src = self.costs.get(src)
            if cost_to_src is None:
                return

            updated = False
            for dest in vec:
                if dest == self.addr:
                    continue
                new_cost = cost_to_src + vec[dest]
                current_cost, _ = self.table.get(dest, (float("inf"), None))

                # Bellman-Ford relaxation step
                if new_cost < current_cost:
                    self.table[dest] = (new_cost, self.ports[src])
                    updated = True
                elif self.table.get(dest, (None, None))[1] == self.ports[src] and new_cost != current_cost:
                    # If route was through src and changed
                    self.table[dest] = (new_cost, self.ports[src])
                    updated = True

            if updated:
                self.broadcast_vector()

    def handle_new_link(self, port, endpoint, cost):
        self.neighbors[port] = (endpoint, cost)
        self.ports[endpoint] = port
        self.costs[endpoint] = cost
        self.table[endpoint] = (cost, port)
        self.broadcast_vector()

    def handle_remove_link(self, port):
        if port in self.neighbors:
            neighbor, _ = self.neighbors.pop(port)
            self.ports.pop(neighbor, None)
            self.costs.pop(neighbor, None)
            self.vectors.pop(neighbor, None)

            removed = [dst for dst, (_, p) in self.table.items() if p == port]
            for dst in removed:
                self.table.pop(dst)

            # Recompute possible alternatives via other neighbors
            updated = False
            for src, vec in self.vectors.items():
                if src not in self.costs or src not in self.ports:
                    continue
                for dest in vec:
                    if dest == self.addr:
                        continue
                    new_cost = self.costs[src] + vec[dest]
                    current_cost, _ = self.table.get(dest, (float("inf"), None))
                    if new_cost < current_cost:
                        self.table[dest] = (new_cost, self.ports[src])
                        updated = True
            if updated:
                self.broadcast_vector()

    def handle_time(self, time_ms):
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.broadcast_vector()

    def broadcast_vector(self):
        vec = {dst: cost for dst, (cost, _) in self.table.items()}
        content = json.dumps(vec)
        for port in self.neighbors:
            packet = Packet(Packet.ROUTING, self.addr, "*", content)
            self.send(port, packet)

    def __repr__(self):
        routes = ", ".join(f"{dst}:{p}" for dst, (_, p) in self.table.items())
        return f"DVrouter(addr={self.addr}, table={{ {routes} }})"
