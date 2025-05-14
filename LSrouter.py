####################################################
# LSrouter.py
# Name: Hoang Thanh Dat (completed)
# HUID:
####################################################

import json
import heapq
from packet import Packet
from router import Router


class LSrouter(Router):
    """Link‑State routing protocol implementation.

    Each router maintains a link‑state database (LSDB) of the entire topology.
    Whenever its own links change – or on a periodic heartbeat – it floods a
    Link‑State Packet (LSP) to all neighbours.  When an LSP that carries a more
    recent sequence number is received, the router updates its LSDB, runs
    Dijkstra to recompute shortest paths, then forwards (floods) the same LSP
    to every neighbour except the one it arrived on.

    The forwarding table maps *destination address ➜ outgoing port* and is
    consulted whenever a traceroute (data) packet is handled.
    """

    # ----------------------------------------------------------------------------------
    # Construction / helpers
    # ----------------------------------------------------------------------------------

    def __init__(self, addr, heartbeat_time):
        super().__init__(addr)  # DO NOT REMOVE

        # Timing -----------------------------------------------------------------------
        self.heartbeat_time = heartbeat_time      # ms between heart‑beats
        self.last_time = 0                        # last heartbeat time (ms)

        # Neighbour state --------------------------------------------------------------
        #   neighbours:  port ➜ (nbr_addr, cost)
        #   port_of:     nbr_addr ➜ port   (reverse lookup for first‑hop)
        self.neighbours = {}
        self.port_of = {}

        # Sequencing for *own* LSP -----------------------------------------------------
        self.seq_num = 0  # monotone increasing whenever own link‑state changes

        # Link‑State Database -----------------------------------------------------------
        #   lsdb: router_addr ➜ (seq#, {nbr: cost, ...})
        self.lsdb = {}

        # Forwarding table -------------------------------------------------------------
        #   forwarding_table: dst_addr ➜ port
        self.forwarding_table = {}

        # Store an initial (empty) link‑state entry for ourselves.
        self._store_own_ls()

    # ----------------------------------------------------------------------------------
    # Internal utility methods
    # ----------------------------------------------------------------------------------

    # ---------- LS helpers ----------
    def _current_links_dict(self):
        """Return {neighbour_addr: cost, ...} for own current links."""
        return {nbr: cost for nbr, cost in (self.neighbours[p] for p in self.neighbours)}

    def _make_lsp_packet(self):
        """Create a Packet.ROUTING that carries this router's current LSP."""
        body = {
            "src": self.addr,
            "seq": self.seq_num,
            "links": self._current_links_dict(),
        }
        return Packet(Packet.ROUTING, self.addr, "*", json.dumps(body))

    def _store_own_ls(self):
        """Update our own entry inside LSDB using current seq_num & links."""
        self.lsdb[self.addr] = (self.seq_num, self._current_links_dict())

    # ---------- flooding helper ----------
    def _flood(self, packet, except_port=None):
        """Send *packet* out every link except *except_port* (if provided)."""
        for p in self.links:  # self.links is maintained by base class
            if p != except_port:
                self.send(p, packet)

    # ---------- Dijkstra / forwarding ----------
    def _recompute_forwarding(self):
        """Run Dijkstra on LSDB ➜ refresh self.forwarding_table."""
        # Build adjacency list from LSDB
        graph = {}
        for node, (_, links) in self.lsdb.items():
            graph.setdefault(node, {})
            for nbr, cost in links.items():
                graph[node][nbr] = cost
        # Ensure own neighbours are present even if their LSP hasn't arrived yet
        graph.setdefault(self.addr, {})
        for port in self.neighbours:
            nbr, cost = self.neighbours[port]
            graph[self.addr][nbr] = cost

        # Dijkstra
        dist = {self.addr: 0}
        first_hop = {}
        pq = [(0, self.addr, None)]  # (distance, node, first_hop)
        while pq:
            d, u, via = heapq.heappop(pq)
            if d != dist.get(u, float("inf")):
                continue
            for v, w in graph.get(u, {}).items():
                nd = d + w
                if nd < dist.get(v, float("inf")):
                    dist[v] = nd
                    first_hop[v] = via if via else v
                    heapq.heappush(pq, (nd, v, first_hop[v]))

        # Build forwarding table
        self.forwarding_table.clear()
        for dst, hop in first_hop.items():
            if dst == self.addr:
                continue
            out_port = self.port_of.get(hop)
            if out_port is not None:
                self.forwarding_table[dst] = out_port

    # ----------------------------------------------------------------------------------
    # Required callbacks called by the simulator
    # ----------------------------------------------------------------------------------

    # New link comes up -----------------------------------------------------
    def handle_new_link(self, port, endpoint, cost):
        # Record neighbour mapping
        self.neighbours[port] = (endpoint, cost)
        self.port_of[endpoint] = port

        # Update own sequence & LSDB, recompute & flood
        self.seq_num += 1
        self._store_own_ls()
        self._recompute_forwarding()
        self._flood(self._make_lsp_packet())

    # Existing link goes down ----------------------------------------------
    def handle_remove_link(self, port):
        if port not in self.neighbours:
            return
        endpoint, _ = self.neighbours.pop(port)
        self.port_of.pop(endpoint, None)

        self.seq_num += 1
        self._store_own_ls()
        self._recompute_forwarding()
        self._flood(self._make_lsp_packet())

    # Incoming packet -------------------------------------------------------
    def handle_packet(self, port, packet):
        # 1️⃣ Data / traceroute packet – just forward using table
        if packet.is_traceroute:
            out_port = self.forwarding_table.get(packet.dst_addr)
            if out_port is not None:
                self.send(out_port, packet)
            # If unknown destination, silently drop
            return

        # 2️⃣ Routing packet (LSP)
        # Parse JSON body safely
        try:
            body = json.loads(packet.content)
            src = body["src"]
            seq = body["seq"]
            links = body["links"]
        except Exception:
            return  # malformed content → ignore

        # Ignore if old or duplicate
        old_seq, _ = self.lsdb.get(src, (-1, None))
        if seq <= old_seq:
            return

        # Store, recompute shortest paths, flood onward
        self.lsdb[src] = (seq, links)
        self._recompute_forwarding()
        self._flood(packet, except_port=port)

    # Time handler / heartbeat ---------------------------------------------
    def handle_time(self, time_ms):
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            # Heart‑beat flood (reuse current LSP without incrementing seq)
            self._flood(self._make_lsp_packet())

    # ----------------------------------------------------------------------------------
    # Debug helper shown in visualizer when router is clicked
    # ----------------------------------------------------------------------------------
    def __repr__(self):
        fwd = ", ".join(f"{dst}:{port}" for dst, port in self.forwarding_table.items())
        return f"LSrouter({self.addr}) seq={self.seq_num} fwd{{{fwd}}}"
