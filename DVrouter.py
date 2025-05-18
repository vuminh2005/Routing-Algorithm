####################################################
# DVrouter.py
# Name:
# HUID:
#####################################################

import json
from router import Router
from packet import Packet


class DVrouter(Router):
    """Distance Vector routing protocol implementation.

    This router maintains a table of the lowest known cost to each destination
    and updates it based on distance vectors received from its neighbors.
    """

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Required base class initialization
        self.heartbeat_time = heartbeat_time  # Interval for periodic broadcasts
        self.last_time = 0  # Last time a broadcast occurred

        # Main routing data structures
        self.distance_vector = {self.addr: (0, self.addr)}  # {destination: (cost, next_hop)}
        self.forwarding_table = {}  # {destination: port}
        self.neighbors = {}  # {port: (neighbor address, cost)}
        self.dv_from_neighbors = {}  # {neighbor address: its last advertised DV}

    def handle_packet(self, port, packet):
        """Handles both data and routing packets arriving at this router."""
        if packet.is_traceroute:
            # Handle traceroute/data packets: forward to next hop using forwarding table
            if packet.dst_addr in self.forwarding_table:
                port = self.forwarding_table[packet.dst_addr]
                self.send(port, packet)
        else:
            # Handle routing packets (distance vector updates from neighbors)
            try:
                neighbor_dv = json.loads(packet.content)
                neighbor = packet.src_addr  # Who sent the DV
                self.dv_from_neighbors[neighbor] = neighbor_dv  # Save the DV

                # Recalculate local DV and broadcast if it changed
                if self.recalculate():
                    self.update_forwarding_table()
                    self.broadcast_dv()
            except:
                pass  # Ignore malformed packets

    def handle_new_link(self, port, endpoint, cost):
        """Called when a new link is created between this router and a neighbor."""
        self.neighbors[port] = (endpoint, cost)
        self.distance_vector[endpoint] = (cost, endpoint)  # Direct link entry

        # Recalculate full DV and update table if anything changed
        if self.recalculate():
            self.update_forwarding_table()
            self.broadcast_dv()

    def handle_remove_link(self, port):
        """Called when a link is removed (e.g., cable cut or neighbor fails)."""
        if port not in self.neighbors:
            return

        endpoint = self.neighbors[port][0]
        del self.neighbors[port]

        # Remove the stored DV from this neighbor
        if endpoint in self.dv_from_neighbors:
            del self.dv_from_neighbors[endpoint]

        # Recalculate DV to remove any route that relied on the removed link
        if self.recalculate():
            self.update_forwarding_table()
            self.broadcast_dv()

    def handle_time(self, time_ms):
        """Called periodically; sends out current DV if heartbeat interval passed."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.broadcast_dv()

    def recalculate(self):
        """Recalculates the current router's distance vector based on:
        - Direct neighbor costs
        - Neighbor distance vectors
        Returns True if the DV changed (triggering an update and broadcast).
        """
        changed = False
        new_dv = {self.addr: (0, self.addr)}  # Always knows itself with cost 0

        for port, (neighbor, cost_to_neighbor) in self.neighbors.items():
            # Directly reachable neighbor
            new_dv[neighbor] = (cost_to_neighbor, neighbor)

            if neighbor in self.dv_from_neighbors:
                neighbor_dv = self.dv_from_neighbors[neighbor]

                for dest in neighbor_dv:
                    if dest == self.addr:
                        continue  # Skip self-loops

                    cost_to_dest_from_neighbor, _ = neighbor_dv[dest]
                    total_cost = cost_to_neighbor + cost_to_dest_from_neighbor

                    # Keep the lowest cost path seen
                    if dest not in new_dv or total_cost < new_dv[dest][0]:
                        new_dv[dest] = (total_cost, neighbor)

        # Only update if something changed
        if new_dv != self.distance_vector:
            self.distance_vector = new_dv
            changed = True
        return changed

    def update_forwarding_table(self):
        """Updates the port-based forwarding table based on next hops in DV."""
        self.forwarding_table = {}
        for dest in self.distance_vector:
            if dest == self.addr:
                continue  # Skip self

            next_hop = self.distance_vector[dest][1]
            for port, (neighbor, _) in self.neighbors.items():
                if neighbor == next_hop:
                    self.forwarding_table[dest] = port
                    break  # Found the correct port

    def broadcast_dv(self):
        """Broadcasts this router's current distance vector to all neighbors."""
        content = json.dumps(self.distance_vector)
        for port, (neighbor, _) in self.neighbors.items():
            packet = Packet(Packet.ROUTING, self.addr, neighbor, content)
            self.send(port, packet)

    def __repr__(self):
        """String representation for visualization/debugging."""
        return f"DVrouter(addr={self.addr}, dv={self.distance_vector}, table={self.forwarding_table}, neighbor={self.neighbors})"