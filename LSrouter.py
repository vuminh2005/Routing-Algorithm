####################################################
# LSrouter.py
# Name:
# HUID:
#####################################################

from router import Router
from packet import Packet
import json
import heapq

class LSrouter(Router):
    """Link State routing protocol implementation.

    Each router maintains a complete map (link state database) of the network topology.
    When a link changes, it broadcasts updated link state information to all routers.
    Routers use Dijkstra's algorithm to compute shortest paths based on the current map.
    """

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # DO NOT REMOVE: base class initialization
        self.heartbeat_time = heartbeat_time
        self.last_time = 0  # Timestamp for the last broadcast

        # Link state database: {router: (sequence_number, {neighbor: cost})}
        self.link_state_db = {}

        # Local sequence number for own LSA (Link State Advertisement)
        self.sequence_number = 0

        # Initialize this router's own entry in the database
        self.link_state_db[self.addr] = (self.sequence_number, {})

        # Current forwarding table: {destination: output_port}
        self.forwarding_table = {}

        # Mapping of local ports to (neighbor address, cost)
        self.port_to_endpoint = {}

        # Ports that have been used to broadcast recent LSAs (for debug/log)
        self.broadcast_ports = set()

    def handle_packet(self, port, packet):
        """Process incoming packet (either data or link state update)."""
        if packet.is_traceroute:
            # Forward data packet based on forwarding table
            if packet.dst_addr in self.forwarding_table:
                out_port = self.forwarding_table[packet.dst_addr]
                self.send(out_port, packet)
        else:
            try:
                # Extract link state information from the packet
                link_state_info = json.loads(packet.content)
                router_addr = link_state_info["router"]
                sequence_number = link_state_info["seq"]
                link_state = link_state_info["links"]

                # Check if we received a newer update from this router
                is_new_or_updated = False
                if router_addr not in self.link_state_db:
                    is_new_or_updated = True
                else:
                    current_seq, _ = self.link_state_db[router_addr]
                    if sequence_number > current_seq:
                        is_new_or_updated = True

                if is_new_or_updated:
                    # Save new information and recompute paths
                    self.link_state_db[router_addr] = (sequence_number, link_state)
                    self.compute_shortest_paths()

                    # Flood the updated link state to other neighbors (except incoming port)
                    self.broadcast_ports = {port}
                    for p in self.port_to_endpoint:
                        if p != port:
                            self.send(p, packet)
                            self.broadcast_ports.add(p)
            except Exception as e:
                # Ignore malformed packets or parsing errors
                pass

    def compute_shortest_paths(self):
        """Use Dijkstra's algorithm to compute shortest paths and update forwarding table."""
        # Build graph from link state database
        graph = {}
        for router, (_, link_state) in self.link_state_db.items():
            graph[router] = link_state

        distances = {self.addr: 0}
        previous = {}
        priority_queue = [(0, self.addr)]

        # Dijkstra's algorithm
        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_node in distances and current_distance > distances[current_node]:
                continue  # Skip outdated entry

            if current_node not in graph:
                continue  # Skip unknown nodes

            for neighbor, cost in graph[current_node].items():
                distance = current_distance + cost
                if neighbor not in distances or distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))

        # Build forwarding table from shortest paths
        self.forwarding_table = {}
        for dest in distances:
            if dest != self.addr:
                current = dest
                # Backtrack path until reaching direct neighbor
                while previous.get(current) != self.addr:
                    if previous.get(current) is None:
                        break  # No path found
                    current = previous[current]

                # Map first hop to correct output port
                if previous.get(current) == self.addr:
                    for port, (endpoint, _) in self.port_to_endpoint.items():
                        if endpoint == current:
                            self.forwarding_table[dest] = port
                            break

    def create_link_state_packet(self):
        """Create a link state advertisement packet for this router."""
        self.sequence_number += 1

        # Build link state info: current neighbors and their costs
        link_state = {}
        for _, (endpoint, cost) in self.port_to_endpoint.items():
            link_state[endpoint] = cost

        # Update local copy of link state database
        self.link_state_db[self.addr] = (self.sequence_number, link_state)

        # Construct JSON-encoded packet content
        content = json.dumps({
            "router": self.addr,
            "seq": self.sequence_number,
            "links": link_state
        })

        packet = Packet(Packet.ROUTING, self.addr, "broadcast", content)
        return packet

    def broadcast_link_state(self):
        """Broadcast this router's current link state to all neighbors."""
        packet = self.create_link_state_packet()
        self.broadcast_ports = set()
        for port in self.port_to_endpoint:
            self.send(port, packet)
            self.broadcast_ports.add(port)

    def handle_new_link(self, port, endpoint, cost):
        """Triggered when a new link is established."""
        self.port_to_endpoint[port] = (endpoint, cost)

        # Broadcast the new topology and update paths
        self.broadcast_link_state()
        self.compute_shortest_paths()

    def handle_remove_link(self, port):
        """Triggered when an existing link is removed."""
        if port in self.port_to_endpoint:
            del self.port_to_endpoint[port]

            # Broadcast new topology without the removed link
            self.broadcast_link_state()
            self.compute_shortest_paths()

    def handle_time(self, time_ms):
        """Periodically broadcasts link state to maintain connectivity."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.broadcast_link_state()

    def __repr__(self):
        """Return a readable debug view of the router's state."""
        result = f"LSrouter(addr={self.addr})\n"
        result += f"Sequence Number: {self.sequence_number}\n"
        result += "Link State:\n"
        _, link_state = self.link_state_db.get(self.addr, (0, {}))
        for neighbor, cost in link_state.items():
            result += f"  {neighbor}: {cost}\n"
        result += "Forwarding Table:\n"
        for dest, port in self.forwarding_table.items():
            result += f"  {dest} -> Port {port}\n"
        return result
