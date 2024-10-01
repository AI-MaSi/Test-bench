from http.server import HTTPServer, BaseHTTPRequestHandler
import json

NUM_SEND = 8
NUM_RECV = 8

# ADDR = '192.168.2.204'
ADDR = '192.168.2.133'
PORT = 8000

class UDPServer:
    def __init__(self):
        self.client_data = {
            "client1": {"received_values": None, "local_values": [0.0] * NUM_SEND},
            "client2": {"received_values": None, "local_values": [0.0] * NUM_SEND}
        }

    def update_latest(self, client_id, values):
        if client_id not in self.client_data:
            raise ValueError(f"Invalid client ID: {client_id}")
        if len(values) != NUM_SEND:
            raise ValueError(f"Expected {NUM_SEND} values, got {len(values)}")
        self.client_data[client_id]["local_values"] = values.copy()

        # Transfer data to the other client
        other_client_id = "client1" if client_id == "client2" else "client2"
        self.client_data[other_client_id]["received_values"] = values.copy()

    def get_latest(self, client_id):
        if client_id not in self.client_data:
            raise ValueError(f"Invalid client ID: {client_id}")
        return self.client_data[client_id]["received_values"]

    def receive_data(self, client_id, values):
        if client_id not in self.client_data:
            raise ValueError(f"Invalid client ID: {client_id}")
        if len(values) != NUM_RECV:
            raise ValueError(f"Expected {NUM_RECV} values, got {len(values)}")
        self.client_data[client_id]["received_values"] = values.copy()


class RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith('/receive'):
            client_id = self.path.split('/')[-1]
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = json.dumps({'values': udp_server.get_latest(client_id)})
            self.wfile.write(response.encode())
        else:
            self.send_error(404)

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        data = json.loads(post_data.decode())

        if self.path.startswith('/send'):
            client_id = self.path.split('/')[-1]
            udp_server.update_latest(client_id, data['values'])
        elif self.path.startswith('/receive_data'):
            client_id = self.path.split('/')[-1]
            udp_server.receive_data(client_id, data['values'])
        else:
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps({'status': 'ok'}).encode())


def run_server(addr=ADDR, port=PORT, local=True):
    # Configure and start the HTTP server
    if local:
        server_address = ('localhost', port)
    else:
        server_address = (addr, port)

    httpd = HTTPServer(server_address, RequestHandler)
    print(f"Server running on {server_address}")
    httpd.serve_forever()


if __name__ == "__main__":
    udp_server = UDPServer()

    # Run the HTTP server
    run_server(local=False)
