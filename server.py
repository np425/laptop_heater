import asyncio
import websockets
import json
import argparse
import signal
from http.server import SimpleHTTPRequestHandler, HTTPServer
from threading import Thread
import os

# Assuming TemperatureController is defined in another module
from temperature_control import TemperatureController

connected_clients = set()
slider_value = 55  # Initial slider value
temperature_controller = TemperatureController(target_temperature=slider_value)  # Initialize with slider value

# Custom HTTP handler to serve only index.html
class SingleFileHTTPRequestHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        self.path = '/index.html'  # Serve only index.html for root requests
        return SimpleHTTPRequestHandler.do_GET(self)

# WebSocket server handler
async def slider_handler(websocket, path):
    global slider_value
    connected_clients.add(websocket)
    print("Client connected")

    try:
        # Send the current slider value to the newly connected client
        await websocket.send(json.dumps({"sliderValue": slider_value}))

        # Start a background task to send core temperatures periodically
        send_temperature_task = asyncio.create_task(send_core_temperatures(websocket))

        # Listen for messages from the client
        async for message in websocket:
            data = json.loads(message)
            if "sliderValue" in data:
                slider_value = float(data["sliderValue"])
                print(f"Received slider value: {slider_value}")

                # Update the temperature target for the PID controller
                temperature_controller.set_target_temperature(slider_value)

                # Broadcast the new slider value to all connected clients
                broadcast_tasks = [
                    asyncio.create_task(client.send(json.dumps({"sliderValue": slider_value})))
                    for client in connected_clients
                ]
                await asyncio.gather(*broadcast_tasks)

    except websockets.ConnectionClosed:
        print("Client disconnected")
    finally:
        # Clean up when client disconnects
        connected_clients.remove(websocket)
        send_temperature_task.cancel()

# Function to send core temperatures to all connected clients
async def send_core_temperatures(websocket):
    while True:
        if connected_clients:  # Send only if there are connected clients
            core_temperatures = temperature_controller.get_core_temperatures()
            data = json.dumps({"coreTemperatures": core_temperatures})
            await asyncio.gather(*[client.send(data) for client in connected_clients])
        await asyncio.sleep(1)  # Send updates every second

# Function to gracefully shutdown the WebSocket server and connected clients
async def shutdown(server, loop):
    print("Shutting down WebSocket server...")
    
    # Make a copy of connected_clients to avoid RuntimeError
    for client in connected_clients.copy():
        await client.close()
    connected_clients.clear()

    # Cancel all running tasks
    tasks = [t for t in asyncio.all_tasks(loop) if t is not asyncio.current_task()]
    list(map(lambda task: task.cancel(), tasks))
    
    await asyncio.gather(*tasks, return_exceptions=True)
    
    # Stop the loop
    loop.stop()
    print("WebSocket server shut down complete.")

# Function to start the WebSocket server
async def start_websocket_server(port):
    async with websockets.serve(slider_handler, "localhost", port) as server:
        print(f"WebSocket server started on ws://localhost:{port}")
        
        # Register signal handlers for smooth shutdown
        loop = asyncio.get_running_loop()
        loop.add_signal_handler(signal.SIGINT, lambda: asyncio.create_task(shutdown(server, loop)))
        loop.add_signal_handler(signal.SIGTERM, lambda: asyncio.create_task(shutdown(server, loop)))

        # Run the server until shutdown is triggered
        await asyncio.Future()  # Run until signal

# Function to start the HTTP server to serve only index.html
def start_http_server(port):
    httpd = HTTPServer(("localhost", port), SingleFileHTTPRequestHandler)
    print(f"HTTP server started on http://localhost:{port}")
    httpd.serve_forever()

if __name__ == "__main__":
    # Parse command-line arguments for WebSocket and HTTP ports
    parser = argparse.ArgumentParser(description="WebSocket server for slider.")
    parser.add_argument('--ws-port', type=int, default=8080, help="Port to run the WebSocket server on (default: 8080)")
    parser.add_argument('--http-port', type=int, default=8000, help="Port to run the HTTP server on (default: 8000)")
    args = parser.parse_args()

    # Ensure the current working directory contains index.html
    if not os.path.isfile("index.html"):
        print("Error: index.html not found in the current directory.")
        exit(1)

    # Start the HTTP server in a separate thread
    http_thread = Thread(target=start_http_server, args=(args.http_port,))
    http_thread.start()

    # Run the WebSocket server with the specified port
    asyncio.run(start_websocket_server(args.ws_port))
