"""
Example client for testing the controller server.

Connects to HOST:PORT, receives packets, and prints [X, Y, Z, RX, RY, RZ].
"""

import socket
import struct
import time

HOST = "127.0.0.1"   # change to server IP when testing over network
PORT = 12345

LENGTH_STRUCT = struct.Struct("!Q")
VECTOR_STRUCT = struct.Struct("!6f")


def recv_exact(sock: socket.socket, n: int) -> bytes:
    """Receive exactly n bytes or raise EOFError if connection closes."""
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise EOFError("Connection closed by server")
        data += chunk
    return data


def main() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print(f"Connecting to {HOST}:{PORT}...")
        s.connect((HOST, PORT))
        print("Connected. Press Ctrl+C to stop.")

        try:
            while True:
                # Read length prefix
                header = recv_exact(s, LENGTH_STRUCT.size)
                (length,) = LENGTH_STRUCT.unpack(header)

                # Read payload
                payload = recv_exact(s, length)
                vector = VECTOR_STRUCT.unpack(payload)

                print("Received vector:",
                      " ".join(f"{v:+.3f}" for v in vector))

        except (EOFError, KeyboardInterrupt):
            print("Connection closed.")


if __name__ == "__main__":
    main()
