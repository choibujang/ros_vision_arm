import socket
import struct
from collections import defaultdict
import time

class UDPServer:
    def __init__(self, queue):
        self.host = 'localhost'
        self.port = 8080
        self.queue = queue

        self.HEADER_SIZE = 12
        self.MAX_UDP_PAYLOAD = 65000
        self.MAX_WAIT_TIME = 0.2
        self.RECEIVE_THRESHOLD = 80

        # { (device_id, frame_id) : { 'chunks': {}, 'total': N, 'start': time.time() } }
        self.recv_buffers = defaultdict(lambda: { 'chunks': {}, 'total': None, 'start': time.time() })

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))

        print(f"Start UDP server on {self.host}:{self.port}")

    """"
    UDP로 수신되는 청크들을 조립해서 완전한 프레임으로 만든 후 큐에 넣는 함수.
    """
    def receiver(self):
        while True:
            data, addr = self.sock.recvfrom(self.MAX_UDP_PAYLOAD)
            if len(data) < self.HEADER_SIZE:
                continue

            device_id = int.from_bytes(data[0:4], 'little')
            frame_id = int.from_bytes(data[4:8], 'little')
            chunk_idx = int.from_bytes(data[8:10], 'little')
            total_chunks = int.from_bytes(data[10:12], 'little')
            payload = data[self.HEADER_SIZE:]

            key = (device_id, frame_id)
            buf = self.recv_buffers[key]

            buf['chunks'][chunk_idx] = payload
            buf['total'] = total_chunks
            buf['start'] = buf.get('start', time.time())

            if len(buf['chunks']) == total_chunks:
                self.complete_and_enqueue(key)

    def complete_and_enqueue(self, key):
        buf = self.recv_buffers[key]
        chunks = buf['chunks']
        if len(chunks) < buf['total']:
            return

        data = b''.join([chunks[i] for i in sorted(chunks.keys())])
        self.queue.put({ 'device_id': key[0], 'frame_id': key[1], 'data': data })
        del self.recv_buffers[key]

    def buffer_cleaner(self):
        while True:
            now = time.time()
            expired = []
            for key, buf in list(self.recv_buffers.items()):
                elapsed = now - buf['start']
                if elapsed > self.MAX_WAIT_TIME:
                    if len(buf['chunks']) >= buf['total'] * self.RECEIVE_THRESHOLD:
                        self.complete_and_enqueue(key)
                    else:
                        print(f"Dropped frame {key} (only {len(buf['chunks'])}/{buf['total']})")
                        expired.append(key)
            for key in expired:
                if key in self.recv_buffers:
                    del self.recv_buffers[key]

            time.sleep(0.05)