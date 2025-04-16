import socket
import struct

class UDPReceiver:
    def __init__(self, queue, port, host='localhost'):
        self.host = host
        self.port = port

        self.queue = queue

        self.HEADER_SIZE = 8
        self.FRAME_END = b"END"
        self.MAX_UDP_PAYLOAD = 65000

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))

        print(f"Start UDP server on {self.host}:{self.port}")

    """"
    UDP로 수신되는 청크들을 조립해서 완전한 프레임으로 만든 후 큐에 넣는 함수.
    각 프레임은 여러 청크로 나뉘어 전송되며 'END' 시그널이 오면 프레임들을 처리한다.
    """
    def start(self):
        print(f"Listening on {self.host}:{self.port}")
        received_chunks = {} # {frame_id: {chunk_idx: chunk_data}}
        total_chunks_expected = {} # {frame_id: total_chunks}

        while True:
            data, addr = self.sock.recvfrom(self.MAX_UDP_PAYLOAD)

            if data == self.FRAME_END:
                for frame_id in sorted(received_chunks.keys()):
                    chunks = received_chunks[frame_id]
                    expected_count = total_chunks_expected[frame_id]

                    if len(chunks) == expected_count:
                        frame_bytes = []
                        for i in range(expected_count):
                            frame_bytes.append(chunks[i])
                        full_frame = b"".join(frame_bytes)

                        self.queue.put(full_frame)

                received_chunks.clear()
                total_chunks_expected.clear()

            else:
                frame_id, chunk_idx, total_chunks = struct.unpack("IHH", data[:self.HEADER_SIZE])
                chunk_data = data[self.HEADER_SIZE:]

                if frame_id not in received_chunks:
                    received_chunks[frame_id] = {}
                    total_chunks_expected[frame_id] = total_chunks

                received_chunks[frame_id][chunk_idx] = chunk_data