from gz.msgs10.clock_pb2 import Clock

from gz.transport13 import Node
import signal

def handler(msg):
    print(msg)

def main():
    node = Node()
    topic = "clock"

    sub = node.subscribe(Clock, topic, handler)

    signal.pause()

if __name__ == "__main__":
    main()