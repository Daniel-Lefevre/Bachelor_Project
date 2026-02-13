from collections import deque

class CustomPriorityQueue:
    def __init__(self, numbers_of_priorities_in_system):
        self.queues = []
        for i in range(numbers_of_priorities_in_system):
            self.queues.append(deque())

    def get(self):
        for queue in self.queues:
            if (queue):
                return queue.popleft()
        return None

    def put(self, parameter):
        priority, value = parameter
        print(f"Value added to queue: {value.shape}, {value.color}")
        self.queues[priority - 1].append(value)

    def peek(self):
        for queue in self.queues:
            if (not queue):
                continue
            else:
                return queue[0]
        return None

    def empty(self):
        for queue in self.queues:
            if (queue):
                return False

        return True
