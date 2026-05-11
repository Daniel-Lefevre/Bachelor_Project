from collections import deque


class CustomPriorityQueue:
    def __init__(self, numbers_of_priorities_in_system, printing=False):
        self.queues = []
        self.printing = printing
        for _ in range(numbers_of_priorities_in_system):
            self.queues.append(deque())

    def get(self):
        for queue in self.queues:
            if queue:
                return queue.popleft()
        return None

    def put(self, parameter):
        priority, value = parameter
        self.queues[priority - 1].append(value)
        if self.printing:
            print(f"Queue: {priority}")
            print({k: v for k, v in vars(value).items() if k != "states"})

    def peek(self):
        for queue in self.queues:
            if not queue:
                continue
            else:
                return queue[0]
        return None

    def empty(self):
        for queue in self.queues:
            if queue:
                return False

        return True
