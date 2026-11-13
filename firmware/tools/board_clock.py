"""Shared board-time reconstruction. Repeated samples never create time."""
class BoardClock:
    def __init__(self, elapsed=0.0):
        self.elapsed = elapsed
        self.previous = None
        self.changed = False
        self.reset = False
        self.epoch = 0

    def advance(self, milliseconds):
        value = int(milliseconds) & 65535
        self.reset = False
        self.changed = self.previous is None or value != self.previous
        if self.previous is not None:
            delta = (value-self.previous) & 65535
            if delta > 5000:
                self.epoch += 1
                self.reset = True
                self.elapsed += 1.0  # separate reset blocks; never a measured interval
            else:
                self.elapsed += delta/1000.0
        self.previous = value
        return self.elapsed


def reconstruct_times(milliseconds, initial=0.0):
    clock = BoardClock(initial)
    return [clock.advance(value) for value in milliseconds]


def collapse_times(milliseconds, initial=0.0):
    """Keep the last metadata row for each control sample without inventing time."""
    clock = BoardClock(initial)
    times, indices = [], []
    for index, value in enumerate(milliseconds):
        stamp = clock.advance(value)
        if clock.changed:
            times.append(stamp); indices.append(index)
        else:
            indices[-1] = index
    return times, indices


def signed_delta_ms(later, earlier):
    return ((int(later)-int(earlier)+32768) & 65535)-32768
