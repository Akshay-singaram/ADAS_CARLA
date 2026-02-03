"""
Event Bus

Lightweight synchronous pub/sub transport. publish() calls subscribers
immediately in registration order — no queues, no threads.
"""


class EventBus:
    def __init__(self):
        self._subscribers = {}  # event_name -> [(callback, owner_label)]

    def subscribe(self, event_name, callback, owner=None):
        """Register a callback for an event."""
        self._subscribers.setdefault(event_name, []).append((callback, owner))

    def publish(self, event_name, payload=None):
        """Call every subscriber for event_name in registration order."""
        for callback, _ in self._subscribers.get(event_name, []):
            callback(payload)

    def unsubscribe(self, event_name, callback):
        """Remove a specific callback from an event."""
        if event_name in self._subscribers:
            self._subscribers[event_name] = [
                (cb, owner) for cb, owner in self._subscribers[event_name]
                if cb is not callback
            ]

    def get_subscribers(self, event_name):
        """Return list of (callback, owner) for introspection."""
        return list(self._subscribers.get(event_name, []))
