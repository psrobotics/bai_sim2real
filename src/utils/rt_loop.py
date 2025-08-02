import threading
import time
import sys
from typing import Callable

class real_time_loop:
    """
    An abstract base class for creating high-frequency loops in Python.
    """
    def __init__(self, name: str, period: float):

        self._name = name
        self._period = period
        self._is_running = False
        self._thread = None
        # Event for interruptible sleep and clean shutdown
        self._shutdown_event = threading.Event()

    def start(self):
        if self._is_running:
            print(f"WARN: Loop '{self._name}' is already running.", file=sys.stderr)
            return
        
        print(f"INFO: Starting loop '{self._name}'...")
        self._is_running = True
        self._shutdown_event.clear()
        self._thread = threading.Thread(target=self._entry_func, name=self._name)
        self._thread.start()

    def shutdown(self):
        if not self._is_running:
            return

        self._is_running = False
        self._shutdown_event.set() # Wake up the thread if it's sleeping
        if self._thread and self._thread.is_alive():
            self._thread.join()
        print(f"INFO: Loop '{self._name}' has been shut down.", file=sys.stderr)


    def function_cb(self):
        raise NotImplementedError("This method must be implemented by a subclass.")

    def _entry_func(self):
        """
        The main entry point for the thread.
        """
        next_time = time.monotonic()
        while self._is_running:
            # Execute the  callback function
            self.function_cb()
            next_time += self._period
            
            # Check for deadline miss before sleeping
            now = time.monotonic()
            if now > next_time:
                missed_by = now - next_time
                print(
                    f"WARN: Loop '{self._name}' missed its deadline by {missed_by:.6f} s",
                    file=sys.stderr
                )
                # Reset next_time to prevent runaway loop if the system is heavily loaded
                next_time = now + self._period

            # Interruptible sleep
            sleep_duration = next_time - time.monotonic()
            if sleep_duration > 0:
                self._shutdown_event.wait(timeout=sleep_duration)

class loop_func(real_time_loop):
    """
    A concrete implementation of real_time_loop that executes a given function.
    """
    def __init__(self, name: str, period: float, cb: Callable[[], None]):
        super().__init__(name, period)
        self._fp = cb

    def function_cb(self):
        if self._fp:
            self._fp()


if __name__ == "__main__":
    
    # Simple periodic task
    def my_task():
        print("Executing my_task...")

    print("\nStarting Example 1 -> Simple Task")
    loop1 = loop_func(name="simple_task_loop", period=0.5, cb=my_task)
    loop1.start()
    time.sleep(2.1)
    loop1.shutdown()
    print("-" * 20)
    
    # class method as a callback
    class my_counter:
        def __init__(self):
            self.count = 0
        
        def increment(self):
            self.count += 1
            print(f"Executing my_counter.increment... Count: {self.count}")

    print("\nStarting Example 2 -> Class Method")
    counter_obj = my_counter()
    loop2 = loop_func(name="counter_loop", period=0.25, cb=counter_obj.increment)
    
    try:
        loop2.start()
        print("Running counter loop for 3 seconds. Press Ctrl+C to exit early.")
        time.sleep(3)
    except KeyboardInterrupt:
        print("\nCtrl+C received.")
    finally:
        loop2.shutdown()
        
    print("\nExample script finished.")
