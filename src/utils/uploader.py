import queue
import requests

from threading import Thread
from time import sleep


class Uploader:
    """
    Class for uploading some data to the server.

    params:
    url: string specyfing the server address (should contain endpoint).
    upload_interval: float specifying the wait time between requests, default to 0.5.
    """

    def __init__(self, url, upload_interval=0.5):
        self._url = url
        self._data_queue = queue.Queue()
        self._upload_interval = upload_interval
        self._thread = Thread(target=self._run, daemon=True)

        self._thread.start()

    def add(self, item):
        """Add an item to the sending queue."""
        self._data_queue.put(item, block=False)

    def _run(self):
        while True:
            try:
                data = self._data_queue.get()
                response = self._send(data)
                print(response)
                self._data_queue.task_done()
            except Exception as e:
                print(f"Uploader error: {e=}")
            sleep(self._upload_interval)

    def _send(self, data):
        response = requests.post(self._url, data=data)
        return response
