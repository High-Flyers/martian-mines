from abc import ABC, abstractmethod


class DroneBase(ABC):
    @abstractmethod
    def set_fixed_heading(self, fixed_heading):
        pass

    @abstractmethod
    async def start(self):
        pass

    @abstractmethod
    async def connect(self):
        pass

    @abstractmethod
    def get_telem(self):
        pass
