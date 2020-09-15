from abc import ABC

from lobster_simulator.common.pybullet_api import PybulletAPI


class PyBulletObject(ABC):

    def __init__(self, object_id):
        self._object_id = object_id

    def remove(self) -> None:
        """
        Removes current object from GUI
        """
        PybulletAPI.removeUserDebugItem(self._object_id)
        self._object_id = None

    @property
    def object_id(self):
        if self._object_id is None:
            print("Trying to get object id but object has been removed or not instantiated")
        return self._object_id
