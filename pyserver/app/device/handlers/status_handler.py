from paho.mqtt.client import topic_matches_sub

from app.core.logger import get_logger
from device.protocol.builder import DevicePayload

async def handle_device_heartbeat(self, data: dict):
    pass
