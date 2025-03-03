from typing import Dict, Callable
from app.core.logger import get_logger
from app.device.handlers.status_handler import handle_device_status

logger = get_logger(__name__)

class SubscriptionManager:
    def __init__(self, mqtt_client):
        self.client = mqtt_client
        self._handlers: Dict[str, Callable] = {}

    def register(self, topic_pattern: str, handler: Callable):
        """注册主题处理函数"""
        self._handlers[topic_pattern] = handler
        self.client.subscribe(topic_pattern, handler)
        logger.info(f"Registered handler for {topic_pattern}")

    def unregister(self, topic_pattern: str):
        """取消订阅"""
        if topic_pattern in self._handlers:
            self.client.unsubscribe(topic_pattern)
            del self._handlers[topic_pattern]
            logger.info(f"Unsubscribed {topic_pattern}")

# 初始化（在main.py中）
def init_subscriptions(app):
    """带安全检查的订阅初始化"""
    if not hasattr(app.state, "mqtt_client"):
        raise RuntimeError("MQTT client not initialized")

    mgr = SubscriptionManager(app.state.mqtt_client)
    mgr.register("devices/+/status", handle_device_status)
    app.state.subscription_mgr = mgr