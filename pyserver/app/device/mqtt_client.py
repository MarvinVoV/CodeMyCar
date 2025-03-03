import asyncio
import json
from typing import Dict, Callable

import paho.mqtt.client as mqtt
from fastapi import Depends
from app.config.settings import get_settings
from app.config.settings import Settings
from app.core.logger import get_logger

logger = get_logger(__name__)


class MQTTClientWrapper:
    def __init__(self, settings: Settings):
        self.client = mqtt.Client(
            client_id=settings.MQTT_CLIENT_ID
        )
        self.settings = settings
        self._message_queue = asyncio.Queue(maxsize=100)
        self._overflow_strategy = "drop_oldest"  # 或 "reject_new"
        self._subscriptions: Dict[str, Callable] = {}
        # 配置回调
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

    def _on_connect(self, client, userdata, flags, rc):
        """连接成功回调"""
        logger.info(f"Connected to MQTT broker with code {rc}")
        # 自动重订阅
        for topic in self._subscriptions:
            client.subscribe(topic, qos=1)

    def _on_disconnect(self, client, userdata, rc):
        """断开连接回调"""
        logger.warning(f"Disconnected from broker (code: {rc})")

    def _on_message(self, client, userdata, msg):
        """原始消息接收回调"""
        try:
            payload = msg.payload.decode()
            asyncio.create_task(
                self._process_message(msg.topic, payload)
            )
        except Exception as e:
            logger.error(f"Message processing failed: {str(e)}")

    async def connect(self):
        """异步连接方法"""
        self.client.connect(
            self.settings.MQTT_BROKER,
            self.settings.MQTT_PORT,
            keepalive=self.settings.MQTT_KEEPALIVE
        )
        self.client.loop_start()
        logger.info("MQTT client started")

    async def publish(self, topic: str, payload: str):
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            lambda: self.client.publish(topic, payload)
        )

    async def disconnect(self):
        """增强健壮性的断开连接"""
        try:
            # 先清空队列
            while not self._message_queue.empty():
                self._message_queue.get_nowait()

            # 停止网络循环
            self.client.loop_stop()

            # 安全断开连接
            if self.client.is_connected():
                self.client.disconnect()

            logger.info("MQTT client stopped successfully")
        except Exception as e:
            logger.error(f"Disconnection failed: {str(e)}")
            raise

    def subscribe(self, topic: str, handler: Callable):
        """注册主题处理器"""
        self._subscriptions[topic] = handler
        self.client.subscribe(topic, qos=1)
        logger.info(f"Subscribed to {topic}")

    async def _process_message(self, topic: str, payload: str):
        """增强消息处理安全"""
        try:
            # 从队列获取消息（带超时）
            async with asyncio.timeout(5):
                data = json.loads(payload)
                handler = self._subscriptions.get(topic)

                if handler:
                    await handler(data)
                else:
                    logger.warning(f"No handler for {topic}")

        except json.JSONDecodeError:
            logger.error(f"Invalid JSON in {payload}")
        except asyncio.TimeoutError:
            logger.warning("Message processing timed out")
        except Exception as e:
            logger.error(f"Processing error: {str(e)}")

    async def _reconnect(self):
        while True:
            try:
                await self.connect()
                break
            except Exception as e:
                logger.error(f"Reconnect failed: {str(e)}")
                await asyncio.sleep(5)


# 依赖注入
def get_mqtt_client(settings=Depends(get_settings))->MQTTClientWrapper:
    client = MQTTClientWrapper(settings)
    return client
