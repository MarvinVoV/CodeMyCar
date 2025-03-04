import datetime
import logging
from pathlib import Path
from typing import Optional, List
import structlog
from structlog.types import Processor
from structlog.stdlib import ProcessorFormatter, BoundLogger
from logging.handlers import RotatingFileHandler

from app.config.settings import Settings


def setup_logging(settings: Settings):
    """配置 structlog 和标准库 logging，满足不同环境和设备日志需求。"""
    # 创建日志目录
    log_dir = Path(settings.LOG_FILE_PATH).parent if settings.LOG_FILE_PATH else Path("logs")
    log_dir.mkdir(parents=True, exist_ok=True)

    # structlog 处理器链
    processors: List[Processor] = [
        structlog.contextvars.merge_contextvars,
        structlog.stdlib.filter_by_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        add_custom_timestamp,
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
        structlog.stdlib.ProcessorFormatter.wrap_for_formatter,
    ]

    # 配置 structlog
    structlog.configure(
        processors=processors,
        logger_factory=structlog.stdlib.LoggerFactory(),
        wrapper_class=structlog.stdlib.BoundLogger,
        cache_logger_on_first_use=True,
    )

    # 配置标准库 logging 的处理器和日志器
    handlers = []

    if settings.ENV == "dev":
        # 开发环境：所有日志输出到控制台
        console_handler = logging.StreamHandler()
        console_formatter = ProcessorFormatter(
            processor=_get_console_renderer(),
            foreign_pre_chain=processors[:-1],
        )
        console_handler.setFormatter(console_formatter)
        handlers.append(console_handler)
    else:
        # 生产环境：应用主日志和设备日志输出到文件
        # 应用主日志文件处理器
        app_handler = RotatingFileHandler(
            filename=log_dir / "app.log",
            maxBytes=settings.LOG_MAX_SIZE,
            backupCount=settings.LOG_BACKUP_COUNT,
        )
        app_formatter = ProcessorFormatter(
            processor=structlog.processors.JSONRenderer(),
            foreign_pre_chain=processors[:-1],
        )
        app_handler.setFormatter(app_formatter)
        handlers.append(app_handler)

        # 设备日志文件处理器
        devices = ["stm32", "esp32"]
        for device in devices:
            handler = RotatingFileHandler(
                filename=log_dir / f"{device}.log",
                maxBytes=settings.LOG_MAX_SIZE,
                backupCount=settings.LOG_BACKUP_COUNT,
            )
            formatter = ProcessorFormatter(
                processor=structlog.processors.JSONRenderer(),
                foreign_pre_chain=processors[:-1],
            )
            handler.setFormatter(formatter)
            # 创建设备专用的日志器并添加处理器
            device_logger = logging.getLogger(f"device.{device}")
            device_logger.addHandler(handler)
            device_logger.setLevel(settings.LOG_LEVEL.upper())
            device_logger.propagate = False  # 阻止传播到根日志器

    # 配置根日志器
    root_logger = logging.getLogger()
    root_logger.handlers = handlers
    root_logger.setLevel(settings.LOG_LEVEL.upper())


def add_custom_timestamp(logger, method_name, event_dict):
    """添加自定义时间戳，精确到毫秒。"""
    now = datetime.datetime.now()
    event_dict['timestamp'] = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    return event_dict


def _get_console_renderer() -> structlog.dev.ConsoleRenderer:
    """控制台渲染器（无颜色依赖）。"""
    return structlog.dev.ConsoleRenderer(
        pad_event=30,
        colors=True,
        exception_formatter=structlog.dev.plain_traceback
    )


def get_logger(name: Optional[str] = None) -> BoundLogger:
    """获取 structlog 日志器。"""
    return structlog.get_logger(name or __name__)


def get_stm32_logger() -> BoundLogger:
    """获取 stm32 设备专用的 structlog 日志器。"""
    return structlog.get_logger("device.stm32")


def get_esp32_logger() -> BoundLogger:
    """获取 esp32 设备专用的 structlog 日志器。"""
    return structlog.get_logger("device.esp32")
