import sys
from pathlib import Path
from typing import Optional
import structlog
from structlog.types import Processor

from app.config.settings import Settings


def setup_logging(settings: Settings):
    """纯 structlog 日志配置"""
    # 创建日志目录
    if settings.LOG_FILE_PATH:
        log_path = Path(settings.LOG_FILE_PATH)
        log_path.parent.mkdir(parents=True, exist_ok=True)

    # 统一处理器配置
    processors: list[Processor] = [
        structlog.contextvars.merge_contextvars,
        structlog.stdlib.filter_by_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
    ]

    # 开发环境配置
    if settings.ENV == "dev":
        processors.append(_get_console_renderer())
    else:
        processors.extend([
            structlog.processors.dict_tracebacks,
            structlog.processors.JSONRenderer()
        ])

    # 生产环境文件输出
    if settings.LOG_FILE_PATH and settings.ENV != "dev":
        file_processor = structlog.WriteFiles(
            path=settings.LOG_FILE_PATH,
            rotate=settings.LOG_MAX_SIZE,
            max_files=settings.LOG_BACKUP_COUNT
        )
        processors.append(file_processor)

    # 最终配置
    structlog.configure(
        processors=processors,
        wrapper_class=structlog.make_filtering_bound_logger(
            # 将日志级别字符串转换为对应的数值
            _get_structlog_level(settings.LOG_LEVEL)
        ),
        context_class=dict,
        logger_factory=structlog.stdlib.LoggerFactory(),
        cache_logger_on_first_use=True
    )


def _get_structlog_level(level_str: str) -> int:
    """将日志级别字符串转换为structlog级别数值"""
    level_map = {
        "debug": 10,
        "info": 20,
        "warning": 30,
        "error": 40,
        "critical": 50
    }
    return level_map.get(level_str.lower(), 20)  # 默认info级别


def _get_console_renderer() -> structlog.dev.ConsoleRenderer:
    """控制台渲染器（移除 colorama 依赖）"""
    return structlog.dev.ConsoleRenderer(
        colors=sys.stdout.isatty()  # 自动检测终端颜色支持
    )


def get_logger(name: Optional[str] = None) -> structlog.stdlib.BoundLogger:
    """获取日志记录器"""
    return structlog.get_logger(name or __name__)
