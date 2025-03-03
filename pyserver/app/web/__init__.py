from fastapi import APIRouter

from app.core.middleware import register_middlewares

router = APIRouter()


def init_app(app):
    # 注册路由
    app.include_router(router)

    # 注册中间件
    register_middlewares(app)