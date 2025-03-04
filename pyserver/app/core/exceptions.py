from fastapi import HTTPException, Request
from fastapi.responses import JSONResponse

class CustomHTTPException(HTTPException):
    def __init__(self, status_code: int, code: int, message: str, detail: str = None):
        super().__init__(status_code=status_code, detail=detail)
        self.code = code
        self.message = message

async def custom_http_exception_handler(request: Request, exc: CustomHTTPException):
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "code": exc.code,
            "message": exc.message,
            "detail": exc.detail
        },
    )

async def general_exception_handler(request: Request, exc: Exception):
    return JSONResponse(
        status_code=500,
        content={
            "code": 5000,
            "message": "Internal Server Error",
            "detail": str(exc)
        }
    )

def register_exceptions(app):
    app.add_exception_handler(CustomHTTPException, custom_http_exception_handler)
    app.add_exception_handler(Exception, general_exception_handler)