from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates

from .routers import orders

templates = Jinja2Templates(directory="app/templates")


def create_app() -> FastAPI:
    app = FastAPI(title="UDS Web Backend", version="0.1.0")

    # 订单相关接口（前端主要入口）
    app.include_router(orders.router, prefix="/orders", tags=["orders"])

    @app.get("/", response_class=HTMLResponse)
    async def index(request: Request):
        """
        极简 Web 页面：
        - 左侧表单创建订单（调用 POST /orders）
        - 右侧表格展示当前所有订单（调用 GET /orders）
        """
        return templates.TemplateResponse("index.html", {"request": request})

    return app


app = create_app()

