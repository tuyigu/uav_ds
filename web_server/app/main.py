from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
import logging
import asyncio
import json

from .config import settings
from .services.grpc_client import drone_client
from .routers import api, pages

# Configure Logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("WebServer")

def create_app() -> FastAPI:
    app = FastAPI(title=settings.PROJECT_NAME, version=settings.VERSION)

    # CORS
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Mount Static & Templates
    app.mount("/static", StaticFiles(directory="app/static"), name="static")
    
    # Include Routers
    app.include_router(pages.router)
    app.include_router(api.router, prefix="/api")

    @app.on_event("startup")
    async def startup_event():
        logger.info("Connecting to Drone gRPC Service...")
        await drone_client.connect()

    return app

app = create_app()
