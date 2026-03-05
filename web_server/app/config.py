import os

class Settings:
    PROJECT_NAME: str = "UAV Delivery System"
    VERSION: str = "2.0.0"
    
    # gRPC Connection to Drone (Mock or Real)
    GRPC_TARGET: str = os.getenv("GRPC_TARGET", "localhost:50051")
    
    # WebRTC Signaling Server URL (Mock)
    WEBRTC_SIGNALING_URL: str = os.getenv("WEBRTC_SIGNALING_URL", "http://localhost:8081/offer")

settings = Settings()
