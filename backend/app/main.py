"""FastAPI application for Physical AI Textbook RAG Chatbot."""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from app.config import get_settings
from app.routers import chat_router, health_router, diagram_router, podcast_router, auth_router
from app.services.vector_store import get_vector_store
from app.services.indexer import index_all_content


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan events."""
    # Startup: Initialize vector store and index content
    try:
        vector_store = get_vector_store()
        created = vector_store.ensure_collection()
        if created:
            print(f"Created Qdrant collection: {vector_store.collection_name}")
        else:
            print(f"Using existing Qdrant collection: {vector_store.collection_name}")

        # Check if collection is empty and index content
        info = vector_store.get_collection_info()
        if info.get("points_count", 0) == 0:
            print("Collection is empty, indexing content...")
            indexed = index_all_content(vector_store)
            print(f"Indexed {indexed} chunks on startup")
        else:
            print(f"Collection has {info.get('points_count')} points, skipping indexing")

    except Exception as e:
        print(f"Warning: Could not initialize vector store: {e}")

    yield

    # Shutdown: Cleanup if needed
    print("Shutting down...")


app = FastAPI(
    title="Physical AI Textbook API",
    description="RAG-powered chatbot for the Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    lifespan=lifespan,
)

# Configure CORS
settings = get_settings()
origins = [origin.strip() for origin in settings.cors_origins.split(",")]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(health_router)
app.include_router(chat_router, prefix="/api")
app.include_router(diagram_router, prefix="/api")
app.include_router(podcast_router, prefix="/api")
app.include_router(auth_router, prefix="/api")


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI Textbook API",
        "docs": "/docs",
        "health": "/health",
    }
