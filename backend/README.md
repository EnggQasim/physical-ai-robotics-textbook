---
title: Physical AI Textbook API
emoji: 🤖
colorFrom: green
colorTo: blue
sdk: docker
pinned: false
license: mit
app_port: 7860
---

# Physical AI Textbook RAG API

A RAG-powered chatbot API for the Physical AI & Humanoid Robotics textbook.

## Features

- Semantic search over textbook content using OpenAI embeddings
- Grounded responses with source citations
- FastAPI backend with health monitoring

## Endpoints

### Core
- `GET /` - API info
- `GET /health` - Health check

### Chat (RAG)
- `POST /api/chat/` - Chat with RAG context
- `GET /api/chat/search` - Search textbook content

### Diagram Generator
- `POST /api/diagram/generate` - Generate AI diagrams
- `GET /api/diagram/predefined` - List predefined diagrams
- `GET /api/diagram/cached` - List cached diagrams
- `GET /api/diagram/concept/{id}` - Get specific diagram

### Podcast Generator
- `GET /api/podcast/providers` - List TTS providers (OpenAI, Higgs Audio)
- `GET /api/podcast/providers/higgs/status` - Check Higgs Audio availability
- `POST /api/podcast/generate` - Generate podcast from chapter
- `GET /api/podcast/chapters` - List chapters with podcast info
- `GET /api/podcast/list` - List generated podcasts
- `GET /api/podcast/{id}` - Get specific podcast
- `GET /api/podcast/chapter/{id}/info` - Get chapter podcast info

## Environment Variables

Set the following secrets in your Hugging Face Space:
- `OPENAI_API_KEY` - Your OpenAI API key
- `GEMINI_API_KEY` - Your Google Gemini API key (for diagrams)
