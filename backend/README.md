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

- `GET /` - API info
- `GET /health` - Health check
- `POST /api/chat/` - Chat endpoint

## Environment Variables

Set the following secret in your Hugging Face Space:
- `OPENAI_API_KEY` - Your OpenAI API key
