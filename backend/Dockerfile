FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY app/ ./app/
COPY content/ ./content/

# Set environment variables
ENV QDRANT_USE_MEMORY=true
ENV PYTHONUNBUFFERED=1

# Expose port (Hugging Face uses 7860)
EXPOSE 7860

# Run the application
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "7860"]
