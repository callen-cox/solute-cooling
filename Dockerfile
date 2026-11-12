FROM python:3.12-slim
WORKDIR /app
COPY . .
RUN pip install --no-cache-dir fastapi uvicorn
EXPOSE 8000
CMD ["python", "-m", "coolant.server"]
