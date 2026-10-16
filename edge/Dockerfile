FROM python:3.12-slim

ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY tdoa_server ./tdoa_server

# Unprivileged: the service only opens outbound MQTT connections.
RUN useradd --system --no-create-home edge
USER edge

CMD ["python", "-m", "tdoa_server"]
