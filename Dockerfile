# On part d'une image qui contient déjà les headers de Webots
FROM cyberbotics/webots:latest

# On repasse en root pour installer OpenCV
USER root
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    pkg-config \
    build-essential \
    python3-pandas \
    python3-numpy \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# Définition du répertoire de travail
WORKDIR /app

# Copie de ton projet
COPY . .

# On définit WEBOTS_HOME (indispensable pour le Makefile)
ENV WEBOTS_HOME=/usr/local/webots

# Compilation du contrôleur
RUN cd controllers/EPuckLineFollowerOOP && make