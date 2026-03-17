FROM python:3.9-slim-bullseye

RUN mkdir -p /home/pi
RUN usermod -aG dialout root

COPY serial-reader /home/pi/serial-reader
RUN cd /home/pi/serial-reader && pip3 install . && chmod +x main.py

LABEL version="1.0.0"
LABEL permissions='{"ExposedPorts":{"9001/tcp":{}},"HostConfig":{"Privileged":true,"Binds":["/dev:/dev"],"ExtraHosts":["host.docker.internal:host-gateway"],"PortBindings":{"9001/tcp":[{"HostPort":""}]}}}'
LABEL authors='[{"name":"Your Name","email":"you@example.com"}]'
LABEL company='{"about":"","name":"Your Company","email":"you@example.com"}'
LABEL type="tool"
LABEL tags='["serial","uart","monitor"]'
LABEL readme='https://raw.githubusercontent.com/your-repo/main/README.md'
LABEL links='{"website":"https://github.com/your-repo","support":"https://github.com/your-repo/issues"}'
LABEL requirements="core >= 1.1"

ENTRYPOINT /home/pi/serial-reader/main.py