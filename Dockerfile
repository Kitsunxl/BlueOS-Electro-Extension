FROM python:3.9-slim-bullseye

RUN apt-get update && apt-get install -y \
    build-essential \
    gcc \
    g++ \
    make \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

COPY install.sh /install.sh

RUN chmod +x /install.sh && /install.sh

COPY app /app
RUN python /app/setup.py install

EXPOSE 80/tcp

LABEL version="1.0.1"
# TODO: Add a Volume for persistence across boots
LABEL permissions='\
{\
  "ExposedPorts": {\
    "80/tcp": {}\
  },\
  "HostConfig": {\
    "Privileged": true,\
    "Binds":["/root/.config:/root/.config"],\
    "PortBindings": {\
      "80/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    }\
  }\
}'
LABEL authors='[\
    {\
        "name": "Willian Galvani",\
        "email": "willian@bluerobotics.com"\
    }\
]'
LABEL company='{\
        "about": "",\
        "name": "Blue Robotics",\
        "email": "support@bluerobotics.com"\
    }'
LABEL type="example"
LABEL tags='[\
        "interaction"\
    ]'
LABEL readme='https://raw.githubusercontent.com/Williangalvani/BlueOS-examples/{tag}/example5-gpio-control/Readme.md'
LABEL links='{\
        "website": "https://github.com/Williangalvani/BlueOS-examples/",\
        "support": "https://github.com/Williangalvani/BlueOS-examples/"\
    }'
LABEL requirements="core >= 1.1"

ENTRYPOINT ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "80", "--app-dir", "/app"]
