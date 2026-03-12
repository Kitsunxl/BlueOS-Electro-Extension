FROM python:3.9-slim-bullseye

# 用 pip + requirements.txt 安装，精确控制依赖，避免 setup.py 乱拉 ujson 等 C 扩展
COPY app/requirements.txt /requirements.txt
RUN pip install --no-cache-dir -r /requirements.txt

COPY app /app

EXPOSE 80/tcp

LABEL version="1.0.0"
LABEL permissions='\
{\
  "ExposedPorts": {\
    "80/tcp": {}\
  },\
  "HostConfig": {\
    "Privileged": true,\
    "Binds": ["/dev:/dev"],\
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
        "name": "Your Name",\
        "email": "you@example.com"\
    }\
]'
LABEL company='{\
    "about": "",\
    "name": "Your Company",\
    "email": "you@example.com"\
}'
LABEL type="tool"
LABEL tags='["serial", "uart", "monitor"]'
LABEL readme='https://raw.githubusercontent.com/your-repo/main/README.md'
LABEL links='{\
    "website": "https://github.com/your-repo",\
    "support": "https://github.com/your-repo/issues"\
}'
LABEL requirements="core >= 1.1"

ENTRYPOINT ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "80", "--app-dir", "/app"]
