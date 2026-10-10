ARG IDF_IMAGE=espressif/idf@sha256:1355cce31b723f9c66ccbadc1d1e066653f463172452260c86aac194ecbfea95
FROM ${IDF_IMAGE}

WORKDIR /project

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN apt-get update && apt-get install -y \
    git \
    curl \
    udev \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> ~/.bashrc

COPY . .

ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]
CMD ["/bin/bash", "-c"]
