FROM carlasim/carla:0.9.14

USER root

RUN apt-get update && apt-get -y upgrade \
      && apt-get install -y --no-install-recommends \
    git \
    wget \
    g++ \
    gcc \
    sudo \
    ca-certificates \
    graphviz

RUN mkdir /carla_sgg/
COPY ./environment.yml /carla_sgg/environment.yml
ENV PATH="/root/miniconda3/bin:${PATH}"
ARG PATH="/root/miniconda3/bin:${PATH}"

RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
    && mkdir /root/.conda \
    && bash Miniconda3-latest-Linux-x86_64.sh -b \
    && rm -f Miniconda3-latest-Linux-x86_64.sh \
    && conda init bash \
    && . /root/.bashrc \
    && conda update conda \
    && conda env create python=3.7 -f /carla_sgg/environment.yml


ENV PYTHONPATH="/home/carla/PythonAPI/carla/:${PYTHONPATH}"
ARG PYTHONPATH="/home/carla/PythonAPI/carla/:${PYTHONPATH}"
ENV MPLBACKEND="TkAgg"
ARG MPLBACKEND="TkAgg"

COPY ./docker_example.sh /docker_example_internal.sh
RUN chmod +x /docker_example_internal.sh
