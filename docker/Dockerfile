FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y -- \
    # C++11コンパイラ、CMake、Gitなどの基本的なビルドツール
    build-essential \
    cmake \
    git \
    wget \
    # Eigen3 (g2oに必要)
    libeigen3-dev \
    # OpenCV (画像処理に必要)
    libopencv-dev \
    # Pangolinの依存関係 (可視化に必要)
    libglew-dev \
    # Python (TUMデータセットの関連付けスクリプトなどに必要)
    libpython2.7-dev \
    libboost-all-dev \
    libssl-dev 

WORKDIR /opt
RUN git clone https://github.com/stevenlovegrove/Pangolin.git \
    && cd Pangolin \
    && git checkout v0.6 \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Debug \
    && make -j 3 \
    && make install
RUN apt-get install -y xcb

CMD ["bash"]