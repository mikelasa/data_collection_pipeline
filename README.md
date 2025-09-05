# data_collection_pipeline
This is a pipeline to collect data from human ddemonstrations for franka robot with a realsense D415 camera

# installation
1.- install mamba first

2.- create the environment called collector_env
mamba env create -f environment.yml

3.- activate env
mamba activate collector_env

4.- install libfranka 0.9.2 version (since we have the old FER)
sudo apt-get update
sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev
git clone --branch 0.9.2 --recursive https://github.com/frankaemika/libfranka libfranka-0.9.2
cd libfranka-0.9.2/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=$HOME/.local -DBUILD_EXAMPLES=OFF
make -j$(($(nproc)-1))


5.- build data_collection
cd ../..
cd data_collection
mkdir build && cd build
cmake -S . -B build   -DCMAKE_BUILD_TYPE=Release   -DCMAKE_PREFIX_PATH=$HOME/.local   -DFranka_DIR=$HOME/.local/lib/cmake/Franka
cmake --build build -j