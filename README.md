sudo apt-get update
# data_collection_pipeline
This is a pipeline to collect data from human demonstrations for the Franka robot using an Intel RealSense D415 camera.

## Installation 

1) Install mamba (if you don't already have it).

2) Create the conda environment named `collector_env`:

```bash
mamba env create -f environment.yml
```

3) Activate the environment:

```bash
mamba activate collector_env
```

4) Install libfranka v0.9.2 (required for this project):

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev
git clone --branch 0.9.2 --recursive https://github.com/frankaemika/libfranka libfranka-0.9.2
cd libfranka-0.9.2/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
	-DBUILD_TESTS=OFF \
	-DCMAKE_INSTALL_PREFIX=$HOME/.local \
	-DBUILD_EXAMPLES=OFF
make -j$(($(nproc)-1))
```

5) Build the `data_collection` project:

```bash
cd ../..
cd data_collection
mkdir build && cd build
cmake -S . -B build \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_PREFIX_PATH=$HOME/.local \
	-DFranka_DIR=$HOME/.local/lib/cmake/Franka
cmake --build build -j
```

That's it — the commands above are rendered as code blocks so they are easier to copy and run.
