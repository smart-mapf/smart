ROOT_DIR=$(pwd)

# ─── RPC ──────────────────────────────────────────────────────────────────────

cd client
mkdir externalDependencies
cd externalDependencies
git clone --recurse-submodules https://github.com/rpclib/rpclib
cd rpclib
mkdir build
cd build
cmake ..
make
sudo make install

cd "$ROOT_DIR"

# ─── Client ───────────────────────────────────────────────────────────────────

cd client
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make

cd "$ROOT_DIR"

# ─── Server ───────────────────────────────────────────────────────────────────

cd server
mkdir build
cd build
cmake ..
make
