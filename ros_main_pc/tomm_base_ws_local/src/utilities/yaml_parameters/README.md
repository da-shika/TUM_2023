# yaml_parameters

Tool for loading parameters directly from yaml files instead of from ros parameter server.

## Prerequisites
Install [`yaml-cpp`](https://github.com/jbeder/yaml-cpp), a YAML parse (and emitter if needed) for C++
```
git clone https://github.com/jbeder/yaml-cpp.git

cd yaml-cpp

mkdir build

cd build

cmake -DYAML_BUILD_SHARED_LIBS=ON ..

sudo make install
```

## Usage
See examples in *src/tests* with yaml files in *configs/test.yaml*