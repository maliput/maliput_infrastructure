## Wheel generation

### Summary

A dockerfile is provided for creating a [manylinux](https://github.com/pypa/manylinux)-compatible wheels of maliput packages.


It is based on the `quay.io/pypa/manylinux2014_x86_64` image (See [manylinux#docker_images](https://github.com/pypa/manylinux#docker-images)) and adds the corresponding dependencies to build `maliput` packages.

It is verified for the following packages:
- maliput
- maliput_py
- maliput_malidrive

### Usage

```
docker build -t maliput_manylinux2014 .
```

```
docker run --rm -it maliput_manylinux2014 bash
```
