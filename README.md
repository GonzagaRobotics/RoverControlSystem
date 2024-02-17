# Rover Control System (RCS)

## Introduction

TLDR: Overall project and software to control the rover.

## Quick Start

The main script to run is `start.sh`.

```bash
# To run as the "host" (i.e. development)
sh start.sh host

# To run as the "jetson" (i.e. production)
sh start.sh jetson

# An optional flag can be passed after to force a rebuild of the docker image
sh start.sh $TARGET -b
```

Once `start.sh` has run, you will be inside the docker container. After a full build, the RCS packages will need to be built. This can be done with:

```bash
source build.sh
```

Once the script completes, it will not have to be run again unless

- The volume `$TARGET_ros_vol` is removed
- A full rebuild is done again
- The `update_ros.sh` script is run

The primary launch file can be run with the alias `auto_launch`.

## Development

When changes are made to the contents of the `ros` directory, you will need to update the volume.

First, make sure the container is running. Then, run:

```bash
sh update_ros.sh $TARGET
```

This will update the volume with the latest changes to the `ros` directory. This will not affect any other part of the container.

Make sure to run `source build.sh` once inside the container to build the changes.
