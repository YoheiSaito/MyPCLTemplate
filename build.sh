#!/bin/sh
cmake source && cp compile_commands.json source && make -j8
