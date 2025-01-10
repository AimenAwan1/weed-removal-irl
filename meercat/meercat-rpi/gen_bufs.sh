#!/bin/bash
protoc  --experimental_allow_proto3_optional \
        --python_out=app/src/proto/ \
        -Iproto/ \
        $(find proto -iname "*.proto")