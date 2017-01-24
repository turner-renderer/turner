#!/bin/sh
set -ex
apk --no-cache add py2-pip
pip install cpp-coveralls

coveralls \
    --root . \
    --build-root build \
    --exclude build/vendor \
    --exclude tests \
    --gcov "/usr/bin/llvm-cov gcov" \
    --gcov-options '\-lp'
