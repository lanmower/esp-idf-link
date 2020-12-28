#!/usr/bin/env bash
set -e

. $IDF_PATH/export.sh

cd {path}

idf.py build
