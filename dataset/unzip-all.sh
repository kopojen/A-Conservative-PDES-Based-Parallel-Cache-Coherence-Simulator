#!/bin/bash

mkdir -p data

for i in {0..3}; do
  unzip "MT${i}-canneal.zip" -d data
done
