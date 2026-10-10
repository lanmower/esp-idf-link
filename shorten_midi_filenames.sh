#!/bin/bash

echo "Shortening MIDI filenames in data directory..."

find data -name "*.mid" | while read file; do
  dir=$(dirname "$file")
  base=$(basename "$file" .mid | tr -d ' ' | cut -c1-8)
  
  counter=1
  new_file="${dir}/${base}.mid"
  
  while [ -f "$new_file" ] && [ "$file" != "$new_file" ]; do
    new_file="${dir}/${base}_${counter}.mid"
    counter=$((counter + 1))
  done
  
  if [ "$file" != "$new_file" ]; then
    echo "Renaming: $file -> $new_file"
    mv "$file" "$new_file"
  fi
done

echo "All MIDI filenames shortened for SPIFFS compatibility." 