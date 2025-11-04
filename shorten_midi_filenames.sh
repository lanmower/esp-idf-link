#!/bin/bash

echo "Shortening MIDI filenames in data directory..."

# Find all MIDI files in the data directory and shorten their names
find data -name "*.mid" | while read file; do
  dir=$(dirname "$file")
  # Extract just the first few characters of the name to keep it short
  # Also maintain a counter to ensure uniqueness
  base=$(basename "$file" .mid | tr -d ' ' | cut -c1-8)
  
  # Ensure the new name will be unique by adding a counter if needed
  counter=1
  new_file="${dir}/${base}.mid"
  
  while [ -f "$new_file" ] && [ "$file" != "$new_file" ]; do
    new_file="${dir}/${base}_${counter}.mid"
    counter=$((counter + 1))
  done
  
  # Only rename if the file name has changed
  if [ "$file" != "$new_file" ]; then
    echo "Renaming: $file -> $new_file"
    mv "$file" "$new_file"
  fi
done

echo "All MIDI filenames shortened for SPIFFS compatibility." 