#!/bin/bash

# Directory to search in
SEARCH_DIR="./src"

# Output file
OUT="merged_submission.cpp"

# Marker file that marks directories to be skipped
MARKER_FILE=".skip-merge"

# Start fresh
rm -f "$OUT"

# --- Collect directories to ignore (those containing MARKER_FILE) ---
mapfile -t IGNORE_DIRS < <(find "$SEARCH_DIR" -type f -name "$MARKER_FILE" -printf '%h\n')

should_skip() {
  local file="$1"
  for dir in "${IGNORE_DIRS[@]}"; do
    case "$file" in
      "$dir"/*) return 0 ;;  # yes, skip
    esac
  done
  return 1  # no, don't skip
}

# --- Header of merged file ---
{
  echo "// === Combined C++ Submission ==="
  echo "// Generated on $(date)"
  echo "// Source root: $SEARCH_DIR"
  echo
  echo "// === Header Files ==="
} >> "$OUT"


# --- Merge header files (.h, .hpp) ---
find "$SEARCH_DIR" -type f \( -name "*.h" -o -name "*.hpp" \) | sort | while IFS= read -r file; do
  if should_skip "$file"; then
    continue
  fi
  {
    echo
    echo
    echo "// ===== File: $file ====="
    cat "$file"
  } >> "$OUT"
done


# --- Merge source files (.cpp) ---
{
  echo
  echo
  echo "// === Source Files ==="
} >> "$OUT"

find "$SEARCH_DIR" -type f -name "*.cpp" | sort | while IFS= read -r file; do
  if should_skip "$file"; then
    continue
  fi
  {
    echo
    echo
    echo "// ===== File: $file ====="
    cat "$file"
  } >> "$OUT"
done


# --- End marker ---
{
  echo
  echo
  echo "// === End of Combined File ==="
} >> "$OUT"

echo "Done! Output written to $OUT"
