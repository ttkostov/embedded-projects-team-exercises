#!/bin/bash

# Directories to search in (add more if needed)
SEARCH_DIRS=(
  "./src"
  "./include"
)

# Output file
OUT="merged_submission.cpp"

# Marker file that marks directories to be skipped
MARKER_FILE=".skip-merge"

# Remove old output
rm -f "$OUT"

# --- Collect ignore directories from all search roots ---
IGNORE_DIRS=()
for ROOT in "${SEARCH_DIRS[@]}"; do
  while IFS= read -r dir; do
    IGNORE_DIRS+=("$dir")
  done < <(find "$ROOT" -type f -name "$MARKER_FILE" -printf '%h\n')
done

# --- Function: should this file be skipped? ---
should_skip() {
  local file="$1"
  for dir in "${IGNORE_DIRS[@]}"; do
    case "$file" in
      "$dir"/*) return 0 ;;  # skip file
    esac
  done
  return 1
}

# --- Begin output file ---
{
  echo "// === Combined C++ Submission ==="
  echo "// Generated on $(date)"
  echo "// Source roots: ${SEARCH_DIRS[*]}"
  echo
  echo "// === Header Files ==="
} >> "$OUT"


# --- Merge header files (.h, .hpp) from all roots ---
for ROOT in "${SEARCH_DIRS[@]}"; do
  find "$ROOT" -type f \( -name "*.h" -o -name "*.hpp" \) | sort | while IFS= read -r file; do
    if should_skip "$file"; then
      continue
    fi
    {
      echo
      echo "// ===== File: $file ====="
      cat "$file"
    } >> "$OUT"
  done
done


# --- Merge source files (.cpp) ---
{
  echo
  echo
  echo "// === Source Files ==="
} >> "$OUT"

for ROOT in "${SEARCH_DIRS[@]}"; do
  find "$ROOT" -type f -name "*.cpp" | sort | while IFS= read -r file; do
    if should_skip "$file"; then
      continue
    fi
    {
      echo
      echo "// ===== File: $file ====="
      cat "$file"
    } >> "$OUT"
  done
done


# --- End marker ---
{
  echo
  echo "// === End of Combined File ==="
} >> "$OUT"

echo "Done! Output generated in $OUT"
