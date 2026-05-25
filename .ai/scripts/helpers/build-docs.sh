#!/usr/bin/env bash
# Build Documentation Script
# Usage: ./.ai/scripts/helpers/build-docs.sh [--clean]

clean=false
if [[ "${1:-}" == "--clean" ]]; then
  clean=true
fi

root_dir="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
cd "$root_dir"

echo "--- Building Documentation ---"

dist_dir="${root_dir}/dist"
mkdir -p "$dist_dir"

# Build LaTeX documents
tex_files=$(find docs -name "*.tex" 2>/dev/null)

if [[ -z "$tex_files" ]]; then
  echo "No .tex files found in docs/."
  exit 0
fi

echo "$tex_files" | while read -r file; do
  echo "Building $(basename "$file")..."
  out_dir="$(dirname "$file")/build"
  mkdir -p "$out_dir"

  latexmk -pdf -interaction=nonstopmode -shell-escape "-outdir=$out_dir" "$file"

  if [[ $? -eq 0 ]]; then
    pdf_name="$(basename "$file" .tex).pdf"
    if [[ -f "${out_dir}/${pdf_name}" ]]; then
      cp "${out_dir}/${pdf_name}" "${dist_dir}/"
      echo "Done: $pdf_name -> dist/"
    fi
  else
    echo "FAILED: $(basename "$file")"
  fi
done

if [[ "$clean" == true ]]; then
  echo "Cleaning build directories..."
  find docs -type d -name "build" -exec rm -rf {} + 2>/dev/null
fi

echo "--- Build Finished ---"
