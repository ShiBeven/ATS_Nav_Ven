#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target="${1:-}"

cd "${workspace_root}"

find_package_root() {
  local name="$1"
  rg --files -g 'package.xml' src 2>/dev/null \
    | while read -r pkgxml; do
        local dir
        dir="$(dirname "$pkgxml")"
        if [[ "$(basename "$dir")" == "$name" ]]; then
          printf '%s\n' "$dir"
        fi
      done \
    | head -n 1
}

print_workspace_brief() {
  local branch
  branch="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"

  echo "# Workspace Brief"
  echo
  echo "- Root: ${workspace_root}"
  echo "- Branch: ${branch}"
  echo "- Main packages: sp_vision25, pb2025_sentry_behavior, pb2025_sentry_nav, pb2025_sentry_bringup, pb_rm_interfaces"
  echo "- Workflow docs: AGENTS.md, docs/codex_workflow.md"
  echo
  echo "## Dirty Files"
  git status --short | sed -n '1,20p'
  echo
  echo "## Suggested Prompt Skeleton"
  cat <<'EOF'
目标:
范围:
约束:
验证:
现象:
EOF
}

print_package_brief() {
  local package_name="$1"
  local package_root
  package_root="$(find_package_root "$package_name")"

  if [[ -z "${package_root}" ]]; then
    echo "Package not found: ${package_name}" >&2
    exit 1
  fi

  local readme=""
  if [[ -f "${package_root}/README.md" ]]; then
    readme="${package_root}/README.md"
  elif [[ -f "${package_root}/readme.md" ]]; then
    readme="${package_root}/readme.md"
  fi

  echo "# Package Brief: ${package_name}"
  echo
  echo "- Root: ${package_root}"
  if [[ -n "${readme}" ]]; then
    echo "- README: ${readme}"
  fi
  echo "- Build: colcon build --packages-select ${package_name} --cmake-args -DCMAKE_BUILD_TYPE=Release"
  echo "- Search roots:"
  for subdir in include src launch configs params tests behavior_trees plugins tools io tasks calibration docs; do
    if [[ -d "${package_root}/${subdir}" ]]; then
      echo "  - ${package_root}/${subdir}"
    fi
  done
  echo
  echo "## Dirty Files In Scope"
  git status --short -- "${package_root}" | sed -n '1,20p'
  echo
  echo "## Key Files"
  for subdir in include src launch configs params tests behavior_trees plugins tools io tasks calibration docs; do
    if [[ -d "${package_root}/${subdir}" ]]; then
      rg --files "${package_root}/${subdir}"
    fi
  done | sed -n '1,30p'
}

if [[ -z "${target}" ]]; then
  print_workspace_brief
else
  print_package_brief "${target}"
fi
