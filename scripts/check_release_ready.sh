#!/usr/bin/env bash
# Copyright 2026 Korea Electronics Technology Institute (KETI)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Public-release gate for kadif-b5 open-source repositories.
# Fails (exit 1) if any blocking issue remains. Run from the repository root.

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}" || exit 1

RED=$'\033[0;31m'; GRN=$'\033[0;32m'; YLW=$'\033[0;33m'; BLD=$'\033[1m'; RST=$'\033[0m'
FAILED=0
WARNED=0

# Source files subject to the license-header rule.
SRC_GLOB=(-name '*.h' -o -name '*.cc' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.py' -o -name '*.sh')

srcfiles() {
  find . -type f \( "${SRC_GLOB[@]}" \) \
    -not -path './.git/*' -not -path './build/*' -not -path './devel/*' \
    -not -path './install/*' -not -path './log/*'
}

# Text files scanned for leaked internal information.
scanfiles() {
  find . -type f \( "${SRC_GLOB[@]}" -o -name '*.md' -o -name '*.xml' -o -name '*.yaml' \
    -o -name '*.yml' -o -name '*.launch' -o -name '*.txt' -o -name '*.json' \) \
    -not -path './.git/*' -not -path './build/*' -not -path './devel/*' \
    -not -path './install/*' -not -path './log/*' -not -name 'LICENSE'
}

pass() { printf '  %s[ OK ]%s %s\n' "${GRN}" "${RST}" "$1"; }
fail() { printf '  %s[FAIL]%s %s\n' "${RED}" "${RST}" "$1"; FAILED=$((FAILED + 1)); }
warn() { printf '  %s[WARN]%s %s\n' "${YLW}" "${RST}" "$1"; WARNED=$((WARNED + 1)); }
sect() { printf '\n%s%s%s\n' "${BLD}" "$1" "${RST}"; }

# Reports a blocking violation when the pattern is found in scanned files.
# $1 = human label, $2 = extended regex, $3 = "warn" to downgrade to a warning
forbid() {
  local label="$1" pattern="$2" level="${3:-fail}" hits
  hits="$(scanfiles -print0 | xargs -0 grep -InE "${pattern}" 2>/dev/null)"
  if [[ -n "${hits}" ]]; then
    if [[ "${level}" == "warn" ]]; then warn "${label}"; else fail "${label}"; fi
    printf '%s\n' "${hits}" | head -n 8 | sed 's/^/         /'
    local n; n="$(printf '%s\n' "${hits}" | wc -l)"
    [[ "${n}" -gt 8 ]] && printf '         ... and %d more\n' "$((n - 8))"
  else
    pass "${label}"
  fi
}

printf '%skadif-b5 public-release gate%s  (%s)\n' "${BLD}" "${RST}" "${REPO_ROOT}"

# ---------------------------------------------------------------- 1. required files
sect '1. Required repository files'
for f in LICENSE NOTICE README.md CONTRIBUTING.md SECURITY.md .gitignore; do
  if [[ -f "${f}" ]]; then pass "${f} exists"; else fail "${f} is missing"; fi
done
if grep -q 'Apache License' LICENSE 2>/dev/null; then
  pass 'LICENSE is Apache-2.0'
else
  fail 'LICENSE does not contain the Apache License text'
fi

# ---------------------------------------------------------------- 2. license headers
sect '2. Apache-2.0 license headers'
missing_hdr=()
while IFS= read -r f; do
  head -n 20 "${f}" | grep -q 'Apache License, Version 2.0' || missing_hdr+=("${f}")
done < <(srcfiles)
if [[ ${#missing_hdr[@]} -eq 0 ]]; then
  pass "all $(srcfiles | wc -l | tr -d ' ') source files carry the header"
else
  fail "${#missing_hdr[@]} source file(s) missing the Apache-2.0 header"
  printf '         %s\n' "${missing_hdr[@]}" | head -n 12
fi

# ---------------------------------------------------------------- 3. package metadata
sect '3. Package metadata'
while IFS= read -r pkg; do
  lic="$(sed -n 's:.*<license>\(.*\)</license>.*:\1:p' "${pkg}" | head -n 1)"
  mnt="$(sed -n 's:.*<maintainer[^>]*>\(.*\)</maintainer>.*:\1:p' "${pkg}" | head -n 1)"
  [[ "${lic}" == "Apache-2.0" ]] \
    && pass "${pkg}: <license>Apache-2.0</license>" \
    || fail "${pkg}: <license> is '${lic}' (must be Apache-2.0)"
  if grep -qE 'todo\.todo|TODO' "${pkg}"; then
    fail "${pkg}: TODO placeholder still present (maintainer='${mnt}')"
  else
    pass "${pkg}: no TODO placeholders"
  fi
done < <(find . -name package.xml -not -path './.git/*')

# ---------------------------------------------------------------- 4. leaked internals
sect '4. Internal information leakage'
forbid 'no hardcoded private IPv4 addresses' \
  '(^|[^0-9.])(192\.168\.[0-9]{1,3}\.[0-9]{1,3}|10\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}|172\.(1[6-9]|2[0-9]|3[01])\.[0-9]{1,3}\.[0-9]{1,3})'
forbid 'no internal GitLab URLs' 'gitlab\.com/(keti-mobility|keti_autonomous)'
forbid 'no absolute user home paths' '/home/[a-z][a-z0-9_-]+/'
forbid 'no credentials or tokens' '(password|passwd|secret|api[_-]?key|access[_-]?token|private[_-]?key)[[:space:]]*[:=]'
forbid 'no personal email addresses' '[A-Za-z0-9._%+-]+@(gmail|naver|daum|hanmail|nate|todo)\.'

# ---------------------------------------------------------------- 5. license conflicts
sect '5. License compatibility (Apache-2.0 forbids linking GPL)'
forbid 'no CGAL dependency (CGAL is GPLv3+)' '(find_package\([[:space:]]*CGAL|CGAL::|<CGAL/)'
forbid 'no other copyleft-licensed dependency' \
  '(GPL-3\.0|GPL-2\.0|AGPL|GNU General Public License)' warn

# ---------------------------------------------------------------- 6. formatting
sect '6. Formatting'
if command -v clang-format >/dev/null 2>&1; then
  cpp_files="$(find . \( -name '*.h' -o -name '*.cc' \) -not -path './.git/*' -not -path './build/*')"
  if [[ -z "${cpp_files}" ]]; then
    pass 'no C++ sources to format'
  elif [[ ! -f .clang-format ]]; then
    fail '.clang-format is missing but C++ sources exist'
  elif printf '%s\n' "${cpp_files}" | xargs clang-format --dry-run --Werror >/dev/null 2>&1; then
    pass 'clang-format --dry-run --Werror is clean'
  else
    fail 'clang-format reports formatting differences'
  fi
else
  warn 'clang-format not installed, formatting not verified'
fi

trailing="$(scanfiles -print0 | xargs -0 grep -Iln '[[:space:]]$' 2>/dev/null)"
if [[ -z "${trailing}" ]]; then
  pass 'no trailing whitespace'
else
  fail 'trailing whitespace present'
  printf '         %s\n' ${trailing} | head -n 8
fi

# ---------------------------------------------------------------- 7. git hygiene
sect '7. Git hygiene'
if git rev-parse --git-dir >/dev/null 2>&1; then
  bad_authors="$(git log --format='%an <%ae>' | sort -u | grep -E '@(gmail|naver|daum|hanmail|nate)\.' || true)"
  if [[ -z "${bad_authors}" ]]; then
    pass 'all commit authors use institutional addresses'
  else
    fail 'commit history contains personal email addresses (history rewrite required)'
    printf '         %s\n' "${bad_authors}"
  fi
  remotes="$(git remote -v | grep -E 'gitlab\.com' || true)"
  if [[ -n "${remotes}" ]]; then
    warn "internal GitLab remote still configured (rename it to 'internal' before publishing)"
    printf '         %s\n' "${remotes}" | head -n 2
  else
    pass 'no internal GitLab remote configured as origin'
  fi
  big="$(git ls-files -z | xargs -0 -r du -k 2>/dev/null | awk '$1 > 5000 {print $1"KB\t"$2}')"
  if [[ -z "${big}" ]]; then
    pass 'no tracked file larger than 5 MB'
  else
    warn 'large tracked files present'
    printf '         %s\n' "${big}" | head -n 5
  fi
else
  warn 'not a git repository, git checks skipped'
fi

# ---------------------------------------------------------------- summary
sect 'Summary'
if [[ ${FAILED} -eq 0 ]]; then
  printf '  %sREADY%s to publish — %d blocking issue(s), %d warning(s)\n' \
    "${GRN}${BLD}" "${RST}" "${FAILED}" "${WARNED}"
  exit 0
else
  printf '  %sNOT READY%s — %d blocking issue(s), %d warning(s)\n' \
    "${RED}${BLD}" "${RST}" "${FAILED}" "${WARNED}"
  exit 1
fi
