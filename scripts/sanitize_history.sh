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
# ⚠️  DESTRUCTIVE — rewrites git history. Run only after explicit approval.
#
# Replaces personal email addresses in the commit history with an institutional
# identity, so that publishing the repository does not expose contributor PII.
#
# Usage:
#   scripts/sanitize_history.sh --dry-run          # show what would change
#   scripts/sanitize_history.sh --apply            # rewrite history (irreversible)
#
# Configure the mapping below before running.

set -euo pipefail

# ---------------------------------------------------------------- configuration
# Personal addresses to replace, and the institutional identity to replace them
# with. Add one NEW_* pair per author if separate credit must be preserved.
NEW_NAME="Youngbo Shim"
NEW_EMAIL="youngbo.shim@keti.re.kr"

# ---------------------------------------------------------------- preconditions
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MODE="${1:---dry-run}"

if ! git rev-parse --git-dir >/dev/null 2>&1; then
  echo "error: not a git repository" >&2
  exit 1
fi

if [[ -n "$(git status --porcelain)" ]]; then
  echo "error: working tree is dirty; commit or stash first" >&2
  exit 1
fi

echo "Current authors in history:"
git log --format='  %an <%ae>' | sort -u

if [[ "${MODE}" == "--dry-run" ]]; then
  cat <<EOF

Dry run only. Nothing was changed.

Would rewrite every commit to:
  ${NEW_NAME} <${NEW_EMAIL}>

To apply:
  1. Back up the repository first:
       git bundle create ../\$(basename "\$PWD")-backup.bundle --all
  2. Re-run with --apply
  3. Verify:  git log --format='%an <%ae>' | sort -u
  4. Force-push only to the PUBLIC remote, never to the internal GitLab remote.
EOF
  exit 0
fi

if [[ "${MODE}" != "--apply" ]]; then
  echo "error: expected --dry-run or --apply" >&2
  exit 1
fi

# ---------------------------------------------------------------- backup
BACKUP="../$(basename "$PWD")-backup-$(git rev-parse --short HEAD).bundle"
git bundle create "${BACKUP}" --all
echo "backup written to ${BACKUP}"

# ---------------------------------------------------------------- rewrite
if command -v git-filter-repo >/dev/null 2>&1; then
  MAILMAP="$(mktemp)"
  # Any author is folded into the institutional identity.
  git log --format='%an <%ae>' | sort -u | while read -r who; do
    printf '%s <%s> %s\n' "${NEW_NAME}" "${NEW_EMAIL}" "${who}"
  done > "${MAILMAP}"
  git filter-repo --force --mailmap "${MAILMAP}"
  rm -f "${MAILMAP}"
else
  echo "git-filter-repo not found, falling back to filter-branch" >&2
  FILTER_BRANCH_SQUELCH_WARNING=1 git filter-branch --force --env-filter "
    export GIT_AUTHOR_NAME='${NEW_NAME}'
    export GIT_AUTHOR_EMAIL='${NEW_EMAIL}'
    export GIT_COMMITTER_NAME='${NEW_NAME}'
    export GIT_COMMITTER_EMAIL='${NEW_EMAIL}'
  " --tag-name-filter cat -- --branches --tags
fi

echo
echo "Rewritten. Authors now:"
git log --format='  %an <%ae>' | sort -u
echo
echo "filter-repo removes remotes by design. Re-add them explicitly:"
echo "  git remote add origin git@github.com:<org>/<repo>.git"
