#!/usr/bin/env bash

set -euo pipefail

readonly REPOSITORY="Thunderbird37/ConNect"
readonly RELEASE_EMAIL="202194566+Thunderbird37@users.noreply.github.com"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "${SCRIPT_DIR}"

fail() {
    printf 'Fehler: %s\n' "$*" >&2
    exit 1
}

for command in git gh pio c++; do
    command -v "${command}" >/dev/null 2>&1 || fail "${command} wurde nicht gefunden"
done

[[ "$(git branch --show-current)" == "main" ]] || fail "Release muss auf main erstellt werden"
[[ -z "$(git status --porcelain)" ]] || {
    git status --short
    fail "Release erfordert einen sauberen Arbeitsbaum"
}
[[ "$(git config user.email || true)" == "${RELEASE_EMAIL}" ]] ||
    fail "Repository-E-Mail muss ${RELEASE_EMAIL} sein"

gh auth status --hostname github.com >/dev/null 2>&1 || fail "GitHub CLI ist nicht angemeldet"
[[ "$(gh repo view "${REPOSITORY}" --json visibility --jq .visibility)" == "PUBLIC" ]] ||
    fail "Repository muss vor dem Release öffentlich sein, damit OTA ohne Token funktioniert"

git fetch origin main --tags
[[ "$(git rev-parse HEAD)" == "$(git rev-parse origin/main)" ]] ||
    fail "Lokales main und origin/main müssen identisch sein"

last_tag="$(git tag --list 'v[0-9]*.[0-9]*.[0-9]*' --sort=-version:refname | head -n 1)"
if [[ "${last_tag}" =~ ^v([0-9]+)\.([0-9]+)\.([0-9]+)$ ]]; then
    suggested_version="v${BASH_REMATCH[1]}.${BASH_REMATCH[2]}.$((BASH_REMATCH[3] + 1))"
else
    suggested_version="v1.0.0"
fi

version="${1:-}"
if [[ -z "${version}" ]]; then
    read -r -p "Version [${suggested_version}]: " version
    version="${version:-${suggested_version}}"
fi
[[ "${version}" =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]] ||
    fail "Version muss dem Format vMAJOR.MINOR.PATCH entsprechen"

if git rev-parse --verify --quiet "refs/tags/${version}" >/dev/null; then
    fail "Tag ${version} existiert bereits"
fi
if git ls-remote --exit-code --tags origin "refs/tags/${version}" >/dev/null 2>&1; then
    fail "Remote-Tag ${version} existiert bereits"
fi

tag_created=false
tag_pushed=false
cleanup() {
    exit_code=$?
    if [[ "${tag_created}" == true && "${tag_pushed}" == false ]]; then
        git tag --delete "${version}" >/dev/null 2>&1 || true
    fi
    exit "${exit_code}"
}
trap cleanup EXIT

git tag --annotate "${version}" --message "ConNect ${version}"
tag_created=true

c++ -std=c++17 -Wall -Wextra -Werror -Isrc \
    test/terminal_transport_test.cpp -o /tmp/connect-terminal-test
/tmp/connect-terminal-test
pio run -e esp32-s3

git push origin "refs/tags/${version}"
tag_pushed=true
trap - EXIT

printf '\nRelease %s wurde gestartet.\n' "${version}"
printf 'Workflow: https://github.com/%s/actions\n' "${REPOSITORY}"
printf 'Release:  https://github.com/%s/releases/tag/%s\n' "${REPOSITORY}" "${version}"