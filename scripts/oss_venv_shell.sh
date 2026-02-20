#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

. "${SCRIPT_DIR}/../oss-cad-suite/oss-cad-suite/environment"
. "${SCRIPT_DIR}/../.venv/bin/activate"

export OSS_CAD_SUITE_ACTIVE=1

ZSHRC_DIR="${SCRIPT_DIR}/../scripts/zshrc_oss_venv"
export ZDOTDIR="${ZSHRC_DIR}"

exec /usr/bin/zsh -i
