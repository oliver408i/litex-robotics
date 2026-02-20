#!/usr/bin/env zsh

if [[ -f "${HOME}/.zshrc" ]]; then
  source "${HOME}/.zshrc"
fi

venv_name=""
if [[ -n "${VIRTUAL_ENV}" ]]; then
  venv_name="${VIRTUAL_ENV:t}"
fi

prompt_prefix=""
if [[ -n "${venv_name}" ]] && [[ -n "${OSS_CAD_SUITE_ACTIVE}" ]]; then
  prompt_prefix="(${venv_name}+oss) "
elif [[ -n "${venv_name}" ]]; then
  prompt_prefix="(${venv_name}) "
elif [[ -n "${OSS_CAD_SUITE_ACTIVE}" ]]; then
  prompt_prefix="(oss) "
fi

if [[ -n "${prompt_prefix}" ]]; then
  PROMPT="${prompt_prefix}${PROMPT}"
fi
