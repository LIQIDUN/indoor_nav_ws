#!/usr/bin/env bash
exec "$(dirname -- "${BASH_SOURCE[0]}")/indoor_nav_preflight.sh" --status "$@"
