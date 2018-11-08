#!/bin/bash
git submodule update --init --recursive

export ROBOTREPO=$(git rev-parse --show-toplevel 2>/dev/null)

docker-compose --file docker-compose.yml run \
               --workdir="$(pwd)" --rm \
               --name="$(uuidgen)" \
               --entrypoint="$ROBOTREPO/docker-entrypoint.sh" \
               -e ROBOTREPO="$ROBOTREPO" \
               bash "$*"

