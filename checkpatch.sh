#!/bin/bash

set -e

if [ -f "../../scripts/checkpatch.pl" ]; then
    check_patch="../../scripts/checkpatch.pl"
else
    if [ ! -f "checkpatch.pl" ]; then
        wget https://raw.githubusercontent.com/ev3dev/ev3-kernel/ev3dev-jessie/scripts/checkpatch.pl
        chmod +x checkpatch.pl
    fi
    check_patch=./checkpatch.pl
fi

if [ -z "$TRAVIS_REPO_SLUG" ]; then
    git diff origin/master | $check_patch --no-tree --no-signoff --ignore BAD_SIGN_OFF -
    echo "OK!"
else
    if [ "$TRAVIS_PULL_REQUEST" = "false" ] && [ "$TRAVIS_BRANCH" = "master" ]; then
        # it is too late to check now.
        exit 0
    fi
    curl -i "https://github.com/$TRAVIS_REPO_SLUG/compare/ev3dev:master...$TRAVIS_COMMIT.diff" \
        | $check_patch --no-tree --no-signoff -
fi

# fail if git command failed
exit $PIPESTATUS
