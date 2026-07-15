#!/bin/bash
set -eux

USERNAME=$1
USER_UID=$2
USER_GID=$3

# Create the group if it does not exist
if ! getent group "$USER_GID" >/dev/null; then groupadd --gid "$USER_GID" "$USERNAME"; fi

# Create the user if it does not exist
if ! id "$USERNAME" >/dev/null 2>&1; then useradd --uid "$USER_UID" --gid "$USER_GID" -m "$USERNAME"; fi
[ -d /etc/sudoers.d ] || mkdir /etc/sudoers.d
# Grant sudo privileges
echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/$USERNAME"
chmod 0440 "/etc/sudoers.d/$USERNAME"
    