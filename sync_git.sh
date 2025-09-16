#!/bin/bash

# Navigate to the repository directory
cd /workspaces/isaac_ros-dev || exit

# Add all changes, including submodule updates
git add -A

# Commit only if there are staged changes to prevent errors
if ! git diff-index --quiet HEAD --; then
    git commit -m "Automated hourly backup: $(date)"
fi

# Push changes to the remote repository
git push origin main