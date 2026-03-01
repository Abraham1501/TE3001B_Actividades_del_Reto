# TE3001B Actividades del Reto

This repository contains the activities for the TE3001B challenge, along with a submodule referencing the official course material.

## Submodule: TE3001B_Intelligent_Robotics_Implementation_2026

The folder `TE3001B_Intelligent_Robotics_Implementation_2026` is a git submodule pointing to:
[https://github.com/ManchesterRoboticsLtd/TE3001B_Intelligent_Robotics_Implementation_2026](https://github.com/ManchesterRoboticsLtd/TE3001B_Intelligent_Robotics_Implementation_2026)

### First-time setup

After cloning this repository, initialize and pull the submodule:

```bash
git submodule update --init --recursive
```

### Updating the submodule to the latest version

Since the course material is updated regularly, follow these steps to pull the latest changes:

1. Enter the submodule directory and fetch + checkout the desired commit:

```bash
cd TE3001B_Intelligent_Robotics_Implementation_2026
git fetch origin
git checkout <commit-hash>   # or a branch, e.g. "main"
cd ..
```

2. Stage the updated submodule reference in this repo:

```bash
git add TE3001B_Intelligent_Robotics_Implementation_2026
```

3. Commit the change:

```bash
git commit -m "Update submodule to latest version"
```

4. Push:

```bash
git push
```

> The parent repository only stores a pointer (commit hash) to the submodule. Every time the course material is updated, repeat the steps above to advance that pointer to the new commit.
