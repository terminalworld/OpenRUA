---
summary: Running OpenRUA under rootless Podman instead of Docker
read_when:
  - Your machine has no Docker daemon or you cannot join the docker group
  - openrua doctor reports rootless podman without --userns=keep-id
---

# Podman

OpenRUA drives its containers through the `docker` command line and
uses nothing beyond what Podman implements too: `run`, `exec`, `ps`,
`inspect`, `build`, user-defined networks with `--internal`, `--env-file`,
`--restart`. Rootless Podman is the usual way to get containers on a
shared machine where nobody will start a Docker daemon for you. The
code does not change; two things on the machine do.

## 1. Provide the `docker` command

Install Podman and its docker-compatible command:

```sh
sudo apt install podman podman-docker      # Debian / Ubuntu
sudo dnf install podman podman-docker      # Fedora / RHEL
```

Without root, a link in your own PATH does the same:

```sh
mkdir -p ~/.local/bin && ln -s "$(command -v podman)" ~/.local/bin/docker
```

`docker --version` should now print `podman version ...`. Rootless
Podman needs a subordinate uid range for your user (`grep $USER
/etc/subuid`); distributions add one when the user is created, and an
administrator can add one with `usermod --add-subuids
100000-165535 --add-subgids 100000-165535 $USER`.

## 2. Keep your uid in the sandbox

The agent runs in the sandbox as the user `robot`, built with your own
uid so that files it writes under `/workspace` are yours on the host.
Rootless Podman maps your uid to root inside a container by default, so
`robot` would land on a subordinate uid and could not write the
workspace. `--userns=keep-id` restores the mapping. Put it in your
defaults file once:

```yaml
# ~/.openrua/config.yaml
sandbox:
  run_args: ["--userns=keep-id"]
```

`sandbox.run_args` is appended verbatim to the sandbox's `docker run`
and belongs to the machine, not to a benchmark config. The robot's own
container runs as root and needs no flag: rootless root is you.
`openrua doctor` checks both points and prints the lines above when
they are missing; each trial's `provenance.json` records the engine
string and the flags the sandbox took.

## GPU rendering

`machine.backend.gpus: true` passes `--gpus all`, which Podman resolves
through the Container Device Interface. Generate the spec once on a
machine with the NVIDIA driver installed:

```sh
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
```

Without it Podman refuses the flag with "no known GPU vendor found in
CDI specs"; leave `gpus` unset for software rendering.

## What was verified

Podman 6.1 (netavark, aardvark-dns), rootless, Ubuntu 24.04: name
resolution inside an `--internal` network, no egress from it, stdin
piping with `run -i`, `ps --filter name=^x$`, `exec --env-file`,
`--restart unless-stopped`, and the ownership behaviour above. Older
Podman (before 4.0, CNI networking) has no DNS on internal networks and
is not supported.

## Not supported

Apptainer / Singularity: no long-running container with `exec`, no
isolated networks. The two-container design (robot plus sandbox on a
private network) does not map onto it.
