# Chameleon Host Access

OpenCode runs inside a container. To run a command on the Chameleon host, use:

```bash
ssh -i /home/jovyan/.ssh/id_ed25519 -o BatchMode=yes -o IdentitiesOnly=yes -o UpdateHostKeys=no opencode@host.docker.internal '<command>'
```

The host user has passwordless sudo. For commands that require root access, use:

```bash
ssh -i /home/jovyan/.ssh/id_ed25519 -o BatchMode=yes -o IdentitiesOnly=yes -o UpdateHostKeys=no opencode@host.docker.internal 'sudo <command>'
```

The host key is pinned in `/home/jovyan/.ssh/known_hosts`. Do not disable host-key checking.
