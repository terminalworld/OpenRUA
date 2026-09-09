"""How the sandbox reaches the robot's graph and the proxy, per backend."""

from openrua.runner import bringup


def test_peers_profile_lists_every_peer():
    xml = bringup.peers_profile(["rc-x-sim", "10.0.0.7"])
    assert xml.count("<locator>") == 2
    assert "<address>rc-x-sim</address>" in xml and "<address>10.0.0.7</address>" in xml


def test_simulated_robot_shares_the_internal_network():
    r = bringup.sandbox_reachability({"kind": "sim"}, "openrua-internal", "rc-1-sim",
                                     "http://openrua-proxy:8888")
    assert r["network"] == "openrua-internal" and r["static_peer"] == "rc-1-sim"
    assert "<address>rc-1-sim</address>" in r["peers_xml"]
    assert r["internet"] == "proxy:http://openrua-proxy:8888" and r["env"] == ()


def test_real_robot_joins_the_host_network_and_reaches_the_proxy_by_address(monkeypatch):
    monkeypatch.setattr(bringup, "proxy_url_from_network",
                        lambda name, network="bridge": "http://172.17.0.9:8888")
    host = bringup.sandbox_reachability(
        {"kind": "real", "discovery": {"network": "host"}}, "openrua-internal", "r", "http://x")
    assert host["network"] == "host" and host["static_peer"] is None
    assert host["internet"] == "proxy:http://172.17.0.9:8888"

    peers = bringup.sandbox_reachability(
        {"kind": "real", "discovery": {"static_peers": ["192.168.1.20", "192.168.1.21"]}},
        "openrua-internal", "r", "http://x")
    assert peers["static_peer"] == "192.168.1.20,192.168.1.21"
    assert peers["peers_xml"].count("<locator>") == 2

    server = bringup.sandbox_reachability(
        {"kind": "real", "discovery": {"discovery_server": "192.168.1.20:11811"}},
        "openrua-internal", "r", "http://x")
    assert server["env"] == ("ROS_DISCOVERY_SERVER=192.168.1.20:11811",)
    assert server["static_peer"] is None


def test_graph_probe_runs_inside_the_sandbox():
    argv = bringup.graph_probe("rc-1-sandbox")
    assert argv[:3] == ["docker", "exec", "rc-1-sandbox"] and "ros2 node list" in argv[-1]


# ---- ROS domain: chosen by the bring-up, verified after the first container

def _docker(monkeypatch, network_names, inspect_infos, vanished: int = 0):
    """Stand in for the two docker calls domains_in_use makes. ``vanished``
    containers were listed by ps but are gone by inspect: docker then
    exits 1 and prints the rest."""
    import json
    import subprocess

    def fake_run(argv, **kw):
        if argv[:2] == ["docker", "ps"]:   # --filter network=<ours>
            return subprocess.CompletedProcess(argv, 0, "\n".join(network_names) + "\n", "")
        if argv[:2] == ["docker", "inspect"]:
            return subprocess.CompletedProcess(
                argv, 1 if vanished else 0, json.dumps(inspect_infos),
                "Error: No such object: gone\n" * vanished)
        raise AssertionError(argv)
    monkeypatch.setattr(bringup.subprocess, "run", fake_run)


def _info(name, started, domain=None):
    env = [f"ROS_DOMAIN_ID={domain}"] if domain is not None else []
    return {"Name": f"/{name}", "State": {"StartedAt": started}, "Config": {"Env": env}}


def test_domains_in_use_reads_the_containers_on_the_network(monkeypatch):
    _docker(monkeypatch, ["openrua-proxy", "a-sim", "a-sandbox", "b-sandbox"], [
        _info("openrua-proxy", "2026-09-09T01:00:00.000000000Z"),          # no domain: not a graph
        _info("a-sandbox", "2026-09-09T01:00:01.000000000Z", 0),
        _info("a-sim", "2026-09-09T01:00:02.000000000Z", 0),               # later: a-sandbox holds 0
        _info("b-sandbox", "2026-09-09T01:00:03.000000000Z", 2),
    ])
    used = bringup.domains_in_use("openrua-internal")
    assert used == {0: ("2026-09-09T01:00:01.000000000", "a-sandbox"),
                    2: ("2026-09-09T01:00:03.000000000", "b-sandbox")}
    assert bringup.free_domain(used) == 1


def test_an_empty_network_means_domain_zero(monkeypatch):
    _docker(monkeypatch, [], [])
    assert bringup.free_domain(bringup.domains_in_use("openrua-internal")) == 0


def test_every_domain_taken_is_an_error():
    import pytest
    from openrua.errors import UnavailableError
    with pytest.raises(UnavailableError, match="in use") as e:
        bringup.free_domain({d: ("t", "x") for d in bringup.DOMAINS})
    assert "--ros-domain" in e.value.hint


def test_a_requested_domain_is_taken_as_given():
    started = []
    d = bringup.claim_domain("net", 7, lambda dom: started.append(dom) or "me", lambda n: None)
    assert d == 7 and started == [7]


def test_claim_takes_the_lowest_free_domain_and_keeps_it(monkeypatch):
    world = {"a-sandbox": ("2026-09-09T01:00:00.000000000Z", 0)}
    _docker(monkeypatch, list(world), [_info(n, t, d) for n, (t, d) in world.items()])
    starts, stops = [], []

    def start(domain):
        starts.append(domain)
        world["me-sandbox"] = ("2026-09-09T01:00:05.000000000Z", domain)
        _docker(monkeypatch, list(world), [_info(n, t, d) for n, (t, d) in world.items()])
        return "me-sandbox"

    d = bringup.claim_domain("net", None, start, stops.append)
    assert d == 1 and starts == [1] and stops == []


def test_a_race_lost_is_retried_on_the_next_domain(monkeypatch):
    # Both bring-ups read "0 is free"; the rival starts first, so ours
    # backs off and takes 1. The rival's earlier StartedAt decides.
    world = {}
    _docker(monkeypatch, [], [])
    starts, stops = [], []

    def start(domain):
        starts.append(domain)
        if domain == 0:
            world["rival-sandbox"] = ("2026-09-09T01:00:04.000000000Z", 0)
        world["me-sandbox"] = ("2026-09-09T01:00:05.000000000Z", domain)
        _docker(monkeypatch, list(world), [_info(n, t, d) for n, (t, d) in world.items()])
        return "me-sandbox"

    def stop(name):
        stops.append(name)
        del world[name]

    d = bringup.claim_domain("net", None, start, stop)
    assert d == 1 and starts == [0, 1] and stops == ["me-sandbox"]


def test_a_race_won_keeps_the_domain(monkeypatch):
    # Same collision, but we started first: the rival is the one to move.
    world = {}
    _docker(monkeypatch, [], [])

    def start(domain):
        world["me-sandbox"] = ("2026-09-09T01:00:04.000000000Z", domain)
        world["rival-sandbox"] = ("2026-09-09T01:00:05.000000000Z", domain)
        _docker(monkeypatch, list(world), [_info(n, t, d) for n, (t, d) in world.items()])
        return "me-sandbox"

    assert bringup.claim_domain("net", None, start, lambda n: None) == 0


def test_started_at_orders_across_fraction_lengths():
    # podman writes as few fraction digits as it needs; docker nine.
    a, b = "2026-09-09T15:23:59.12Z", "2026-09-09T15:23:59.123Z"
    assert bringup._instant(a) < bringup._instant(b)
    assert bringup._instant("2026-09-09T15:23:59.9Z") > bringup._instant("2026-09-09T15:23:59.85Z")
    assert bringup._instant("2026-09-09T15:24:00+00:00") > bringup._instant("2026-09-09T15:23:59.999999999Z")


def test_a_container_gone_between_ps_and_inspect_is_simply_absent(monkeypatch):
    _docker(monkeypatch, ["a-sandbox", "gone-sandbox"],
            [_info("a-sandbox", "2026-09-09T01:00:01.000000000Z", 0)], vanished=1)
    assert bringup.domains_in_use("openrua-internal") == {
        0: ("2026-09-09T01:00:01.000000000", "a-sandbox")}
