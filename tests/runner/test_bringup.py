"""How the sandbox reaches the robot's graph and the outside world, per backend."""

import pytest

from openrua.runner import bringup


def test_peers_profile_lists_every_peer():
    xml = bringup.peers_profile(["rc-x-sim", "10.0.0.7"])
    # the default multicast peer stays, so processes in one container meet
    assert xml.count("<locator>") == 3 and "<address>239.255.0.1</address>" in xml
    assert "<address>rc-x-sim</address>" in xml and "<address>10.0.0.7</address>" in xml


def test_simulated_robot_shares_the_internal_network():
    r = bringup.sandbox_reachability({"kind": "sim"}, "openrua-internal", "rc-1-sim",
                                     "proxy:http://openrua-proxy-ab12cd34:8888")
    assert r["network"] == "openrua-internal" and r["static_peer"] == "rc-1-sim"
    assert "<address>rc-1-sim</address>" in r["peers_xml"]
    assert r["internet"] == "proxy:http://openrua-proxy-ab12cd34:8888" and r["env"] == ()
    assert r["networks"] == ()       # the proxy is its only way out


def test_a_simulated_robots_open_internet_is_a_routed_network_of_its_own():
    # The internal network has no route out, so an open internet means
    # joining a routed one too; OpenRUA's own, not docker's default bridge,
    # where another container on the same ROS domain would see the
    # sandbox's topics.
    r = bringup.sandbox_reachability({"kind": "sim"}, "openrua-internal", "rc-1-sim", "open")
    assert r["network"] == "openrua-internal" and r["internet"] == "open"
    assert r["networks"] == (bringup.INTERNET_NETWORK,)
    assert bringup.INTERNET_NETWORK not in ("bridge", "openrua-internal")
    probe = bringup.sandbox_reachability({"kind": "sim"}, "openrua-internal", "rc-1-sim", "none")
    assert probe["internet"] == "none" and probe["networks"] == ()


def test_the_way_out_follows_the_proxy():
    assert bringup.internet_access("http://openrua-proxy-ab12cd34:8888") \
        == "proxy:http://openrua-proxy-ab12cd34:8888"
    assert bringup.internet_access("") == "open"


def test_real_robot_joins_the_host_network_and_reaches_the_proxy_by_address(monkeypatch):
    asked = []
    def by_address(name, network="bridge"):
        asked.append(name)
        return "http://172.17.0.9:8888"
    monkeypatch.setattr(bringup, "proxy_url_from_network", by_address)
    host = bringup.sandbox_reachability(
        {"kind": "real", "discovery": {"network": "host"}}, "openrua-internal", "r",
        "proxy:http://openrua-proxy-ab12cd34:8888")
    assert host["network"] == "host" and host["static_peer"] is None
    assert host["internet"] == "proxy:http://172.17.0.9:8888"
    assert asked == ["openrua-proxy-ab12cd34"]      # the proxy the URL names
    assert host["networks"] == ()

    # Without a proxy the host network is already the way out.
    open_ = bringup.sandbox_reachability(
        {"kind": "real", "discovery": {"network": "host"}}, "openrua-internal", "r", "open")
    assert open_["internet"] == "open" and open_["networks"] == ()

    peers = bringup.sandbox_reachability(
        {"kind": "real", "discovery": {"static_peers": ["192.168.1.20", "192.168.1.21"]}},
        "openrua-internal", "r", "open")
    assert peers["static_peer"] == "192.168.1.20,192.168.1.21"
    assert peers["peers_xml"].count("<locator>") == 3

    server = bringup.sandbox_reachability(
        {"kind": "real", "discovery": {"discovery_server": "192.168.1.20:11811"}},
        "openrua-internal", "r", "open")
    assert server["env"] == ("FASTDDS_BUILTIN_TRANSPORTS=UDPv4",
                             "ROS_DISCOVERY_SERVER=192.168.1.20:11811",
                             "ROS_SUPER_CLIENT=TRUE")
    assert server["static_peer"] is None


@pytest.mark.parametrize("distro", ["jazzy", "humble"])
def test_a_host_network_sandbox_speaks_udp_only(distro):
    # Containers on the host network share its addresses, so Fast DDS
    # would reach them through shared memory they do not share.
    r = bringup.sandbox_reachability(
        {"kind": "real", "ros_distro": distro, "discovery": {"network": "host"}},
        "openrua-internal", "r", "open")
    if distro == "humble":
        assert "<useBuiltinTransports>false</useBuiltinTransports>" in r["peers_xml"]
        assert r["env"] == ()
    else:
        assert r["peers_xml"] is None
        assert r["env"] == ("FASTDDS_BUILTIN_TRANSPORTS=UDPv4",)


def test_the_agent_is_given_the_proxy_url_its_sandbox_can_reach(monkeypatch):
    # A real robot's sandbox is on the host network, where the proxy's
    # container name does not resolve: the agent CLI must get the address.
    asked = []
    def by_address(name, network="bridge"):
        asked.append(name)
        return "http://172.17.0.9:8888"
    monkeypatch.setattr(bringup, "proxy_url_from_network", by_address)
    assert bringup.agent_proxy_url({"kind": "real"}, "http://openrua-proxy-ab12cd34:8888") \
        == "http://172.17.0.9:8888"
    assert asked == ["openrua-proxy-ab12cd34"]
    assert bringup.agent_proxy_url({"kind": "sim"}, "http://openrua-proxy:8888") \
        == "http://openrua-proxy:8888"
    assert bringup.agent_proxy_url({"kind": "real"}, "") == ""


def test_graph_probe_runs_inside_the_sandbox():
    argv = bringup.graph_probe("rc-1-sandbox")
    assert argv[:3] == ["docker", "exec", "rc-1-sandbox"]
    assert "ros2 node list --no-daemon --spin-time 2.0" in argv[-1]


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


def test_a_failed_verification_stops_the_container_it_started(monkeypatch):
    import pytest
    _docker(monkeypatch, [], [])
    calls = {"n": 0}
    real = bringup.domains_in_use

    def flaky(network):
        calls["n"] += 1
        if calls["n"] == 2:          # the read after start blows up
            raise RuntimeError("daemon hiccup")
        return real(network)

    monkeypatch.setattr(bringup, "domains_in_use", flaky)
    stops = []
    with pytest.raises(RuntimeError, match="hiccup"):
        bringup.claim_domain("net", None, lambda d: "me-sandbox", stops.append)
    assert stops == ["me-sandbox"]


def test_failed_driver_shutdown_still_cleans_sandbox_after_startup_error(tmp_path, monkeypatch):
    from types import SimpleNamespace
    import pytest
    from openrua.runner import bringup
    calls = []
    def fail_ready():
        raise TimeoutError("graph unavailable")
    def fail_shutdown():
        calls.append("driver")
        raise RuntimeError("driver cleanup failed")
    monkeypatch.setattr(bringup, "resolve_robot_files", lambda *a: None)
    monkeypatch.setattr(bringup.record, "write_config", lambda *a: tmp_path / 'config.yaml')
    monkeypatch.setattr(bringup, "sandbox_reachability", lambda *a: {"networks": ()})
    monkeypatch.setattr(bringup, "claim_domain", lambda *a: 0)
    monkeypatch.setattr(bringup.robot, "up", lambda *a, **k: SimpleNamespace(
        wait_ready=fail_ready, shutdown=fail_shutdown))
    monkeypatch.setattr(bringup, "sandbox_down", lambda name: calls.append(name))
    with pytest.raises(RuntimeError, match="driver cleanup failed"):
        bringup.bring_up({'machine': {'backend': {'kind': 'real'}}}, tmp_path,
                         'robot', 'sandbox', None, None, 'network', 'open', (), None,
                         tmp_path / 'robot.log')
    assert calls == ['driver', 'sandbox']


def test_an_open_internet_creates_and_joins_the_routed_network(tmp_path, monkeypatch):
    from types import SimpleNamespace
    created, joined = [], []
    monkeypatch.setattr(bringup, "resolve_robot_files", lambda *a: None)
    monkeypatch.setattr(bringup.record, "write_config", lambda *a: tmp_path / 'config.yaml')
    monkeypatch.setattr(bringup, "ensure_network",
                        lambda name, internal: created.append((name, internal)) or name)
    def claim(network, requested, start, stop):
        start(7)
        return 7
    monkeypatch.setattr(bringup, "claim_domain", claim)
    monkeypatch.setattr(bringup, "sandbox_up",
                        lambda cfg, ws, **kw: joined.append((kw["internet"], kw["networks"])))
    monkeypatch.setattr(bringup.robot, "up", lambda *a, **k: SimpleNamespace(
        wait_ready=lambda: None, shutdown=lambda: None))
    cfg = {'machine': {'backend': {'kind': 'sim', 'sandbox_image': 'img'}}}
    bringup.bring_up(cfg, tmp_path, 'rc-sim', 'rc-sandbox', None, None,
                     'openrua-internal', 'open', (), None, tmp_path / 'robot.log')
    assert created == [(bringup.INTERNET_NETWORK, False)]
    assert joined == [("open", (bringup.INTERNET_NETWORK,))]


def test_a_proxied_sandbox_joins_no_routed_network(tmp_path, monkeypatch):
    from types import SimpleNamespace
    created, joined = [], []
    monkeypatch.setattr(bringup, "resolve_robot_files", lambda *a: None)
    monkeypatch.setattr(bringup.record, "write_config", lambda *a: tmp_path / 'config.yaml')
    monkeypatch.setattr(bringup, "ensure_network",
                        lambda name, internal: created.append(name) or name)
    def claim(network, requested, start, stop):
        start(7)
        return 7
    monkeypatch.setattr(bringup, "claim_domain", claim)
    monkeypatch.setattr(bringup, "sandbox_up",
                        lambda cfg, ws, **kw: joined.append((kw["internet"], kw["networks"])))
    monkeypatch.setattr(bringup.robot, "up", lambda *a, **k: SimpleNamespace(
        wait_ready=lambda: None, shutdown=lambda: None))
    cfg = {'machine': {'backend': {'kind': 'sim', 'sandbox_image': 'img'}}}
    bringup.bring_up(cfg, tmp_path, 'rc-sim', 'rc-sandbox', None, None, 'openrua-internal',
                     'proxy:http://openrua-proxy-ab12cd34:8888', (), None, tmp_path / 'robot.log')
    assert created == []
    assert joined == [("proxy:http://openrua-proxy-ab12cd34:8888", ())]


def test_agent_environment_does_not_require_robot_controller_files(tmp_path, monkeypatch):
    from openrua.runner import bringup
    cfg = {'machine': {'backend': {'kind': 'real', 'sandbox_image': 'tools', 'ros_distro': 'jazzy'},
                       'controller_config': str(tmp_path / 'missing-controller.yaml')}}
    created = []
    monkeypatch.setattr(bringup, 'sandbox_up', lambda *a, **kw: created.append(kw['name']))
    def claim(network, domain, start, stop):
        start(7)
        return 7
    monkeypatch.setattr(bringup, 'claim_domain', claim)
    config, domain = bringup.prepare_sandbox(cfg, tmp_path, 'robot', 'agent', 'net', 'open', (), 7)
    assert created == ['agent'] and config.is_file() and domain == 7
    assert not (tmp_path / 'missing-controller.yaml').exists()
