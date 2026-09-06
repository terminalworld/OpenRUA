"""How the sandbox reaches the robot's graph and the proxy, per backend."""

from robocli.runner import bringup


def test_peers_profile_lists_every_peer():
    xml = bringup.peers_profile(["rc-x-sim", "10.0.0.7"])
    assert xml.count("<locator>") == 2
    assert "<address>rc-x-sim</address>" in xml and "<address>10.0.0.7</address>" in xml


def test_simulated_robot_shares_the_internal_network():
    r = bringup.sandbox_reachability({"kind": "sim"}, "robocli-internal", "rc-1-sim",
                                     "http://robocli-proxy:8888")
    assert r["network"] == "robocli-internal" and r["static_peer"] == "rc-1-sim"
    assert "<address>rc-1-sim</address>" in r["peers_xml"]
    assert r["internet"] == "proxy:http://robocli-proxy:8888" and r["env"] == ()


def test_real_robot_joins_the_host_network_and_reaches_the_proxy_by_address(monkeypatch):
    monkeypatch.setattr(bringup, "proxy_url_from_network",
                        lambda name, network="bridge": "http://172.17.0.9:8888")
    host = bringup.sandbox_reachability(
        {"kind": "real", "discovery": {"network": "host"}}, "robocli-internal", "r", "http://x")
    assert host["network"] == "host" and host["static_peer"] is None
    assert host["internet"] == "proxy:http://172.17.0.9:8888"

    peers = bringup.sandbox_reachability(
        {"kind": "real", "discovery": {"static_peers": ["192.168.1.20", "192.168.1.21"]}},
        "robocli-internal", "r", "http://x")
    assert peers["static_peer"] == "192.168.1.20,192.168.1.21"
    assert peers["peers_xml"].count("<locator>") == 2

    server = bringup.sandbox_reachability(
        {"kind": "real", "discovery": {"discovery_server": "192.168.1.20:11811"}},
        "robocli-internal", "r", "http://x")
    assert server["env"] == ("ROS_DISCOVERY_SERVER=192.168.1.20:11811",)
    assert server["static_peer"] is None


def test_graph_probe_runs_inside_the_sandbox():
    argv = bringup.graph_probe("rc-1-sandbox")
    assert argv[:3] == ["docker", "exec", "rc-1-sandbox"] and "ros2 node list" in argv[-1]
