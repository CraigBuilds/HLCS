"""Unit tests for the OPC UA simulator."""
from hlcs.sim import OpcuaSimulator


def test_opcua_simulator_init():
    """Test OpcuaSimulator initialization with default endpoint."""
    simulator = OpcuaSimulator()
    assert simulator.endpoint == "opc.tcp://0.0.0.0:4840/freeopcua/server/"
    assert simulator.server is None
    assert simulator.namespace_idx is None
    assert simulator.data_node is None
    assert simulator.counter_node is None


def test_opcua_simulator_init_custom_endpoint():
    """Test OpcuaSimulator initialization with custom endpoint."""
    custom_endpoint = "opc.tcp://localhost:4850/custom/server/"
    simulator = OpcuaSimulator(endpoint=custom_endpoint)
    assert simulator.endpoint == custom_endpoint
