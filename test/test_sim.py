"""Unit tests for the OPC UA simulator."""
import pytest
from hlcs.sim import OpcuaSimulator, CounterLogic


def test_counter_logic_init():
    """Test CounterLogic initialization."""
    logic = CounterLogic()
    assert logic.get_counter() == 0


def test_counter_logic_increment():
    """Test CounterLogic increment."""
    logic = CounterLogic()
    assert logic.increment_counter() == 1
    assert logic.get_counter() == 1
    assert logic.increment_counter() == 2
    assert logic.get_counter() == 2


def test_counter_logic_reset():
    """Test CounterLogic reset."""
    logic = CounterLogic()
    logic.increment_counter()
    logic.increment_counter()
    logic.increment_counter()
    assert logic.get_counter() == 3
    logic.reset_counter()
    assert logic.get_counter() == 0


def test_counter_logic_multiple_operations():
    """Test multiple counter operations."""
    logic = CounterLogic()
    logic.increment_counter()  # 1
    logic.increment_counter()  # 2
    logic.reset_counter()      # 0
    logic.increment_counter()  # 1
    assert logic.get_counter() == 1


def test_opcua_simulator_init():
    """Test OpcuaSimulator initialization with default endpoint."""
    simulator = OpcuaSimulator()
    assert simulator.endpoint == "opc.tcp://0.0.0.0:4840/freeopcua/server/"
    assert simulator.server is None
    assert simulator.namespace_idx is None
    assert simulator.data_node is None
    assert simulator.counter_node is None
    assert isinstance(simulator.counter_logic, CounterLogic)


def test_opcua_simulator_init_custom_endpoint():
    """Test OpcuaSimulator initialization with custom endpoint."""
    custom_endpoint = "opc.tcp://localhost:4850/custom/server/"
    simulator = OpcuaSimulator(endpoint=custom_endpoint)
    assert simulator.endpoint == custom_endpoint


def test_opcua_simulator_with_custom_counter_logic():
    """Test OpcuaSimulator with custom counter logic."""
    custom_logic = CounterLogic()
    custom_logic.increment_counter()
    custom_logic.increment_counter()
    
    simulator = OpcuaSimulator(counter_logic=custom_logic)
    assert simulator.counter_logic.get_counter() == 2

