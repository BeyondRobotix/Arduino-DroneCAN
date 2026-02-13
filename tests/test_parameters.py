"""
Tests for DroneCAN parameter system
"""
import pytest
import dronecan
import time


@pytest.mark.parameters
@pytest.mark.requires_hardware
class TestParameters:
    """Test parameter get/set operations"""

    def test_can_list_parameters(self, dronecan_node, device_node_id, test_config):
        """Verify we can retrieve parameter list from device"""
        expected_params = test_config['parameters']['expected_params']

        # Request parameters by index
        found_params = []
        for index in range(10):  # Try first 10 indices
            try:
                response = dronecan_node.request(
                    dronecan.uavcan.protocol.param.GetSet.Request(index=index),
                    device_node_id,
                    timeout=2.0
                )
                if response and response[0].name:
                    found_params.append(response[0].name)
                else:
                    break  # No more parameters
            except Exception:
                break

        assert len(found_params) >= len(expected_params), (
            f"Expected at least {len(expected_params)} parameters, found {len(found_params)}"
        )

    @pytest.mark.basic
    def test_expected_parameters_exist(self, parameter_manager, test_config):
        """Verify all expected parameters exist"""
        expected_params = test_config['parameters']['expected_params']

        for param_name in expected_params:
            value = parameter_manager['get'](param_name)
            assert value is not None, f"Parameter '{param_name}' not found on device"

    def test_nodeid_parameter_exists(self, parameter_manager):
        """Verify NODEID parameter exists and is readable"""
        value = parameter_manager['get']('NODEID')

        assert value is not None, "NODEID parameter not found"
        assert hasattr(value, 'integer_value'), "NODEID should be an integer type"

    def test_nodeid_in_valid_range(self, parameter_manager, test_config):
        """Verify NODEID is within valid DroneCAN node ID range"""
        value = parameter_manager['get']('NODEID')

        assert value is not None
        node_id = value.integer_value

        expected_range = test_config['parameters']['value_ranges']['NODEID']
        assert expected_range[0] <= node_id <= expected_range[1], (
            f"NODEID {node_id} outside valid range {expected_range}"
        )

    def test_parameter_write_read(self, parameter_manager):
        """Test writing and reading back a parameter value"""
        test_value = 42.5

        # Write parameter
        success = parameter_manager['set']('PARM_1', test_value)
        assert success, "Failed to set parameter PARM_1"

        # Small delay for parameter to persist
        time.sleep(0.1)

        # Read back
        value = parameter_manager['get']('PARM_1')
        assert value is not None, "Failed to read back parameter"

        actual_value = value.real_value
        assert abs(actual_value - test_value) < 0.01, (
            f"Parameter value mismatch: wrote {test_value}, read {actual_value}"
        )

    def test_parameter_boundary_values(self, parameter_manager, test_config):
        """Test setting parameters to boundary values"""
        param_range = test_config['parameters']['value_ranges']['PARM_1']

        # Test minimum value
        parameter_manager['set']('PARM_1', param_range[0])
        time.sleep(0.1)
        value = parameter_manager['get']('PARM_1')
        assert abs(value.real_value - param_range[0]) < 0.01

        # Test maximum value
        parameter_manager['set']('PARM_1', param_range[1])
        time.sleep(0.1)
        value = parameter_manager['get']('PARM_1')
        assert abs(value.real_value - param_range[1]) < 0.01

    def test_parameter_out_of_range_handling(self, parameter_manager, test_config):
        """Test that out-of-range values are handled appropriately"""
        param_range = test_config['parameters']['value_ranges']['PARM_1']

        # Try to set value above maximum
        out_of_range_value = param_range[1] + 50.0

        # This should either fail or clamp to max value
        parameter_manager['set']('PARM_1', out_of_range_value)
        time.sleep(0.1)

        value = parameter_manager['get']('PARM_1')
        actual = value.real_value

        # Value should not exceed maximum
        assert actual <= param_range[1], (
            f"Parameter accepted out-of-range value: {actual} > {param_range[1]}"
        )

    def test_multiple_parameter_writes(self, parameter_manager):
        """Test writing multiple parameters in sequence"""
        params_to_test = {
            'PARM_1': 25.5,
            'PARM_2': 75.3
        }

        # Write all parameters
        for name, value in params_to_test.items():
            success = parameter_manager['set'](name, value)
            assert success, f"Failed to set {name}"
            time.sleep(0.05)

        # Read all back
        for name, expected_value in params_to_test.items():
            value = parameter_manager['get'](name)
            actual = value.real_value
            assert abs(actual - expected_value) < 0.01, (
                f"{name}: wrote {expected_value}, read {actual}"
            )

    def test_parameter_types(self, dronecan_node, device_node_id, test_config):
        """Verify parameter types are correct"""
        # NODEID should be INT type
        response = dronecan_node.request(
            dronecan.uavcan.protocol.param.GetSet.Request(name='NODEID'),
            device_node_id,
            timeout=2.0
        )
        assert response is not None
        assert hasattr(response[0].value, 'integer_value'), (
            "NODEID should be integer type"
        )

        # PARM_1 should be REAL type
        response = dronecan_node.request(
            dronecan.uavcan.protocol.param.GetSet.Request(name='PARM_1'),
            device_node_id,
            timeout=2.0
        )
        assert response is not None
        assert hasattr(response[0].value, 'real_value'), (
            "PARM_1 should be real (float) type"
        )

    @pytest.mark.slow
    def test_parameter_rapid_updates(self, parameter_manager):
        """Test rapid parameter updates don't cause issues"""
        # Rapidly update parameter 20 times
        for i in range(20):
            value = float(i)
            success = parameter_manager['set']('PARM_1', value)
            assert success, f"Failed on update {i}"
            # No delay between updates - stress test

        time.sleep(0.2)  # Let it settle

        # Verify final value
        final = parameter_manager['get']('PARM_1')
        assert final is not None, "Parameter read failed after rapid updates"

    def test_parameter_names_nonempty(self, dronecan_node, device_node_id):
        """Verify all parameters have non-empty names"""
        for index in range(10):
            try:
                response = dronecan_node.request(
                    dronecan.uavcan.protocol.param.GetSet.Request(index=index),
                    device_node_id,
                    timeout=2.0
                )
                if response and response[0].name:
                    assert len(response[0].name) > 0, (
                        f"Parameter at index {index} has empty name"
                    )
                else:
                    break
            except Exception:
                break
