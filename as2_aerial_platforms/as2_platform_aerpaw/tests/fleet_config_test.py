# Copyright 2024 Universidad Politecnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Stage 18: fleet configuration is data, not code (unittest; no pytest/ROS runtime).

A researcher changes the number of UAVs and platform config by editing
config/fleet.yaml - verified here.
"""

import os
import sys
import tempfile
import unittest

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, os.pardir, 'launch')))
import fleet_config  # noqa: E402

_FLEET = os.path.abspath(os.path.join(_HERE, os.pardir, 'config', 'fleet.yaml'))


class TestFleetConfig(unittest.TestCase):

    def test_shipped_fleet_resolves(self):
        fleet = fleet_config.load_fleet(_FLEET)
        self.assertGreaterEqual(len(fleet['vehicles']), 1)
        ns = [v['namespace'] for v in fleet['vehicles']]
        ports = [p for v in fleet['vehicles'] for p in (v['cmd_port'], v['tel_port'])]
        self.assertEqual(len(set(ns)), len(ns))
        self.assertEqual(len(set(ports)), len(ports))
        for v in fleet['vehicles']:
            self.assertTrue(v['id'] and v['conn'] and v['backend'])
        self.assertIn('base', fleet['frames'])
        self.assertIn('odom', fleet['frames'])

    def test_vehicle_count_is_config_only(self):
        fleet = fleet_config.load_fleet(_FLEET, num_drones=5)
        self.assertEqual(len(fleet['vehicles']), 5)
        self.assertEqual([v['namespace'] for v in fleet['vehicles']],
                         [f"drone{i}" for i in range(5)])
        ports = [p for v in fleet['vehicles'] for p in (v['cmd_port'], v['tel_port'])]
        self.assertEqual(len(set(ports)), len(ports))

    def test_per_vehicle_override(self):
        cfg = {
            'backend': 'physical',
            'vehicles': [
                {'id': 'uav-a', 'takeoff_altitude': 30.0},
                {'id': 'uav-b', 'namespace': 'custom-ns', 'conn': 'udpin://10.0.0.9:14550'},
            ],
        }
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as fh:
            yaml.safe_dump(cfg, fh)
            path = fh.name
        try:
            fleet = fleet_config.load_fleet(path)
            self.assertEqual(fleet['vehicles'][0]['params']['takeoff_altitude'], 30.0)
            self.assertEqual(fleet['vehicles'][0]['backend'], 'physical')
            self.assertEqual(fleet['vehicles'][1]['namespace'], 'custom-ns')
            self.assertEqual(fleet['vehicles'][1]['conn'], 'udpin://10.0.0.9:14550')
        finally:
            os.unlink(path)

    def test_duplicate_namespace_rejected(self):
        bad = {'vehicles': [{'id': 'a', 'namespace': 'drone0'},
                            {'id': 'b', 'namespace': 'drone0'}]}
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as fh:
            yaml.safe_dump(bad, fh)
            path = fh.name
        try:
            with self.assertRaises(ValueError):
                fleet_config.load_fleet(path)
        finally:
            os.unlink(path)


if __name__ == '__main__':
    unittest.main()
