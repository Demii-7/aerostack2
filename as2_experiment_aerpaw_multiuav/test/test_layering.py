"""Stage-9 layering boundary check (pure stdlib; runs under pytest, no ROS needed).

Enforces the separation between **experiment logic** (this package) and **platform
integration** (``as2_platform_aerpaw``):

* Experiment code must NOT import/drive the AERPAW adapter internals or the MAVLink
  side (``ipc_bridge``, ``aerpaw_platform``, ``aerpaw_as2_runner``, ``aerpawlib``) nor
  open the UDP IPC sockets. It talks to vehicles only through AeroStack2 APIs.
* The platform adapter must NOT import the experiment package (dependency direction
  is one-way: experiment -> AS2 -> adapter).
"""

import os
import re
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.abspath(os.path.join(_HERE, os.pardir))              # experiment pkg
_WS_ROOT = os.path.abspath(os.path.join(_PKG_ROOT, os.pardir))           # workspace root
_EXPERIMENT_DIRS = [
    os.path.join(_PKG_ROOT, "as2_experiment_aerpaw_multiuav"),
    os.path.join(_PKG_ROOT, "missions"),
]
_ADAPTER_DIR = os.path.join(_WS_ROOT, "as2_aerial_platforms", "as2_platform_aerpaw")

# Experiment code must not *import/use* these adapter internals (mentioning them in a
# docstring that explains the boundary is fine). Match import statements only.
_FORBIDDEN_IMPORTS = re.compile(
    r"^\s*(?:import|from)\s+(ipc_bridge|aerpaw_platform|aerpaw_as2_runner|aerpawlib"
    r"|as2_platform_aerpaw)\b", re.MULTILINE)
_UDP_PORT = re.compile(r"1576\d")  # IPC command/telemetry ports 15760..15769


def _iter_py(dirpath):
    for root, _dirs, files in os.walk(dirpath):
        if "build" in root or "__pycache__" in root or "egg-info" in root:
            continue
        for f in files:
            if f.endswith(".py") and f != os.path.basename(__file__):
                yield os.path.join(root, f)


def _strip_docstrings_and_comments(text: str) -> str:
    """Drop triple-quoted blocks and # comments so boundary mentions don't trip checks."""
    no_doc = re.sub(r'""".*?"""', '', text, flags=re.DOTALL)
    no_doc = re.sub(r"'''.*?'''", '', no_doc, flags=re.DOTALL)
    no_comment = re.sub(r"#.*", '', no_doc)
    return no_comment


class TestLayering(unittest.TestCase):
    def setUp(self):
        if not os.path.isdir(_ADAPTER_DIR):
            self.skipTest("adapter package not present in this checkout")

    def test_experiment_does_not_reach_into_adapter(self):
        for d in _EXPERIMENT_DIRS:
            for path in _iter_py(d):
                with open(path, encoding="utf-8") as fh:
                    code = _strip_docstrings_and_comments(fh.read())
                m = _FORBIDDEN_IMPORTS.search(code)
                self.assertIsNone(
                    m, f"{path}: experiment layer must not import adapter internal "
                       f"'{m.group(1) if m else ''}'")
                self.assertIsNone(
                    _UDP_PORT.search(code),
                    f"{path}: no raw UDP IPC ports in experiment logic")

    def test_adapter_does_not_import_experiment(self):
        for path in _iter_py(_ADAPTER_DIR):
            with open(path, encoding="utf-8") as fh:
                code = _strip_docstrings_and_comments(fh.read())
            # Only a real import statement counts (docstrings referencing the mission are fine).
            self.assertIsNone(
                re.search(r"(?:import|from)\s+as2_experiment_aerpaw_multiuav", code),
                f"{path}: adapter must not import the experiment package")

    def test_experiment_is_environment_agnostic(self):
        """Stage 13: the SAME experiment must run on Digital Twin and Physical Testbed.

        Therefore experiment logic must NOT branch on the execution environment. Assert
        no code reference to a backend/environment identifier anywhere in the
        experiment package or missions.
        """
        env_ref = re.compile(
            r"\b(platform_backend|digital_twin|DIGITAL_TWIN|is_real_hardware|"
            r"is_aerpaw_environment|aerpaw_environment|physical_testbed)\b")
        for d in _EXPERIMENT_DIRS:
            for path in _iter_py(d):
                with open(path, encoding="utf-8") as fh:
                    code = _strip_docstrings_and_comments(fh.read())
                self.assertIsNone(
                    env_ref.search(code),
                    f"{path}: experiment logic must not branch on execution environment")

    # --- Stage 14: keep ROS 2 confined to the robotics layer ---------------------
    _ROS_TOKENS = re.compile(
        r"\b(ros2|rclpy|launch_ros|geometry_msgs|std_msgs|nav_msgs|sensor_msgs|"
        r"as2_platform_aerpaw|as2_stack)\b")

    def test_aerpaw_side_runner_is_ros_free(self):
        """The AERPAW-side runner must not carry ROS 2 implementation details.

        Its only coupling to the robotics layer is the ROS-free UDP/JSON integration
        interface. (Docstrings/comments describing the boundary are allowed.)
        """
        runner = os.path.join(_ADAPTER_DIR, "aerpawlib_runner", "aerpaw_as2_runner.py")
        if not os.path.isfile(runner):
            self.skipTest("runner not present")
        with open(runner, encoding="utf-8") as fh:
            code = _strip_docstrings_and_comments(fh.read())
        m = self._ROS_TOKENS.search(code)
        self.assertIsNone(m, f"{runner}: AERPAW-side runner must be ROS-free (found {m.group(0) if m else ''})")

    def test_experiment_core_is_ros_free(self):
        """The pure experiment core (policies/loop/state) must not import ROS at all,
        so it is testable and portable without a ROS runtime.

        ROS use is allowed ONLY inside the boundary implementations
        (interfaces.DroneInterfacePlatform, rf_metrics.TopicMeasurementSource) and only
        as lazy function-level imports, never at module top level.
        """
        exp = os.path.join(_PKG_ROOT, "as2_experiment_aerpaw_multiuav", "experiment")
        pure = ("closed_loop.py", "coordination.py", "decision.py", "fleet.py", "policies.py")
        boundary = ("interfaces.py", "rf_metrics.py")
        for name in pure:
            path = os.path.join(exp, name)
            with open(path, encoding="utf-8") as fh:
                code = _strip_docstrings_and_comments(fh.read())
            self.assertIsNone(
                re.search(r"^\s*(import|from)\s+(rclpy|ros2|geometry_msgs|std_msgs|"
                          r"nav_msgs|sensor_msgs|as2_)", code, re.MULTILINE),
                f"{path}: pure experiment core must have no ROS/as2 imports")
        # boundary impls: any ROS import must be function-scoped (indented), never col-0.
        for name in boundary:
            path = os.path.join(exp, name)
            with open(path, encoding="utf-8") as fh:
                lines = _strip_docstrings_and_comments(fh.read()).splitlines()
            for ln in lines:
                if re.match(r"(import|from)\s+(rclpy|geometry_msgs|std_msgs|nav_msgs|sensor_msgs|as2_)", ln):
                    self.fail(f"{path}: ROS/as2 import must be lazy (inside a function), not top-level: {ln!r}")

    def test_protocol_version_consistent(self):
        """IF-1 wire protocol version must match between C++ adapter and AERPAW runner."""
        runner = os.path.join(_ADAPTER_DIR, "aerpawlib_runner", "aerpaw_as2_runner.py")
        hdr = os.path.join(_ADAPTER_DIR, "include", "as2_platform_aerpaw", "ipc_bridge.hpp")
        if not (os.path.isfile(runner) and os.path.isfile(hdr)):
            self.skipTest("adapter files not present")
        with open(runner, encoding="utf-8") as fh:
            py = re.search(r"^PROTOCOL_VERSION\s*=\s*(\d+)", fh.read(), re.MULTILINE)
        with open(hdr, encoding="utf-8") as fh:
            cpp = re.search(r"kProtocolVersion\s*=\s*(\d+)", fh.read())
        self.assertIsNotNone(py, "runner PROTOCOL_VERSION not found")
        self.assertIsNotNone(cpp, "ipc_bridge kProtocolVersion not found")
        self.assertEqual(py.group(1), cpp.group(1),
                         f"protocol version drift: python={py.group(1)} cpp={cpp.group(1)}")


if __name__ == "__main__":
    unittest.main()
