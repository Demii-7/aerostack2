"""AERPAW multi-UAV experiment package.

LAYER BOUNDARY (Stage 9). This package is **experiment logic**: the researcher's
algorithm, mission objectives, decision making, optimisation, wireless/RF metric
handling and multi-UAV coordination. It is kept strictly separate from **platform
integration** (`as2_platform_aerpaw`), which owns only AERPAW<->AeroStack2
communication, state/command translation and the platform abstraction.

Experiment code here talks to the vehicles ONLY through the AeroStack2 public API
(`as2_python_api` behaviours + `as2_msgs`/`sensor_msgs` topics). It never imports
the adapter internals (`ipc_bridge`, `aerpaw_platform`, aerpawlib) or the UDP/JSON
wire - see `interfaces.PlatformInterface` and `test/test_layering.py`.
"""
