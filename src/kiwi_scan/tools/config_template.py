# SPDX-FileCopyrightText: 2026 Helmholtz-Zentrum Berlin für Materialien und Energie GmbH
# SPDX-License-Identifier: MIT

"""Create complete scan-configuration templates from the data model."""

from dataclasses import asdict
from typing import Any, Dict

import yaml

from kiwi_scan.datamodels import (
    ActuatorConfig,
    JogConfig,
    PluginConfig,
    ScanConfig,
    ScanDimension,
    ScanTriggers,
    SubscriptionConfig,
    TriggerAction,
)


def default_scan_config() -> ScanConfig:
    """Return a scan configuration containing the model defaults."""
    return ScanConfig(actuators={}, detector_pvs=[])


def scan_config_to_dict(config: ScanConfig) -> Dict[str, Any]:
    """Convert a scan config to the preferred public YAML structure."""
    data = asdict(config)

    # MonitorConfig is an internal wrapper. YAML stores its free-form
    # parameters directly below ``monitor``.
    data["monitor"] = dict(config.monitor.parameters)
    return data


def _examples() -> Dict[str, Any]:
    """Build examples with illustrative required values and model defaults."""
    actuator = ActuatorConfig(pv="EXAMPLE:MOTOR", jog=JogConfig())
    triggers = ScanTriggers(before=[TriggerAction(pv="EXAMPLE:TRIGGER", value=1)])
    return {
        "actuators": {"actuators": {"axis": asdict(actuator)}},
        "detector_pvs": {"detector_pvs": ["EXAMPLE:DETECTOR"]},
        "scan_dimensions": {
            "scan_dimensions": [asdict(ScanDimension("axis", 0.0, 1.0, 5))],
        },
        "plugin_configs": {
            "plugin_configs": [asdict(PluginConfig(type="LoggingPlugin"))],
        },
        "monitor": {
            "monitor_type": "print",
            "monitor": {},
        },
        "triggers": {"triggers": asdict(triggers)},
        "metadata_pvs": {"metadata_pvs": ["EXAMPLE:METADATA"]},
        "metadata_constants": {"metadata_constants": {"example_key": "example_value"}},
        "subscriptions": {
            "subscriptions": [
                asdict(
                    SubscriptionConfig(
                        name="example_sync",
                        role="sync",
                        pv="EXAMPLE:SYNC",
                    )
                )
            ],
        },
    }


def default_scan_config_yaml() -> str:
    """Render active defaults with commented examples of nested config types.

    Examples use the same data classes as the loader, so newly added optional
    fields automatically appear with their defaults. Free-form plugin and
    monitor parameters are left empty because they have no common schema.
    """
    examples = _examples()
    blocks = [
        (
            "# Scan configuration: active values below are default values.\n"
            "# jog: null is the actuator default; its example expands JogConfig.\n"
            "# Plugin parameters and monitor settings are implementation-specific.\n"
            "# The monitor example does not list monitor specs jet. \n"
            "# parallel_scans/nested_scans use scan_dimensions but this design is obsolete.\n"
        )
    ]
    for key, value in scan_config_to_dict(default_scan_config()).items():
        block = yaml.safe_dump({key: value}, sort_keys=False)
        if key in examples:
            example = yaml.safe_dump(examples[key], sort_keys=False)
            block += f"# Example: {key}\n"
            block += "".join(f"# {line}\n" for line in example.splitlines())
        blocks.append(block)
    return "\n".join(blocks)
