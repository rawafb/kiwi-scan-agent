# SPDX-FileCopyrightText: 2026 Helmholtz-Zentrum Berlin fuer Materialien und Energie GmbH
# SPDX-License-Identifier: MIT

import re
from dataclasses import fields

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
from kiwi_scan.tools.config_template import default_scan_config_yaml


def test_default_yaml_contains_every_scan_config_field() -> None:
    data = yaml.safe_load(default_scan_config_yaml())

    assert list(data) == [field.name for field in fields(ScanConfig)]
    assert data["actuators"] == {}
    assert data["detector_pvs"] == []
    assert data["monitor"] == {}

    parsed = ScanConfig.from_dict(data)
    assert parsed == ScanConfig(actuators={}, detector_pvs=[])


def test_commented_examples_parse_and_cover_nested_fields() -> None:
    rendered = default_scan_config_yaml()
    data = yaml.safe_load(rendered)
    blocks = re.findall(r"# Example: [^\n]+\n((?:# .*\n)+)", rendered)
    assert len(blocks) == 9
    for block in blocks:
        example = "\n".join(line[2:] for line in block.splitlines())
        data.update(yaml.safe_load(example))

    nested = [
        (data["actuators"]["axis"], ActuatorConfig),
        (data["actuators"]["axis"]["jog"], JogConfig),
        (data["scan_dimensions"][0], ScanDimension),
        (data["plugin_configs"][0], PluginConfig),
        (data["triggers"], ScanTriggers),
        (data["triggers"]["before"][0], TriggerAction),
        (data["subscriptions"][0], SubscriptionConfig),
    ]
    for example, model in nested:
        assert set(example) == {item.name for item in fields(model)}

    parsed = ScanConfig.from_dict(data)
    parsed.validate()
    assert parsed.actuators["axis"].jog == JogConfig()
    assert parsed.subscriptions[0].pv == "EXAMPLE:SYNC"
    assert parsed.monitor_type == "print"


