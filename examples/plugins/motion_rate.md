# MotionRatePlugin

`MotionRatePlugin` is an example [async plugin](../../docs/async_plugins.md).
It estimates how fast one scan signal changes, for example the low resolution encoder
readback of the undulator gap during a continuous-motion scan, 
and writes the result as extra scan columns.

For each scan point the plugin takes the value of the row key `signal` and its
detector timestamp `TS-<signal>`, adds the pair to a sliding window of the last
`window` samples and fits a straight line in a background worker:

```text
value = slope * (time - t_first) + offset
```

`t_first` is the first timestamp in the window. It is subtracted before the
fit because POSIX timestamps (about 1.7e9 s) would otherwise cost most of the
float precision. `offset` is therefore the fitted value at the start of the
window, not at time zero.

Source: [`motion_rate.py`](motion_rate.py).

## Enable the plugin

The file is not part of the installed package. Add the directory to the
plugin path:

```bash
export KIWI_SCAN_PLUGIN_PATH=/path/to/kiwi-scan/examples/plugins
```

## YAML example

```yaml
detector_pvs:
  - ${IOC_MONO}:monoGetEnergy

plugin_configs:
  - type: MotionRatePlugin
    name: energy_rate
    parameters:
      signal: ${IOC_MONO}:monoGetEnergy
      window: 50
```

`signal` must be a key of the scan row that has a timestamp, normally a
detector PV. The detector timestamp is available to plugins without
`include_timestamps: true`; set that option only when the detector timestamps
should also be written to the scan file. The column of an earlier plugin also
works, because plugin values carry a timestamp too.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `signal` | required | Row key to fit, e.g. a detector PV name. The time axis is `TS-<signal>`. |
| `window` | `50` | Number of samples in the sliding window. Integer, `>= 2`. |
| `log_level`, `log_file` | base class | Standard plugin logging options. |

Invalid parameters raise `ValueError` when the scan is created, before any
worker thread starts.

## Columns

| Column | Meaning |
|---|---|
| `MotionRateStatus` | `pending` until the first fit is collected, then `ready`. `error` when the window cannot be fitted (all timestamps equal). |
| `MotionRateIndex` | Scan index of the newest sample in the fitted window; `-1` while pending. |
| `MotionRateSlope` | Fitted slope in signal units per second. `NaN` while pending or on error. |
| `MotionRateOffset` | Fitted value at the first timestamp of the window, in signal units. `NaN` while pending or on error. |

The four columns are present on every point. As for every async plugin, the
row shows the latest completed fit, so `MotionRateIndex` can be older than the
row's own index. If the previous fit is still running, the new window is not
submitted; the next point submits the then current window.

## Sample handling

- A sample is added only when both the value and `TS-<signal>` are finite
  numbers. Missing or invalid samples are skipped; the first one is logged
  once per scan as a warning.
- A sample with the same timestamp as the previous sample is skipped. The PV
  did not update since the last point, so the same measurement is not counted
  twice.
- A fit starts as soon as two samples are in the window. Until the window is
  full, the fit uses the samples collected so far.
- The window is cleared in `on_start()`.

## Performance

The scan thread only reads two row values, appends them to the window and
copies it (about 2 µs for `window: 50`). The fit runs in the worker thread.
Most of the per-point cost is the `AsyncScanPlugin` hand-off to the worker,
about 50 µs on the test system when a job is submitted on every point. Measure
your configuration with `performance_report: true` and compare the `plugins`
time with and without the plugin.
