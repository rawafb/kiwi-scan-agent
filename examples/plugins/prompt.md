# Notes on writing a prompt to create a new plugin

AI-generated code must always be tested by hand and fully understood, so that you own it.

- Give context and a technical description.
- Specify the unit tests.
- Specify the feature branch and PR workflow.

## General points to include

The prompt should point to the existing patterns, define the scope and boundaries (what not to touch), acceptance criteria (what means "done).

Examples:

- Read CONTRIBUTING.md first and follow it. Read docs/plugins.md and docs/async_plugins.md.
- Python >= 3.8. No new dependencies.
- Add unit tests without EPICS (fake ScanPluginContext)
- Run one scan end to end with a `type: sim` actuator and `detector_pvs: []` in a test.
- Document the changes
- Add a CHANGELOG entry under Unreleased.
- Run `make lint` and `make test`; everything must pass before you open the PR.
- Only add new files for the plugin, do not change public APIs, config keys, or existing YAML formats.
- Keep the plugin non-blocking: no long sleeps or blocking I/O in scan callbacks. Use the async hooks described in docs/async_plugins.md.
- In case of errors log a clear warning and let the scan continue (or fail cleanly). Never hang the scan.

