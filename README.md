# AUAS Inspection Engine

Short overview and pointers for the AUAS Inspection Engine repository. This repository bundles several related applications used for automated inspection workflows:

- `scenario_inspector/` — Main orchestration application: automated inspection execution, database logging, and user management. See `scenario_inspector/README.md`.
- `scenario_creator/` — GUI to author and export YAML inspection scenarios compatible with the main engine. See `scenario_creator/README.md`.
- `ftp_server/` — Lightweight FTP server for development and artifact uploads. See `ftp_server/README.md`.
- `gui_application/` — Optional/legacy GUIs for per-system control (scanner, camera, gantry, table).

## Quick start

1. Pick the application you want to run and open its README.
2. Follow `QUICK_START.md` for a guided install if you want one-command setup.

## Where to run

- Main automation: `scenario_inspector/src/main.py`
- Scenario authoring: `scenario_creator/scenario_creator.py`
- FTP server: `ftp_server/start_server.py`

## Contributing

Create a virtual environment, install the app-specific requirements, and run the entry point for the application you work on. Open a PR and describe the change and tests.

## Authors and license

Maintained by the AUAS Engineering Team (Alexis Gordien). See application READMEs for more details and any licensing notes.
