# Scenario Creator

A GUI tool for classic users to create their own inspection scenarios for the AUAS Inspection Engine.

## Features
- User-friendly interface to build YAML scenario files
- Step-by-step creation: program info, stages, steps
- Dynamic forms based on selected system and available actions
- Exports valid scenario YAML files compatible with the main engine

## Usage
1. Launch the scenario creator GUI
2. Fill in program name and description
3. Add stages (with name, together, and steps)
4. For each step, select the system, action, and fill in required parameters
5. Save/export the scenario as a YAML file

## Development
- Built with Python and PyQt5 (same stack as main GUI)
- Reads available systems and actions from the main codebase for up-to-date options

---

**To get started, run:**
```bash
python scenario_creator.py
```
