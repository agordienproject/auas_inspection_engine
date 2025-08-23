import sys
import os
import yaml
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QTextEdit, QPushButton,
    QCheckBox, QComboBox, QSpinBox, QDoubleSpinBox, QFileDialog, QGroupBox, QFormLayout, QMessageBox, QScrollArea
)
from PyQt5.QtCore import Qt

# Subclass widgets to always ignore wheel events
class NoWheelSpinBox(QSpinBox):
    def wheelEvent(self, event):
        event.ignore()

class NoWheelDoubleSpinBox(QDoubleSpinBox):
    def wheelEvent(self, event):
        event.ignore()

class NoWheelComboBox(QComboBox):
    def wheelEvent(self, event):
        event.ignore()

# Example system actions and parameters (should be loaded from real system modules)
SYSTEM_ACTIONS = {
    'camera': {
        'initialize_camera': {},
        'capture_image': {},
        'record_video': {
            'filename': 'str',
            'duration': 'int',
            'fps': 'int',
            'quality': 'combo',  # dropdown
            'codec': 'combo',    # dropdown
        },
        'stop_camera': {},
    },
    'scan_control': {
        'recording_data': {
            'param1': {'type': 'int', 'default': 1300},
            'param2': {'type': 'str', 'default': 'ex'},
            'recording_time': {'type': 'int', 'default': 20},
        },
    },
    'gantry': {
        'load_and_run': {'program_name': 'str'},
    },
    'table': {
        'rotate': {'duration': 'int'},
    },
    'xarm': {
        'move_to_position': {'x': 'float', 'y': 'float', 'z': 'float', 'roll': 'float', 'pitch': 'float', 'yaw': 'float'},
        'home': {},
    },
}

class StepWidget(QGroupBox):
    def __init__(self, parent=None, remove_callback=None):
        super().__init__(parent)
        self.setTitle("Step")
        self.layout = QFormLayout()
        self.setLayout(self.layout)

        self.name_edit = QLineEdit()
        self.layout.addRow("Step Name:", self.name_edit)

        self.system_combo = NoWheelComboBox()
        self.system_combo.addItems(SYSTEM_ACTIONS.keys())
        self.layout.addRow("System:", self.system_combo)

        self.action_combo = NoWheelComboBox()
        self.layout.addRow("Action:", self.action_combo)

        self.param_widgets = {}
        self.param_layout = QFormLayout()
        self.layout.addRow(self.param_layout)

        # Remove button
        self.remove_btn = QPushButton("Remove Step")
        self.remove_btn.clicked.connect(self._remove_self)
        self.layout.addRow(self.remove_btn)
        self._remove_callback = remove_callback

        self.system_combo.currentTextChanged.connect(self.update_actions)
        self.action_combo.currentTextChanged.connect(self.update_params)
        self.update_actions()

    def _remove_self(self):
        if self._remove_callback:
            self._remove_callback(self)

    def update_actions(self):
        system = self.system_combo.currentText()
        self.action_combo.clear()
        if system:
            self.action_combo.addItems(SYSTEM_ACTIONS[system].keys())
        self.update_params()

    def update_params(self):
        # Remove old param widgets
        for w in self.param_widgets.values():
            self.param_layout.removeRow(w)
        self.param_widgets = {}
        system = self.system_combo.currentText()
        action = self.action_combo.currentText()
        if system and action:
            params = SYSTEM_ACTIONS[system][action]
            for param, typ in params.items():
                # Support dict with type/default
                param_type = typ['type'] if isinstance(typ, dict) and 'type' in typ else typ
                default = typ.get('default') if isinstance(typ, dict) else None
                if param_type == 'bool':
                    widget = QCheckBox()
                    if default is not None:
                        widget.setChecked(bool(default))
                elif param_type == 'int':
                    widget = NoWheelSpinBox()
                    widget.setMaximum(100000)
                    if default is not None:
                        widget.setValue(int(default))
                elif param_type == 'float':
                    widget = QLineEdit()
                    widget.setPlaceholderText('float')
                    if default is not None:
                        widget.setText(str(default))
                elif param_type == 'combo':
                    widget = NoWheelComboBox()
                    # Set dropdown values for camera quality/codec/scan_control mode
                    if param == 'quality':
                        widget.addItems(['low', 'medium', 'high', 'ultra'])
                    elif param == 'codec':
                        widget.addItems(['mp4v', 'xvid', 'h264', 'mjpg'])
                    elif param == 'mode':
                        widget.addItems(['recording_data'])
                    if default is not None:
                        idx = widget.findText(str(default))
                        if idx >= 0:
                            widget.setCurrentIndex(idx)
                else:
                    widget = QLineEdit()
                    if default is not None:
                        widget.setText(str(default))
                # Add .mp4 label for record_video filename
                if system == 'camera' and action == 'record_video' and param == 'filename':
                    hbox = QHBoxLayout()
                    hbox.addWidget(widget)
                    ext_label = QLabel('.mp4')
                    ext_label.setStyleSheet('color: #888; margin-left: 2px;')
                    hbox.addWidget(ext_label)
                    container = QWidget()
                    container.setLayout(hbox)
                    self.param_layout.addRow(param + ':', container)
                else:
                    self.param_layout.addRow(param + ':', widget)
                self.param_widgets[param] = widget

    def get_data(self):
        data = {
            'name': self.name_edit.text(),
            'system': self.system_combo.currentText(),
            'action': self.action_combo.currentText(),
        }
        for param, widget in self.param_widgets.items():
            # For record_video filename, ensure .mp4 is appended if missing
            if data['system'] == 'camera' and data['action'] == 'record_video' and param == 'filename':
                val = widget.text()
                if val and not val.lower().endswith('.mp4'):
                    val += '.mp4'
                data[param] = val
            elif isinstance(widget, QCheckBox):
                data[param] = widget.isChecked()
            elif isinstance(widget, QSpinBox):
                data[param] = widget.value()
            elif isinstance(widget, QComboBox):
                data[param] = widget.currentText()
            else:
                val = widget.text()
                try:
                    if SYSTEM_ACTIONS[data['system']][data['action']][param] == 'float':
                        val = float(val)
                except Exception:
                    pass
                data[param] = val
        return data

class StageWidget(QGroupBox):
    def __init__(self, parent=None, remove_callback=None):
        super().__init__(parent)
        self.setTitle("Stage")
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        form = QFormLayout()
        self.name_edit = QLineEdit()
        form.addRow("Stage Name:", self.name_edit)
        self.together_check = QCheckBox("Run steps together")
        form.addRow(self.together_check)
        self.layout.addLayout(form)

        self.steps = []
        self.steps_layout = QVBoxLayout()
        self.layout.addLayout(self.steps_layout)

        add_step_btn = QPushButton("Add Step")
        add_step_btn.clicked.connect(self.add_step)
        self.layout.addWidget(add_step_btn)

        # Remove button
        self.remove_btn = QPushButton("Remove Stage")
        self.remove_btn.clicked.connect(self._remove_self)
        self.layout.addWidget(self.remove_btn)
        self._remove_callback = remove_callback

    def add_step(self):
        step = StepWidget(remove_callback=self.remove_step)
        self.steps.append(step)
        self.steps_layout.addWidget(step)

    def remove_step(self, step):
        self.steps_layout.removeWidget(step)
        self.steps.remove(step)
        step.setParent(None)

    def _remove_self(self):
        if self._remove_callback:
            self._remove_callback(self)

    def get_data(self):
        return {
            'name': self.name_edit.text(),
            'together': self.together_check.isChecked(),
            'steps': [s.get_data() for s in self.steps]
        }

class ScenarioCreator(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Scenario Creator")
        self.resize(900, 700)
        main_layout = QVBoxLayout(self)

        # Apply modern light theme and style
        self.setStyleSheet('''
            QWidget {
                background-color: #F7F7FA;
                color: #222;
                font-family: "Segoe UI", "Arial", "sans-serif";
                font-size: 15px;
            }
            QGroupBox {
                border: 1.5px solid #D0D0D0;
                border-radius: 12px;
                margin-top: 16px;
                background-color: #FFFFFF;
                padding: 12px;
            }
            QGroupBox:title {
                subcontrol-origin: margin;
                left: 16px;
                padding: 0 6px 0 6px;
                color: #1976D2;
                font-weight: bold;
                font-size: 17px;
            }
            QLabel {
                color: #333;
                font-size: 15px;
                background: transparent;
            }
            QLineEdit, QTextEdit, QComboBox {
                background: #FAFAFA;
                border: 1.5px solid #D0D0D0;
                border-radius: 8px;
                padding: 7px 10px;
                color: #222;
                font-size: 15px;
            }
            QLineEdit:focus, QTextEdit:focus, QComboBox:focus {
                border: 1.5px solid #1976D2;
                background: #FFFFFF;
            }
            QPushButton {
                background-color: #1976D2;
                color: #fff;
                border: none;
                border-radius: 8px;
                padding: 10px 18px;
                font-size: 15px;
                font-weight: 600;
                margin: 8px 0;
                outline: none;
            }
            QPushButton:hover {
                background-color: #1565C0;
                color: #fff;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
                color: #fff;
            }
            QPushButton:focus {
                border: 2px solid #1976D2;
            }
            QTabBar::tab {
                background: #E3E7EF;
                color: #222;
                border: 1px solid #D0D0D0;
                border-bottom: none;
                border-radius: 8px 8px 0 0;
                padding: 8px 20px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background: #FFFFFF;
                color: #1976D2;
                font-weight: bold;
            }
            QTabWidget::pane {
                border: 1px solid #D0D0D0;
                border-radius: 0 0 8px 8px;
                top: -1px;
            }
            QCheckBox {
                spacing: 8px;
                font-size: 15px;
            }
            QCheckBox::indicator:checked {
                background-color: #1976D2;
                border: 1.5px solid #1565C0;
            }
            QScrollArea {
                border: none;
                background: #F7F7FA;
            }
            QFormLayout > QLabel {
                min-width: 120px;
            }
        ''')

        # Program info
        form = QFormLayout()
        self.name_edit = QLineEdit()
        form.addRow("Scenario Name:", self.name_edit)
        self.desc_edit = QTextEdit()
        self.desc_edit.setFixedHeight(60)
        form.addRow("Description:", self.desc_edit)
        main_layout.addLayout(form)

        # Scroll area for stages
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_content = QWidget()
        self.stages_layout = QVBoxLayout(self.scroll_content)
        self.scroll_area.setWidget(self.scroll_content)
        main_layout.addWidget(self.scroll_area, 1)

        self.stages = []

        btn_row = QHBoxLayout()
        add_stage_btn = QPushButton("Add Stage")
        add_stage_btn.clicked.connect(self.add_stage)
        btn_row.addWidget(add_stage_btn)

        save_btn = QPushButton("Save Scenario as YAML")
        save_btn.clicked.connect(self.save_scenario)
        btn_row.addWidget(save_btn)
        btn_row.addStretch(1)
        main_layout.addLayout(btn_row)

    def add_stage(self):
        stage = StageWidget(remove_callback=self.remove_stage)
        self.stages.append(stage)
        self.stages_layout.addWidget(stage)

    def remove_stage(self, stage):
        self.stages_layout.removeWidget(stage)
        self.stages.remove(stage)
        stage.setParent(None)

    def save_scenario(self):
        # Remove tabs from description
        description = self.desc_edit.toPlainText().replace('\t', '').replace('\n', '\n')
        program = {
            'program': {
                'name': self.name_edit.text(),
                'description': description,
                # 'piece_info': ... # Add if needed
                'stages': []
            }
        }
        for stage_idx, stage in enumerate(self.stages, 1):
            stage_data = stage.get_data()
            steps = []
            for step_idx, step in enumerate(stage_data['steps'], 1):
                step_dict = {'step': step_idx}
                # For record_video, nest parameters under 'parameters'
                if step.get('system') == 'camera' and step.get('action') == 'record_video':
                    params = {k: v for k, v in step.items() if k not in ('step', 'name', 'system', 'action')}
                    step_dict['name'] = step.get('name', '')
                    step_dict['system'] = step.get('system', '')
                    step_dict['action'] = step.get('action', '')
                    step_dict['parameters'] = params
                else:
                    step_dict.update({k: v for k, v in step.items() if k != 'step'})
                steps.append(step_dict)
            program['program']['stages'].append({
                'stage': stage_idx,
                'name': stage_data['name'],
                'together': stage_data['together'],
                'steps': steps
            })
        fname, _ = QFileDialog.getSaveFileName(self, "Save Scenario", os.getcwd(), "YAML Files (*.yaml)")
        if fname:
            with open(fname, 'w', encoding='utf-8') as f:
                yaml.dump(program, f, sort_keys=False, indent=4)
            QMessageBox.information(self, "Saved", f"Scenario saved to {fname}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ScenarioCreator()
    win.show()
    sys.exit(app.exec_())
